# Dynamic Oracle V2：动力学可执行参考轨迹 warm-start 设计

## 背景与问题定位

冻结任务 `hard_0_e1_0002` 使用经过球半径 `0.4 m` 与安全裕度 `0.1 m` 膨胀后的真实
占据图（SHA-256：`547dd73c52b45f9cc1aa8dca561c779349391f6ecef06ef59026921284b467da`）。
起点和终点在同一自由连通域；确定性 8 邻域 A* 给出 114 格、12.791 m 的绕障路径。

此前的 V2 软引导方案只把这条 A* 路径加入动态格点搜索的 heap priority：
`g + remaining + 0.35 × lateral_distance`。它严格保留了碰撞、代价、动作和状态标签，
但纯 Python 预门禁在 60 秒、60,440 次扩展后仍为 `timeout`。调大引导权重、放大剩余距离，
或加入固定前视项均没有在同一约束下找到可行解。这说明瓶颈不是“静态路线不知道往哪走”，
而是搜索要从与第一段静态路线不对齐的初始 yaw 出发，同时枚举大量动作、速度、转向、时间
标签，才有机会构造一条可执行起步动作。

因此本设计取代未完成的软 priority 实验。它仍只服务于持有真值地图的 `oracle_v2`，不会把
占据图、A*、参考轨迹或任何额外信息提供给双目世界模型、训练数据或非特权局部策略。

## 目标与边界

目标是在不修改冻结 benchmark 的场景、起终点、膨胀规则、PID、成功判定、机器人动力学、
50 Hz 碰撞采样、32 秒运动时域或 60 秒规划硬上限的前提下，为 Oracle V2 先构造一条真正能
执行的、参考全局路径的控制轨迹。E1 的 Python 静态预门禁必须成功后，才允许启动 Isaac Sim
单任务 Gate。

非目标如下：

- 不声称该轨迹是全局最短或代价最优；它是可复现的特权可行性参考。
- 不做直线、静态 A* 中心线、瞬时转向或 V1 控制器的后备动作。
- 不把参考路径当作动态搜索的硬走廊；若参考轨迹构造失败，后续搜索依旧可以离开该路径。
- 不以手工修改 E1 地图、降低障碍膨胀、提高运动时间或放宽终止条件换取成功。

## 方案选择

考虑过三种方案：

1. 继续调软引导权重或前视距离：实现最小，但诊断已证明多个数量级的权重都未能跨过 E1，
   且会把问题变成不可审计的参数搜索。
2. 对静态 A* 施加强制走廊或直接跟踪中心线：搜索快，但会隐藏球的转向/制动约束，窄通道中
   可能产生碰撞，也不能证明真实控制可执行。
3. 先用同一套物理 primitive 沿 A* 做小束宽（beam）的动力学参考 rollout，再把它作为可行
   incumbent 与搜索 warm-start：每一段都已通过相同的碰撞与动作约束；若成功可以立即返回，
   若失败再保留不受走廊限制的动态格点搜索。

采用方案 3。它直接处理“从当前朝向如何实际接上路线”，并把 Oracle 的静态地图知识限制在
候选动作排序，而不是替代控制或碰撞验证。

## 架构与数据流

`IsaacBenchmarkRuntime.reset()` 在原有 A* 后保留只读的 `reference_cells`，并仅在 active
episode 下通过 defensive-copy 的 `oracle_path_cells(episode)` 访问。`dynamic_plan()` 把该路径
作为 `DynamicPlanRequest.guidance_cells` 传给 Oracle V2；V1、PolicyContext 和训练管线均没有此
接口或数据。

`plan_dynamic_lattice()` 的 V3 路径如下：

1. 运行原有请求、起点、目标、occupancy、距离场和时间上限验证。
2. 有 `guidance_cells` 时，执行 `build_reference_rollout()`。它从真实的 `DynamicState` 开始，
   每秒枚举现有的 `valid_control_targets()`，调用相同的 `propagate_dynamic_primitive()`，并对每段
   50 Hz 样本调用当前碰撞检查和动态边代价函数。
3. rollout 维护固定宽度 `8` 的确定性 beam。对存活候选，以其端点在 A* 上的最近非回退投影，
   按“沿参考的剩余距离、横向偏离、已累计真实代价、稳定的枚举顺序”排序。完成 dominance
   过滤后，先从非空的 `forward`、`reverse`、`stopped` endpoint 方向类各保留一个最佳候选，
   再按原全局排序填满剩余 beam 槽位，beam 总宽度仍为 8。横向距离只用于排序，不能拒绝一条
   合法但离开静态中心线的轨迹。
4. 当任一 beam 候选同时位于目标 `0.2 m`、速度低于 `0.05 m/s`、主轴命令绝对值不大于
   `0.05 rad/s` 时，使用已保存的真实 primitive 链重建 `DynamicPlanResult`，标记为可行
   incumbent。返回的每个 waypoint 均来自 50 Hz propagation，不是插值 A* 点。
5. 若 reference rollout 在 32 秒内没有达成终止条件，它的所有通过碰撞验证的末端状态以原始
   parent/primitive/cost 作为 warm-start 节点加入动态格点搜索。搜索从这些节点和根节点共同
   开始，但不引入路径格禁止条件；仍可绕开 reference。若 rollout 产生了可行 incumbent，
   则它是超时时的有效返回值，而不是把已有可执行轨迹丢弃后报告 `timeout`。
6. 若 rollout 连合法候选也构造不出，或随后动态搜索也无法完成，保持现有的
   `unreachable`、`node_budget_exhausted`、`timeout` 语义；不得回退 V1 或直线控制。

整个流程共享 primitive 缓存、限速、换向停稳、碰撞采样、风险代价和结果重建逻辑，避免两个
规划器对“可执行”给出不一致的定义。

## 候选排序与确定性

参考 rollout 不使用“副轴角度等于航向误差”的简化控制律。副轴命令、主轴加减速、yaw 变化与
位移均由已标定的 `DriveModel` 和当前动力学状态 rollout 得到；评分只观察 rollout 后的真实
状态。这避免把辅助轴延迟误当成立即航向控制。

每个 beam 节点记录其最近的参考进度索引。子节点的索引不得小于父节点，以防 A* 路线相邻处
的最近格抖动导致排序倒退，但这不是空间走廊约束：子节点可位于任意空闲格。若多个候选得分
相同，按累计代价、目标值的既有确定顺序和递增创建序号打破平局。相同输入的 controls、
waypoints、状态和审计哈希必须逐位确定。

束宽 8 是固定公开的 benchmark 配置，而不是针对单个执行结果临时调参。它在每层保留多个
“接入路线”的动力学状态，同时避免旧格点搜索的数万时间标签爆炸。若 E1 预门禁未通过，只能
在新实验记录中比较束宽与评分项；不得暗中修改冻结任务或混入结果。

## 状态、结果和审计

内部节点新增最小必要字段：参考进度索引与 `source`（`root`、`reference_rollout` 或
`lattice`）。`DynamicPlanResult` 保持既有控制/waypoint 语义；如需增加字段，必须是带默认值的
审计字段，不能破坏已有位置参数调用。

`oracle_v2` bundle 的 planner metadata 更新为：

- `planner_version = "dynamic_lattice_v3_reference_rollout"`；
- `reference_mode = "static_astar_dynamics_beam_warm_start"`；
- `reference_beam_width = 8`；
- `reference_cell_count`、`reference_path_length_m` 与 reference SHA-256；
- `reference_rollout_status`（`goal_incumbent`、`seeded_search` 或 `no_valid_seed`）；
- `reference_rollout_expanded`、最终 `plan_source`、路径长度、时长和方向切换数。

这些字段只审计 Oracle 的特权来源，不进入 PolicyObservation、模型 loss 或数据集。V1 artifact
schema 继续不要求它们。

## 失败与安全语义

静态 A* 无路径仍为 `unreachable`；起终点非法仍按既有状态返回。reference rollout 的候选在每个
primitive 上执行全样本碰撞检查，任何越界或占据样本立刻被剔除。它不允许穿越静态 A* 的障碍
格、越过球安全膨胀、跳过换向静止时间或用未建模命令达到目标。

若已得到满足终止条件的 incumbent，之后的搜索超时可以返回该 incumbent，因为它已经是经同一
动力学和碰撞验证的完整计划；审计明确标识其来源，不把它说成 A* 最优解。若没有 incumbent，
则不能把“部分靠近目标”的 reference 轨迹伪装成 success。

## 验证策略与完成标准

1. 先写失败测试：确定性开阔绕障图中 reference rollout 能从错位 yaw 产生已停稳的 goal
   计划；制造碰撞的看似高进度 primitive 必须被拒绝；绕开参考线的安全 primitive 仍可保留；
   同输入输出逐位相同。
2. 单元测试确认 rollout 与常规 lattice 使用同一动作集、50 Hz 样本、速率/换向约束与边代价，
   并确认无 guidance 时 legacy 行为不变。
3. 冻结 E1 纯 Python 预门禁必须从仓库真实 occupancy、manifest、初始 yaw 和 `.5 m` 膨胀
   重建请求，断言 `success`、非空有限 waypoints、`duration_s <= 32`、零占据碰撞和严格确定。
   该测试是运行 Isaac 前的硬门。
4. runtime/CLI/artifact 测试确认仅 Oracle V2 能读取 reference cells，且 audit 元数据完整；V1
   schema 不回归。
5. 只有步骤 1–4 全部通过，才在真实 GPU/Isaac Sim 跑 `hard_0_e1_0002`。它必须产生完整 bundle：
   metrics、state、trajectory、视频/帧、零碰撞、35 秒执行预算内到达、最大横向误差 `< 0.2 m`。
   任一失败都停止后续扩展 benchmark，不发布成功率结论。

## 风险与缓解

- 静态路线在紧通道转角处不可直接跟踪：beam 保留多个动力学接入状态，且所有状态都经过真实
  rollout；必要时搜索从安全末端继续，不把中心线当控制命令。
- 特权规划可能掩盖局部导航困难：版本、reference 哈希、来源和 Oracle 边界都写入 artifact；
  它只作为上界/benchmark 基线，与未来无图世界模型分开报告。
- beam 本身可能找不到 goal：保留无硬走廊的 warm-start 搜索与明确失败状态，而不是强行生成
  一条不安全的轨迹。
- 计算量仍可能过高：预门禁记录 reference 展开数、search 展开数与墙钟；先改善纯 Python
  证据，再申请 GPU/Isaac 资源。
