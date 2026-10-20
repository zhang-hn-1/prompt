# Dynamic Oracle V2：全局路径软引导动态格点搜索设计

## 背景与结论

冻结任务 `hard_0_e1_0002` 在真实 RTX 3060 / Isaac Sim 环境中未进入动作循环；
动态格点搜索在 60 秒墙钟上限返回 `timeout`。该地图为 `620 × 620`、`0.1 m`
栅格，机器人安全膨胀后起终点仍在同一自由连通域；静态 A* 路径长 `12.791 m`，
包含多次转向。问题是动态状态格点在长而紧凑的绕行上扩展过多，不是 GPU、地图
哈希、定位输入或二维连通性错误。

本设计仅增强特权基线 `oracle_v2`。未来的双目世界模型和不持有全局地图的局部
导航策略不接收该路径、距离场或栅格。

## 目标与非目标

目标：在不改变碰撞检查、机器人动力学、动作限速、32 秒预测时域、60 秒规划
上限和冻结 Benchmark 的前提下，使 `oracle_v2` 能在 E1 任务上找到一条连续、
动力学可执行的轨迹。

非目标：不把 Oracle 的地图信息输入世界模型；不更改 E1 起终点、占据图、控制器
PID、成功判定或 `oracle` V1；不承诺动态规划路径是全局最短路径。

## 决策

动态规划器改用冻结膨胀栅格上的确定性 8 邻域 A* 路径作为**软引导**。这条路径
只改变搜索队列的优先级，绝不把节点限制在走廊中，也不跳过任何 50 Hz 碰撞检查、
风险代价或换向停稳约束。

相较于保留全图 Dijkstra 距离场并尝试构造 yaw 可采纳下界，此选择直接利用 Oracle
已经被授权使用的全局占据图，在紧凑绕行上给出更具体的路线提示，并消除当前
Python 全图 Dijkstra 约 2.5 秒的预处理成本。它是有意的非可采纳引导搜索；因此
产物必须记录版本和引导参数，不能把结果描述成最优解。

## 数据流与接口

`IsaacBenchmarkRuntime.dynamic_plan()` 仍向 `DynamicPlanRequest` 提供同一份冻结、
安全膨胀 occupancy，reset 后的物理初态和 episode goal。它额外在同一栅格上调用
已有的 `grid_planning.astar_path(...)`，得到 `reference_cells: (K, 2)`；该 A* 失败
时动态计划直接为 `unreachable`，不回退 V1、直线控制或 GoalGreedy。

`DynamicPlanRequest` 新增可选的、不可变的 `guidance_cells` 和
`guidance_weight: float`。默认 `None` 保持当前纯动态格点行为，供已有单元测试和
无特权调用使用。`oracle_v2` 总是传入 guidance。

规划器把 `guidance_cells` 变换成两个固定字段：

1. `guidance_distance_m`：任一自由栅格到最近 reference cell 的欧氏距离；
2. `guidance_remaining_m`：最近 reference cell 沿 reference 到 goal 的剩余累计长度。

该字段使用 `scipy.ndimage.distance_transform_edt(..., return_indices=True)` 一次性生成；
reference cell 的剩余长度由相邻 8 邻域中心距离累加。对 state cell 的引导值为：

`H_guided = guidance_remaining_m[cell] + guidance_weight × guidance_distance_m[cell]`。

冻结 `guidance_weight = 0.35`，单位无量纲。它只在 heap priority 中使用：

`priority = accumulated_cost + H_guided`。

`accumulated_cost`、best-label 支配关系、控制目标枚举、primitive 重放、碰撞判断和
最终 `DynamicPlanResult.total_cost` 均完全不变。权重不为零时该 priority 不再是
可采纳启发式，因此首次到达 goal 的解不是全局最优保证；它是可复现的 privileged
Oracle 参考轨迹。

若 guidance 缺失，保持现有 `_grid_distance_to_goal` 优先级。若 reference cell
不在地图内、落在障碍上、数组不为 `(K, 2)` 整数且 `K >= 1`，请求构造即报错；若
reference 末端不是 goal cell，亦报错。所有字段冻结为只读数组。

## 运行时和审计边界

`oracle` V1 和非特权 PolicyContext 继续没有动态 reference 或 occupancy 权限。
`oracle_v2` 的 environment / certification 新增：

- `planner_version = "dynamic_lattice_v2_guided"`；
- `guidance_mode = "static_astar_soft_priority"`；
- `guidance_weight = 0.35`；
- `guidance_path_length_m`；
- `guidance_cell_count`；
- guidance cell 的 SHA-256。

这些字段仅证明 Oracle 使用了何种特权引导，不写进 PolicyObservation、训练数据或
世界模型输入。V1 bundle 仍不得要求这些字段。

## 失败语义

静态 A* 无路径：返回 `unreachable`。动态搜索在 60 秒或节点预算内未完成：分别返回
`timeout` 或 `node_budget_exhausted`。两类失败不得发布 success bundle，也不得切换到
V1 或任何直线后备控制。E1 gate 继续要求：计划 success、零碰撞、执行不超过 35 秒、
最大横向误差小于 0.2 米、参考主轴/副轴速率分别不超过 `0.5 rad/s²` 与 `30°/s`。

## 验证策略

1. 单元测试验证 guidance request 的形状、只读性、末端 goal、障碍拒绝和 V1 默认
行为；固定小图上验证无 guidance 与显式 `None` 结果逐位相同。
2. 固定绕障图验证 guidance 的队列扩展数低于无引导版本或在相同短墙钟内成功；完整
输出在相同输入下严格确定。
3. runtime contract 验证仅 `oracle_v2` 构造 A* guidance，V1 不调用该路径。
4. artifact 测试验证 V2 environment/certification 引导元数据，V1 schema 不回归。
5. 先以冻结 E1 静态 occupancy 运行纯 Python 预门禁；成功后再原样重跑真实 Isaac
单任务 Gate。失败时记录状态和搜索统计，停止七任务评测。

## 风险与缓解

- 静态 A* 中心线可能贴近障碍：引导是软优先级，动态搜索可离开中心线；所有候选仍
 经过完整膨胀栅格碰撞检查。
- 非可采纳优先级可能产生更长路径：审计记录它是 guided Oracle，并比较路径长度、
 方向切换和执行成功率，不声称最优性。
- 权重过小可能仍超时，过大可能贴线：固定为 `0.35`，先做纯 Python E1 预门禁；若
 失败才做单独的、可审计参数实验，不在 E1 结果后临时调参。
