# 球形机器人 PointGoal Benchmark V1 设计

## 1. 背景

当前球形机器人已经具备 Isaac Sim 数据采集、双目与本体状态世界模型、CEM 规划、闭环执行和
Success/SPL/Collision/Timeout 统计，但现有结果还不能稳定回答“机器人是否真正学会导航”。主要
问题包括：测试 context 数量少，起点和目标取自已有录制轨迹，SPL 参考不是地图上的最短可行路径，
轨迹图没有叠加完整障碍物，以及缺少一个经过人工确认、能够稳定完成任务的控制基线。

Benchmark V1 先解决实验基础设施问题。第一版只研究静态平面场景中的局部 PointGoal 导航，不以
BEV、风险头或世界模型结构作为前提。任何后续方法都必须在同一任务清单、输入权限、动作边界和
终止规则下评测。

## 2. 目标与非目标

### 2.1 目标

1. 建立一套由研究者逐项审核的简单场景 PointGoal 任务。
2. 用 Oracle A* 和统一球形机器人路径跟踪器证明任务、机器人动力学和评测链路可行。
3. 建立不使用在线全局地图的经典视觉基线，并为 iPlanner 等开源方法提供统一适配接口。
4. 统一保存配置、视频、轨迹、原始 episode 指标和汇总置信区间。
5. 支持本地 GUI 审核和服务器 headless 批量运行，并验证两种运行方式的一致性。
6. 防止初始即成功、空旷场景随机到达、测试集调参、失败 episode 被事后删除等实验偏差。

### 2.2 非目标

- 第一版不加入动态障碍、复杂高程地形、长期记忆、拓扑图或全局 SLAM 地图输入。
- 第一版不实现新的风险头、价值头或世界模型结构。
- Oracle A*、仿真真值深度和真值语义只属于特权诊断，不作为正式无地图方法的在线输入。
- 第一版不要求一次性移植 ViPlanner、NavDP 和 FDM；它们按依赖和公平性逐步加入。
- 当前已在历史研究中使用的 `cluttered_easy` 场景不能被描述为最终完全未见测试集。

## 3. 固定任务协议

### 3.1 在线输入

核心公平赛道中的策略只能读取：

- 同步双目 RGB；
- 当前及历史本体状态；
- Isaac Sim 提供的完美 world-frame 位置和 yaw；
- 当前 PointGoal 在机器人 base frame 下的二维向量；
- 历史已执行动作。

策略不能读取全局障碍地图、未来 RGB、未来状态、目标位置附近图像、Oracle A* 路径或仿真真值
深度。PointGoal 由固定 world-frame 目标和当前仿真真值位姿计算，不依赖环境地图。
该假设命名为 `perfect_localization`，必须写入每个结果的输入权限。真机阶段再将同一接口替换为
GPS/里程计/yaw 融合位姿；里程计漂移、定位噪声和定位失效不属于 V1 的研究变量。

### 3.2 时间和动作

- 物理与控制频率保持 50 Hz；
- 双目和策略重规划频率保持 10 Hz；
- 每个策略输出一个 10 维 action block，表示连续五个 50 Hz 的
  `[main_joint_velocity_cmd, auxiliary_joint_angle_cmd]`；
- 主驱动边界为 `[-2.0, 2.0] rad/s`；
- 辅助关节硬边界为 `[-0.5235987756, 0.5235987756] rad`，即左右各 `±30°`；
- 采集器的默认几何最小转弯半径为
  `0.4 / tan(30°) = 0.692820323 m`，使动作生成、随机探索和规划都能覆盖完整关节范围；
- 规划器每次只执行首个 action block，然后使用新的真实观测重新规划；
- 正式简单任务时间预算为 20 秒，goal tolerance 为 0.2 m。

资产中 `joint2` 的 USD 物理限位为约 `[-30.00007°, 30.00007°]`，控制索引为 1。
`0.692820323 m` 是动作生成器使用的几何值，不能直接声称为球形机器人的真实动力学转弯半径。
在正式采集前，必须完成低速 `±30°` 转向测试，核对关节跟踪、实际轨迹、侧倾/振荡和制动。

新协议标识为 `ball_action_v2_aux30`，动作边界只能从数据契约读取，禁止在采集、训练、
CEM 和评测器中复制常量。旧数据和旧 checkpoint 保留原 `±16.1°` 契约并标记为 legacy，
不得直接在 `±30°` 动作空间中评测。新数据集必须报告辅助关节动作直方图，并确认
`16.1° < |auxiliary| <= 30°` 区间有实际样本覆盖。

### 3.3 终止规则

episode 的终止原因互斥，判定优先级固定为：

1. `collision`；
2. `success`；
3. `timeout`；
4. `runtime_error` 作为执行基础设施失败单独记录。

正式碰撞标签使用真值二维占据几何与球体 footprint 的 clearance 判定；物理 contact 同时记录为
审计字段。两者长期不一致时必须停止正式评测并检查碰撞几何，不能由单个方法选择更有利的定义。

## 4. 场景和 episode 分层

### 4.1 简单任务类型

第一版只包含三类静态平面任务：

| 类型 | 定义 | 目的 |
| --- | --- | --- |
| E0 开放目标 | 起终点直线在膨胀障碍图上可通行 | 验证 PointGoal 响应、动作方向和停止 |
| E1 单障碍绕行 | 起终点直线与膨胀障碍相交，但存在简单左右绕行路线 | 验证真正视觉避障 |
| E2 通道通过 | 最短路径经过宽度有限但可通行的门口或通道 | 验证 clearance 和控制精度 |

任务生成使用从机器人资产读取并写入 manifest 的球体半径 `r_ball`，以及 V1 固定安全裕度
`m_safe = 0.20 m` 膨胀障碍。起点、终点和路径检查必须来自同一张冻结真值占据图。

### 4.2 数量和用途

- `golden`：3 个 episode，E0/E1/E2 各一个，用于本地 GUI 和服务器一致性检查；
- `dev`：30 个 episode，三类各 10 个，可用于调试和参数选择；
- `test`：60 个 episode，来自至少 6 个未用于训练和调参的新简单场景，三类均衡；
- 后续大型统计集不属于 V1 首次实现范围，必须在 V1 通过后另行发布版本。

现有 `easy_0` 至 `easy_9` 只作为 golden/dev 候选。正式 test 必须新建或明确保留未被任何训练、
checkpoint 选择和人工调参使用的场景。

### 4.3 自动有效性条件

任务进入人工审核前必须满足：

- 初始目标距离位于 `[1.5, 4.0] m`；
- 起点和终点均无碰撞且具有规定 clearance；
- 起始状态不在 goal tolerance 内；
- 膨胀占据图上的 A* 路径存在；
- 起始相机没有位于障碍内部或被近距离表面完全遮挡；
- E0 的直线路径无障碍；
- E1 的直线路径必须与障碍或安全膨胀区相交；
- E1 的 A* 长度与欧氏距离之比至少为 1.15；
- E2 的 A* 路径必须经过标注的通道区域，通道宽度位于
  `[2*r_ball + 2*m_safe, 2*r_ball + 2*m_safe + 0.40 m]`；
- 任务数据全部有限且场景、起点、目标和地图哈希一致。

若场景几何无法满足某类阈值，该任务不得通过修改类别名称绕过验证。

## 5. 冻结 manifest

每个任务使用独立 `benchmark_episode_v1` 记录，至少包含：

```json
{
  "schema_version": "ball_pointgoal_benchmark_v1",
  "episode_id": "easy_e1_0001",
  "split": "dev",
  "difficulty": "E1",
  "scene_uri": "${SCENE_ROOT}/benchmark_easy/scene_01.usd",
  "scene_sha256": "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
  "occupancy_sha256": "abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789",
  "start_position_world_m": [0.0, 0.0, 0.0],
  "start_quaternion_world_wxyz": [1.0, 0.0, 0.0, 0.0],
  "goal_position_world_m": [2.5, 0.8, 0.0],
  "euclidean_distance_m": 2.625,
  "oracle_path_length_m": 3.18,
  "seed": 3072,
  "human_review": {
    "status": "approved",
    "reviewer": "project_owner",
    "notes": "single-obstacle left/right detour"
  }
}
```

manifest 不保存主机绝对路径。`${SCENE_ROOT}` 在运行时由 CLI 或环境配置解析。发布后保存 manifest
SHA256；同一 benchmark 版本禁止就地修改。

## 6. Baseline 分层

### 6.1 健全性基线

- `stop`：持续零动作；
- `random`：在统一动作边界内使用固定种子采样；
- `goal_greedy`：只朝 PointGoal 方向前进，不使用障碍观测。

它们用于检查成功判定、任务难度和“随机走到目标”的概率，不作为主要竞争方法。

### 6.2 Oracle 上界

Oracle 使用真值占据图：

1. 按 `r_ball + m_safe` 膨胀障碍；
2. 在 0.1 m 网格上运行 A*，生成 reference shortest path；
3. 对可执行路径做独立平滑，但 SPL 仍使用未平滑的最短网格路径长度；
4. 通过统一 Pure Pursuit/PID 跟踪器转换为球形机器人 action block；
5. 每个控制周期仍使用真实位姿闭环跟踪。

Oracle 的地图和参考路径必须标记为 `privileged=true`，不能出现在核心公平赛道。

### 6.3 经典无地图基线

主要经典 baseline 为：

```text
stereo RGB -> stereo depth -> robot-centric local occupancy -> DWA/trajectory rollout
           -> shared spherical tracker -> action block
```

另保留 `oracle_depth_local_planner` 作为感知上界。它和 stereo 版本使用相同局部规划器与跟踪器，
两者差值用于定位双目深度误差。

### 6.4 开源学习方法

首批外部方法按以下顺序接入：

1. iPlanner：前向 depth + local goal，优先适配；
2. ViPlanner geometric-only：先评估深度路径，语义版本单独标记输入权限；
3. NavDP：作为视觉 PointGoal 强基线；
4. FDM：作为动作条件未来状态和风险规划的后续方法学对照。

官方项目先在其原生示例中验证，再接入球形机器人。官方 checkpoint 结果标记为
`native_checkpoint_transfer`；使用本项目数据重训后才标记为 `fair_retrained`。ViPlanner/FDM 的
代码许可和再分发限制在引入仓库前单独核查，外部代码优先通过独立 checkout 或固定 submodule
引用，不能直接复制后丢失来源和版本。

官方来源：

- iPlanner: https://github.com/leggedrobotics/iPlanner
- ViPlanner: https://github.com/leggedrobotics/viplanner
- NavDP: https://github.com/InternRobotics/NavDP
- FDM: https://github.com/leggedrobotics/fdm

### 6.5 当前研究方法

- `pose_only_wm_cem`；
- `pose_bev_wm_cem`，仅作为已有方法消融；
- 后续 `wm_risk_value`，不属于 V1 首次实现。

## 7. 统一策略接口

正式评测器只依赖一个策略适配协议：

```python
policy.reset(episode_context)
action_block = policy.act(observation, pointgoal_base_xy)
```

`observation` 包含当前同步双目、本体历史、完美仿真位姿和动作历史；返回值必须是有限且位于契约边界内
的 `(10,)` action block。路径输出型方法通过共享球形机器人 tracker 转换动作，直接动作方法通过
边界校验后执行。

评测结果同时标记：

- `online_input_permissions`；
- `privileged`；
- `native_checkpoint_transfer` 或 `fair_retrained`；
- 是否使用共享 tracker；
- checkpoint 和外部代码版本。

不把输入权限不同的方法放进同一个无说明排名。

## 8. 人工审核协议

### 8.1 运行前任务审核

每个 episode 生成任务卡，必须叠加：

- 真值障碍与安全膨胀区；
- 起点、初始朝向和目标；
- 起终点直线；
- Oracle A* 路径；
- 欧氏距离、A* 距离、绕行比和起终点 clearance；
- E0/E1/E2 分类。

项目负责人将任务标记为 `approved`、`rejected` 或 `needs_adjustment`。只有 approved 任务进入
冻结 manifest。

### 8.2 运行后行为审核

正式策略运行后检查机器人是否响应目标、主动避障、擦碰、卡住、振荡，以及终止判定是否与画面
一致。运行后审核只能增加行为标签和技术错误说明，不能因方法失败删除 episode。只有场景加载、
传感器、计时或保存损坏等基础设施错误可以触发整组重跑，且必须保留错误记录。

### 8.3 反随机验证

V1 至少包含：

- 同起点的左、前、右多目标测试；
- 同起终点的有障碍/无障碍配对测试；
- 相同 episode 的多个规划随机种子；
- random 和 goal-greedy 对照。

有效方法的轨迹应随目标与障碍发生可解释变化，不能只依靠一次偶然成功。

## 9. GUI、视频和审核包

运行器提供显式 `--gui`、`--headless` 和 `--record_video`。GUI 至少显示：

- 起点、目标和 Oracle 参考路径；
- 实际轨迹、当前候选轨迹和选中轨迹；
- 当前目标距离、clearance、动作和终止状态。

每个正式 episode 保存：

```text
episode_<id>/
  manifest.json
  environment.json
  task_card.png
  third_person.mp4
  stereo_view.mp4
  trajectory_overlay.png
  actions.npz
  states.npz
  metrics.json
  human_review.json
```

视频渲染不能改变传感器 cadence、物理步长或策略输入。轨迹图必须叠加真值障碍、A* 路径、起点、
目标、终点和碰撞点，不能只画无背景折线。

## 10. 评测指标

主要指标：

- Success Rate；
- Collision Rate；
- SPL，reference 为膨胀地图上的 Oracle 最短路径；
- Timeout Rate。

辅助指标：

- Final Distance；
- Path Length 和 Path/Oracle Ratio；
- Goal Progress；
- Minimum Clearance；
- Time to Goal；
- 动作平滑度、停止次数和控制幅值；
- Runtime Error Rate。

结果按 policy、E0/E1/E2 和 scene 分层汇总。正式置信区间按 scene 做 cluster bootstrap，不能只把
同一场景中的 episode 当作完全独立样本。每个汇总值必须能回溯到逐 episode JSON。

## 11. 本地与服务器执行

### 11.1 职责

本地主机负责：

- golden episode 的 GUI 调试；
- 任务卡审批；
- 服务器生成的视频和轨迹审核；
- 可疑 episode 的交互式重放。

服务器负责：

- Isaac Sim headless 批量执行；
- Oracle 和候选轨迹采集；
- 多 baseline 正式评测；
- 模型训练；
- 生成完整审核包。

### 11.2 可复现环境

每次正式运行记录：

- Git commit；
- Isaac Sim、Python、PyTorch、CUDA、驱动和 GPU；
- 容器 digest 或 Conda/pip 锁定信息；
- scene、occupancy、manifest 和 checkpoint SHA256；
- 随机种子、物理参数、动作边界和策略配置。

代码和小型配置通过 Git；USD 资产、数据集、视频、checkpoint 和 runs 通过 rsync、共享磁盘或对象
存储。运行时路径通过 `SCENE_ROOT`、`DATA_ROOT`、`RUN_ROOT` 等配置解析，不允许正式配置依赖
`/home/an/...` 绝对路径。

### 11.3 Golden parity

三个 golden episode 必须分别在本地 GUI 和服务器 headless 运行。两端要求：

- manifest、scene、policy 和 seed 哈希一致；
- 起点目标完全一致；
- 动作与轨迹在预定义数值容差内一致；
- success/collision/timeout 终止原因一致；
- 视频和 headless 不改变控制时序。

GPU 物理和渲染不要求逐位相同。V1 对确定性 Oracle baseline 要求起点和目标坐标误差不超过
`1e-6 m`、10 Hz 轨迹 XY RMSE 不超过 `0.05 m`、终点差不超过 `0.10 m`、路径长度相对差不超过
2%，并且终止原因相同；超出任一容差时不得开始服务器正式批量任务。

## 12. 测试和错误处理

纯 Python 测试覆盖：

- manifest schema、路径解析和哈希校验；
- `ball_action_v2_aux30` 的单一契约来源、十维展开和 legacy checkpoint 拒绝逻辑；
- E0/E1/E2 自动分类条件；
- 障碍膨胀、A* 可达性和 reference length；
- strategy adapter 的 shape、finite 和动作边界；
- collision/success/timeout 优先级；
- 任务审批状态与冻结规则；
- 按 scene 的 bootstrap 汇总；
- 审核包完整性。

Isaac smoke 覆盖三个 golden episode。episode 写入先使用 `.partial`，所有数组、JSON、视频和哈希
通过检查后原子发布。服务器中断后只能继续未完成任务，不能覆盖已完成正式结果。
此外，一个独立 Isaac smoke 必须在低速下分别命令 `+30°` 和 `-30°`，验证 `joint2` 无越界且
轨迹左右符号正确；该测试通过前不得采集 V1 正式数据。

## 13. V1 准入门槛

以下门槛是工程准入条件，不是论文通用标准：

1. 3 个 golden 任务均经人工批准并由 Oracle 成功完成；
2. 30 个 dev 任务全部完成运行前人工审核；
3. Oracle 在 dev 上成功率至少 95%、碰撞率不高于 2%、runtime error 为 0；
4. E1/E2 视频存在明确绕障或通道通过行为；
5. 在 E1/E2 合集上，Oracle Success Rate 必须分别比 random 和 goal-greedy 高至少 30 个百分点；
6. stereo local planner 在 E0 上成功率至少 80%、E1 上至少 60%，两类合计碰撞率不高于 10%，
   runtime error 为 0；
7. GUI/headless golden parity 通过；
8. 正式 test manifest 冻结后不再用于调参或 checkpoint 选择。

若 Oracle 未通过，停止学习方法开发并检查场景、A*、tracker、动作符号和终止判定。若 Oracle
通过但 stereo baseline 失败，先区分深度感知和局部规划；若经典 baseline 通过而世界模型失败，
再进入候选排序、风险头和价值学习。

## 14. 实施阶段

1. 清理并参数化当前运行路径，定义 V1 schema 和 policy adapter；
2. 创建 E0/E1/E2 golden tasks、任务卡和冻结流程；
3. 实现占据膨胀、A* reference 和共享 tracker；
4. 增加 GUI/headless、视频和障碍叠加轨迹输出；
5. 完成本地 golden 人工审核；
6. 迁移服务器并通过 golden parity；
7. 构建 30-episode dev，并运行 sanity/Oracle/stereo local baselines；
8. 按顺序适配 iPlanner 和其他外部方法；
9. 创建未见场景 test，仅在方法和协议冻结后运行。
