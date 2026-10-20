# 球形机器人跟踪器第一阶段验收设计

## 已批准的有界后续诊断（首轮实测之后）

0.5→1.0 m 前视距离单变量实验的八项结果显示：名义到达由 6/8 增至 7/8，
额外停稳组合验收由 1/8 增至 4/8；前进直线摆幅显著下降，但圆弧和换向仍未全部通过。
先追加同版本 0.5 m 直线 control，检查初始预热差异是否足以解释改善。
若对照仍复现原版振荡，则只允许再试一个预定值 1.5 m，仍使用同一八项配置、PID、
速度、预算和验收规则。这是有界的增益/相位稳定性试验，不进行无上限参数搜索，
不改变正式默认值，也不把诊断到达比例称为正式障碍导航成功率。

## 目的与边界

本阶段只回答一个问题：在没有障碍、没有 Hybrid A* 参与的条件下，现有
`OraclePathPolicy` 能否把一组可复现的几何参考轨迹转换成实际可停稳的球形机器人运动。
它是后续收紧 Hybrid A* 可执行范围和重新跑 7 个正式任务的前置 gate，不是正式
`ball_pointgoal_v1_1_35s` 评测，也不修改冻结的 `hard_0` 地图、任务、起始姿态或
35 秒协议。

已知事实作为基线记录，不重新解释：当前真实 35 秒/7 task baseline 为 `0/7`
（6 次 grid collision、1 次 timeout）；此前的 `4/7` 结果受错误的 `1.8` 时间基污染，
不能作为控制改进对照。主轴加速度限制 `0.5 rad/s²`、主轴 `D=80`、副轴
`Kp=300/Kd=60`、副轴目标速率 `30°/s`、副轴边界 `±30°`、goal 容差 `0.2 m`
和 ground 物理碰撞均保持不变。`drive_dynamics_v1.json` 继续是由 raw54 标定导出的
唯一 DriveModel；不采用 experimental transient tau。

本阶段的唯一允许行为修复是 Task 3 中由 baseline 轨迹证据指向的最小
`OraclePathPolicy`/跟踪链修改。不得先猜 PID、先调阈值、降低碰撞标准或修改规划器。

## 复用的现有边界

| 组件 | 现有接口 | 第一阶段用法 |
| --- | --- | --- |
| 动作契约 | `load_action_protocol(path) -> ActionProtocol` | 保持 `ball_action_v2_aux30`、5 个控制/块、10 维 block；每个物理 tick 仍只有 `(main, auxiliary)`。 |
| V1 Oracle | `OraclePathPolicy(protocol, lookahead_m=..., drive_model=...)`、`reset(EpisodeContext)`、`act(PolicyObservation, pointgoal_base_xy)` | 传入纯几何 `oracle_path_xy` 与 `tuple[KinodynamicWaypoint, ...]`；策略每次从实际 `PolicyObservation.position_world_xy/yaw_world_rad` 投影参考，不按预期时间或名义位移推进。 |
| 上下文/waypoint | `EpisodeContext(..., privileged=True, oracle_path_xy, kinodynamic_reference)`；`KinodynamicWaypoint(x_m, y_m, yaw_rad, drive_direction, main_command_rad, auxiliary_angle_rad)` | 几何生成器仅构造现有类型；不调用 Hybrid A*，不向 `PolicyObservation` 增加字段。方向切换沿用实际 planner cusp 编码；停车边界另存为 segment metadata，不伪造 waypoint。 |
| runner | 当前已核对签名为 `run_episode(runtime, policy, episode, *, action_protocol, config, oracle_path_xy=None, kinodynamic_plan=None, dynamic_plan=None) -> EpisodeRun` | 保持这个调用形状；在 `RunnerConfig` 增加默认关闭的诊断 settling 字段，而不是在文档中假定尚不存在的 `run_episode` 关键字参数。注入几何参考绕过 runtime 的规划结果，pointgoal 仍由 runner 用真实位姿计算。 |
| Isaac runtime | `IsaacRuntimeConfig`、`IsaacBenchmarkRuntime.reset/execute/close`、`captured_frames`、`occupancy` | 每 case 新建一个现有 runtime 实例；`execute()` 保持每 block 5 次 `world.step(render=False)`，之后一次 `world.render()`。只补充时间/roll 记录，不复制或重写仿真适配器。 |
| 产物工具 | `write_rgb_video`、`render_trajectory_map`、`artifact_sha256` | 复用视频编码、轨迹图和 SHA-256；诊断目录不冒充正式 AuditBundle。 |

## 诊断 reference 与 case 集合

新增的 Isaac-free `tracking_acceptance.py` 生成 dense XY/yaw polyline，并把每个点映射为
现有 `KinodynamicWaypoint`。几何参考的点距固定且足够密（实现要求不超过 `0.05 m`），
但跟踪指标使用点到线段的投影距离，避免把采样点间距误当成误差。主轴名义命令只取
已接受的 `0.5` 或 `1.0 rad/s`；副轴前馈仍由现有 `DriveModel`/Oracle 产生。

默认 8 个 case，诊断配置中固定 case 顺序与所有参数：

1. `line_forward_0p5`：直线前进、主轴 `+0.5`，到达后停车；
2. `line_forward_1p0`：直线前进、主轴 `+1.0`，到达后停车；
3. `arc_left_r3_90_0p5`：半径约 `3 m` 的左 90° 圆弧、主轴 `+0.5`；
4. `arc_right_r3_90_0p5`：半径约 `3 m` 的右 90° 圆弧、主轴 `+0.5`；
5. `arc_left_r1p5_90_probe`：半径约 `1.5 m` 的左 90° 能力探测，单独标记为 probe，
   失败不得被叙述成正式安全证明；
6. `s_curve_0p5`：两个相反曲率的平滑 S 弯、主轴 `+0.5`；
7. `line_reverse_0p5`：直线倒车、主轴 `-0.5`，到达后停车；
8. `forward_stop_reverse_stop_forward`：前进—停—倒车—停—前进，所有停车边界均须
   实际速度低于阈值后再换向。

左右 `r=3 m` 圆弧从同一 `(0, 0), yaw=0` 起点出发；左弧终点为 `(3, 3), yaw=π/2`，
右弧终点为 `(3, -3), yaw=-π/2`。直线长度、S 弯半径和采样间距写入配置，不在运行时
随机化。混向 case 的固定几何为从 `(0, 0)` 前进到 `(2, 0)`，倒车到 `(0.75, 0)`，
再前进到 `(3, 0)`；最终目标不与起点重叠，也不会因 reset 状态误判为已到目标。

混向 reference 必须沿用实际 `KinodynamicPlanResult` 的 cusp 编码：waypoint 的
`drive_direction` 只取 `"forward"`/`"reverse"`，非终点 waypoint 的
`main_command_rad_s` 为对应的 `+0.5`/`-0.5`，方向切换不添加合成停车点，
也不虚构 `"stopped"` 方向。该策略当前 `_project_to_reference()` 对同坐标 argmin
取前者，复制 cusp 坐标会把进度卡在旧 segment；停车要求作为独立 `segment_metadata` 和
实测 state 证据记录。唯一的零主轴 waypoint 是末端停车标记。每个 cusp 都报告
`boundary_reached`、实际 active direction 是否切换、停止计数和实际速度；任一方向边界
没有实际经过时，该 mixed case 只能判为诊断失败，不能因 final goal 误判为 pass。

每一 case 都使用显式起点、起始 yaw、目标和 diagnostic budget；不随机化 geometry。
路径终点可用现有 `goal_tolerance_m=0.2` 判定。诊断在首次进入 goal 后最多用 `3.0 s`
零命令 settling（10 Hz、最多 30 个 block），而不是要求刚进 goal 的前 5 个 state
立即静止；在该窗口中，只要存在最后连续 `0.5 s` 的 5 个 10 Hz settling state，且
每个 state 仍在 `0.2 m` 内、实测平面速度 `<0.05 m/s`，即 `stop_hold_passed=true`。
如果先发生 collision/runtime error 或达到 settling 上限仍无该窗口，则 stop hold 失败。
这是预先固定的诊断门槛，不改变正式 success 的定义；`EpisodeRun.termination_reason`
仍按现有 runner 语义记录 goal/collision/timeout，stop hold 独立字段报告。

`RunnerConfig` 增加默认关闭且只用于诊断的
`diagnostic_settling_budget_s=0.0`、`diagnostic_stop_hold_s=0.5`、
`diagnostic_stop_speed_m_s=0.05`；`0.0` 保持正式 runner 遇 goal 即结束。诊断配置为
`diagnostic_settling_budget_s=3.0`，且总 `diagnostic_budget_s` 留出这段时间。正式 35 秒
budget 不被诊断 budget 代替：配置、summary 和 artifact 都同时写出
`diagnostic_budget_s` 与固定 `formal_budget_s=35.0`。

## 时间、状态与 ground 语义

现有 `RuntimeStep`/`EpisodeRun` 尚未携带可核验的物理时钟；这是本阶段的硬接口补充：

- `RuntimeStep` 增加相对 scored clock 的 `simulation_time_s: float`、
  `physics_step_index: int`、`roll_world_rad: float`，并同时增加从 Isaac World 原样读取的
  `raw_simulation_time_s: float`、`raw_physics_step_index: int`。
  `IsaacBenchmarkRuntime._capture()` 只能在已完成的 physics step（或 reset 后的首个
  stable capture）读取 `world.current_time` 和 `world.current_time_step_index`；这两个
  World 读数是唯一时钟来源。
- `EpisodeRun` 增加上述五个 state-aligned 数组；`run_episode()` 保持状态轴为 reset
  加每次 action 后的状态，因此 `state_count == action_block_count + 1`。本 telemetry 补丁
  不改变正式 `elapsed_s`/`time_to_goal_s` 的现有 nominal 语义；诊断 summary 另报由
  相对 `simulation_time_s` 得到的 actual elapsed/goal-entry clock。
- warmup/camera prime 不进入 scored clock：完成 prime 后读取并保存
  `clock_reset_raw_simulation_time_s` 与 `clock_reset_raw_physics_step_index`，之后
  每次 capture 计算
  `simulation_time_s = raw_simulation_time_s - clock_reset_raw_simulation_time_s`、
  `physics_step_index = raw_physics_step_index - clock_reset_raw_physics_step_index`。
  同时把两项 raw 值写入 state/artifact。reset state 应为相对 `(0.0, 0)`；每个 action
  block 后从 World 读到的相邻状态应为 `+5` 与 `+0.1 s`。这才能证明 render 没有多推进，
  而不是仅证明本地计数器递增。
- `world.render()` 不增加物理步；测试用 fake world 断言一个 block 恰好为
  `5*step(render=False) + 1*render`。
- open-plane fixture 不包含 ground prim；runtime 通过
  `world.scene.add_default_ground_plane()` 加入唯一的默认 ground。若 USD 仅含空的
  `/World/TrackingOpenPlane` Xform，runtime 只在显式 `diagnostic_open_plane=true` 时
  允许零外部 collision paths，并把 `_contacts=None`；正式/硬场景仍对空 collision paths
  fail closed。这样不会把 fixture ground 与默认 ground 共面叠加。ground 不进入障碍
  contact filter、不产生 obstacle collision，但其物理支撑仍存在；`physical_contact` 的
  来源在 config 中写成“filtered obstacle contact”，不能把 ground 接触写成障碍证据。

## 诊断执行与指标

审查后的职责边界：纯几何模块仅计算描述性误差与 trace，不输出通过/失败判定。
所有停稳、换向、时钟和整体验收由 harness 使用对齐的实测状态与动作判定；不得用任意
速度 mask、终点附近的几个速度值或跳跃的参考索引代替完整执行证据。

`run_tracking_acceptance.py` 是薄 CLI：读取 tracking config，校验固定 all-free occupancy
和专用 open-plane fixture，为一个 case 构造 `BenchmarkEpisode`，然后调用现有 runtime +
`run_episode`。总控按 case 启动一个独立子进程；每个子进程只创建一次
`SimulationApp`/`IsaacBenchmarkRuntime`、关闭一次后退出，绝不在一个 Isaac 进程中 close
后重建下一个 runtime。不新增第二套 articulation、camera、smoother 或 collision 适配器。
baseline 与 A/B 使用完全相同的 case/config/fixture/seed；A/B 唯一允许差异是 Task 3
明确记录的代码变更。

每个 case 分别报告：

- XY cross-track 的 mean、P95、max（点到 reference 线段）；
- heading error 的 mean、P95、max（归一化到 `[-π, π]`）；
- 每个 state 的 goal error，及最终 goal error；
- 进入 goal 后 `0.5 s` hold 的最大/最终速度和是否全部 `<0.05 m/s`；
- 实际主轴速度（`joint_velocity_rad_s[:,0]`）和实际副轴角（`joint_position_rad[:,1]`）
  的完整 trace 与摘要；
- root roll 的完整 trace 与摘要，仅作滚动/异常诊断，不作为翻倒或通过条件；
- collision、filtered physical contact、timeout/runtime error、reference index 和
  Oracle braking/direction diagnostics。

每 case 输出一个目录，至少包含：

```text
states.npz       # simulation_time_s, physics_step_index, pose/yaw/roll, velocity,
                 # joint states, applied_action, collision/contact, metric traces
commands.npz     # commanded action blocks, 5x2 controls, reference pose/commands
trajectory.png   # reference 与实际 world-XY 轨迹
third_person.mp4 # runtime.captured_frames["third_person"], 10 Hz
config.json      # 解析后的 case/runtime/controller/fixture/profile 参数
config.sha256    # config.json 的 SHA-256
metrics.json     # 本 case 指标、budget、pass/fail 与失败阶段
```

summary 另记录 fixture、contract、drive profile、harness/config 的 SHA-256、代码 revision
（若可读）及所有 case 的原始指标。`states.npz` 还必须包含五个 clock 字段（相对
time/index、raw World time/index、roll），以及 `direction_boundary_reached` /
`active_direction_changed` 等混向 segment diagnostics。cross-track 数值只用于后续推导
通行 margin；本阶段不把它直接宣称为安全裕量或障碍物可执行证明。

## 分阶段 gate

Task 1 先通过纯几何/时间对齐测试，再允许启动 Isaac。Task 2 只跑上述 8 个 diagnostic cases
作为 baseline，保留失败 artifact，不调阈值。Task 3 若 baseline 轨迹能定位一个最小且
可复现的跟踪缺陷，只做一个最小修复并在同 case、同配置下给出 baseline/fix A/B；若证据
不足则不改 policy，明确记录“无修复”。

只有 required cases（左右两个半径 `3 m` case 均为 required；半径 `1.5 m` 为能力 probe，
单独报告）和 clock/settling/cusp 断言达到
项目负责人定义的 tracking acceptance gate 后，后续任务才可收紧 Hybrid A* 的动作/范围
并重新检查规划器。7-task `hard_0` 的正式 35 秒复测、manifest 变更和 benchmark 认证
均是后续阶段依赖，不在此阶段伪装完成。

## Task 3：lookahead 单参数诊断 A/B（2026-09-16）

Task 2 的八 case baseline 轨迹显示，直线前进的横向误差振荡逐步放大，
`line_forward_0p5` 的最大 cross-track 约为 `0.250 m`；倒车直线则保持稳定，
`line_reverse_0p5` 的最大 cross-track 约为 `0.034 m`。这只是控制链现象，不能单凭
该差异确认根因，也不能据此宣称安全裕量或采纳生产修复。

本轮只检验一个假设：pure-pursuit 的短 lookahead 在离散重规划、执行延迟和既有反馈增益
下可能造成前进工况的振荡。比较固定为 baseline `lookahead_m=0.5 m` 与诊断 A/B
`lookahead_m=1.0 m`；A/B 两次使用完全相同的八个 case、纯几何 reference（包括 mixed
case 的 cusp/停车边界编码）、地图/fixture/occupancy、seed、DriveModel、PID、动作契约、
预算、时钟/停稳/碰撞判据和输出校验。除 lookahead 数值外不改变 policy 行为、几何生成器、
mixed reference 或任何生产控制参数。`--policy-variant` 仅作 run label，不引入另一套
constructor 或控制分支。

新增的 `run_tracking_acceptance.py --lookahead-m` 默认显式记录 `0.5`，并把实际有限正值
透传到 child command 与 `OraclePathPolicy` constructor，同时写入每 case `config.json`、
其 SHA-256 覆盖的 canonical bytes、`metrics.json` 和 `summary.json`。已有 baseline artifact
是在该参数显式记录前生成的，但当时 `OraclePathPolicy` 的默认值同为 `0.5 m`；因此它可作为
行为 baseline，后续 A/B 产物必须使用新字段并保留两组原始输出。该诊断未确认根因，也不把
`1.0 m` 变更为生产默认值；是否继续进入 policy/planner 修改需等待同 case A/B 证据与项目
负责人门禁。
