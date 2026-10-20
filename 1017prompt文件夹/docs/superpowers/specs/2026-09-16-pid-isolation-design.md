# 球形机器人 PD / 路径外环振荡隔离设计

## 目标与授权

用户授权自主排查，无需人工审查；代码、调试、测试、实验与代码审查使用 Luna Max。回答：固定指令时是否振荡、目标限速与关节响应分别贡献多少延迟、闭环路径反馈是否放大振荡。不是重做导航架构，也不预设 PD 是根因。

## 冻结条件

- 工作树 `.worktrees/e1-detour-benchmark`，保留既有 dirty changes，不提交、推送或清理。
- 使用现有 `IsaacBenchmarkRuntime`、`tracking_open_plane.usda` 与 all-free occupancy；不复制旧 calibration 的仿真循环。
- 动作协议 `ball_action_v2_aux30`；主轴加速度 0.5 rad/s²、主轴 D=80；副轴基线 Kp=300、Kd=60、目标速率 30°/s、上下限 ±30°。
- 每块 5 次 `world.step(render=False)`，之后一次独立 `world.render()`；50 Hz 状态必须来自真实 World 时钟，不用计数器冒充实测时间。
- 每个 case 新建独立 Isaac 进程，GPU 单一 owner。原始输出不可覆盖，失败保留 partial 与异常信息。
- 不改变正式地图、0.1 m 裕度、0.2 m 到达标准、35 s 正式预算或 DriveModel 文件。PD 实验值不成为默认值，也不得静默重写标定。

## 路线选择

直接调 PD 无法区分外环命令振荡与执行层响应；旧标定脚本缺少请求/应用目标分离和真实时钟/roll，且 GUI stepping 不等价。选择复用已验证 runtime，增加默认关闭的逐物理步记录，再用独立诊断 CLI 驱动固定时序或现有跟踪器。

## 初始案例（全部采用基线 PD）

表中时间为动作执行的实际相对秒，阶段边界落在 10 Hz 块边界。主轴数值单位 rad/s，副轴以度描述、实现保存 rad。

| case | 阶段与目标 | 总时长 |
| --- | --- | --- |
| hold_zero | 全程 main=0, aux=0 | 20 s |
| forward_zero | [0,2) 0/0；[2,22) 0.5/0；[22,30) 0/0 | 30 s |
| forward_pos10_return | [0,2) 0/0；[2,8) 0.5/0；[8,16) 0.5/+10°；[16,24) 0.5/0；[24,30) 0/0 | 30 s |
| forward_neg10_return | 与上一案例相同，转角改为 -10° | 30 s |
| stationary_pos10_return | 与正向脉冲相同的阶段，但主轴全程 0 | 30 s |
| closed_loop_lookahead05 | 现有 3 m 前进直线、main=0.5，OraclePathPolicy 前视 0.5 m | 沿用原 20 s 诊断预算及最多 3 s 的预算内 settling |
| closed_loop_lookahead10 | 同上，前视 1.0 m | 同上 |

固定时序案例不使用路径误差反馈。可复用 run_episode 的观测与记录，但目标设在 12 m 处以避免正常行驶提前触发到达；固定时长到期是实验正常结束，不是导航失败，更不是导航成功。静止测试同样不把路径误差作为失败。记录完整 reset state，跨组对比报告初始状态差异，不能未经核对就称完全同初始状态。

## 逐物理步观测

runtime 增加显式默认关闭的 diagnostics 开关；关闭时原有执行顺序、动作、相机时序不变。开启时每个控制后的物理步记录一次：请求目标、限速后应用指令、关节位置/速度、root 位置/四元数/线速度/角速度、yaw/roll、真实 raw/relative World time 和 step index。

physics trace 是 post-control 对齐，行数为 action_blocks×5；reset state 独立保存在 block-state 产物中，不能混为 physics 第一行动作。sample 来自实际 getter，不允许从目标或有限差分伪造实际关节速度。与任务相关的 gain 与关节名称/有效参数读回信息写入配置，读回不可用应明确标注，不能把请求值称为测量值。

为后续有证据的单参数实验，可提供仅在 diagnostic_open_plane 模式允许的副轴 Kp/Kd 显式覆盖。非正、非有限及布尔参数拒绝；正式模式使用覆盖时报错。不提供主轴参数或速率限制扫描。覆盖参数、基线值、实际读回与 DriveModel 未重新验证的状态都要留档。

## 产物与判读

每例保存 config.json 与 SHA、metrics.json、50 Hz physics.npz、block states/actions、第三视角视频；绘制原始/应用/实际副轴角、关节误差、主轴响应、roll/yaw-rate、实际轨迹。汇总标明阶段，不以固定转角时持续改变的 yaw 判振荡。失败要在关闭仿真前持久化，不能只留下无解释的 partial。

逐阶段报告实际主轴速度、aux actual-applied RMS/max error、末 2 s 的 aux/roll/yaw-rate peak-to-peak 与标准差、主轴速度误差。阶段短于 2 s 时使用完整阶段并注明。转角阶跃的超调和 settling 要按非零阶跃方向计算；零目标阶段不计算百分比超调。±0.5° 是预先声明的副轴 settling 描述带，至少连续 1 s 且保持到阶段结束，未达到记 null，不能填零。原始目标→应用目标延迟与应用目标→实际角度响应分开报告。

所有统计是诊断描述，不等于正式 SR，也不能凭相关性独自证明根因。若固定指令下实际关节/球体持续摆振，可选择一个代表案例做一次单参数 PD A/B；若固定指令稳定而路径闭环明显振荡，优先归因范围转向外环/延迟匹配，不强行调 PD。最多两种非基线 PD 组合，由父任务看完基线后明确选定，禁止 worker 自动扫参或重试。参数改进需保留跟踪对照和标定复查事项，不自动推广。
