# V1 velocity-pursuit 独立首次审查

结论：V1 证据足以支持“这一固定局部参考上的横向跟踪改善”，原冻结验收仍为 **REJECT**。没有发现 Critical 或 Important 问题。发现一项 Minor 计数表述问题，作者已追加澄清。不能据此晋升正式控制器、声称停车资格、整条绕障成功或正式 E0/E1/E2 成功率。V2 尚未纳入本审查；单臂闭合或 RUNNING 状态不构成 V2 验证。

本次为 velocity 分支首次独立审查，没有借用旧 anchor、learning 或 terminal 分支审查结论。完整阅读当前报告和 consumer/fixed-reference 两份计划；审查 consumer、固定参考 policy、测试、runner derivation、closed-row audit、assess/compare/goal-event scripts 与两臂封存数据。没有修改控制实现、原始数据或旧评估，没有启动仿真、推断模型或上传。

## Findings

- **Critical：0。** 未发现未来真值输入、隐蔽多环节改动、伪造实际动作差异或倒改 V1 REJECT 的证据。
- **Important：0。** 下列作用范围和未测停车段已经由报告明确披露，不作为隐瞒问题重复计数。
- **Minor：1，已追加澄清。** 初始报告“188 个完整 smoother/PID 快照”不精确：`execution_states[0]` 为共同 `None`，前缀共 188 条记录，完整快照 187 个。作者已追加区分记录与快照的说明；原 17 字段、936 物理采样和 REJECT 均不受影响。`compare` 中对两条 None 的相等比较只是记录前缀相等，不能称为重建了初始 PID 状态。

原 proto 的继承大任务字段、旧 manifest 的 `observation_scheduled_prefix=False` 和历史 continuation 的 running 文本属于已披露的旧元数据；本审查依据实际 episode、状态、请求、固定参考和闭合状态。不得以那些继承字段解释本次任务。

## 亲自验证的证据

1. **当前真实 class 和单环节变化。** MRO 为 outgoing-endpoint → confident-speed → BankMap → cusp-boundary → `OraclePathPolicy` → `_PolicyBase`；不是 profile-blend、其他控制器或旧 learning 分支。`_compose_tracking_curvature` 使用 pure-pursuit，profile 是诊断量。candidate 只覆盖 lookahead target 几何输入，通过 `R(rootyaw) R(velocitybodyheading)^T` 生成代理；reverse 减 π，低于 .05 m/s 精确返回实际目标，aligned 返回原目标。全局 yaw/pointgoal/reference/投影/BankMap/PID/速度规则均保持原消费链。方向 latch、五次 low/rebound 仍由继承逻辑处理。75 条实际 candidate 几何评估逐条用其当时保存的 XY、yaw 和最后 proprio 重新计算，代理、heading、active 和被消费的目标精确一致。

   独立合并进程初次检查 closed-row 全字典时，仅 MRO 的 bank_mapping 源路径不同：`hybrid` 与 `gated_speed_012` 存在同名副本，两者 SHA256 完全一致，类名及 87 条数值均相同。干净进程重跑完整 audit 与旧 JSON 精确相等。记录此导入路径现象，不将其解释为控制变更。runner derivation 所列替换反向还原后与冻结父 runner 逐字一致。

2. **封存与真实采集。** 亲自校验 protocol 1,284 条来源 SHA、B0 post-close 2,030 个文件、candidate post-close 2,016 个文件，零 mismatch；两套作者 assessor 内部重构断言全部通过。每臂重构 1,144 项 packet/mask 字段；B0/candidate 分别重构 1,542/1,530 个 primary map 数组和 3,084/3,060 项 shadow 诊断。视频完整解码：B0 双视频各 258 帧、candidate 各 256 帧，均 10 fps。作者脚本输出被截获，没有写回原 assessment/comparison/audit。

3. **当前输入、冻结参考和无未来。** 512 个实际 policy call 输入逐条核对 position、当时世界 XY velocity、request、来源 frame/time 和 .3 cap；全部 source time ≤ 当前 time。来源 RGB/knownpose 命名空间限制及 future-payload trap 通过。参考正好为源 frame_180 的前四个 pose/direction，前三条 command 相同，末 command 归零，源 SHA 匹配；两臂每个 local_plan 的 pose/direction/command 与同一冻结 reference 精确相等。两臂均在 frame 180 交接，参考先验明确不是当前观察。本任务使用 privileged known pose，并且只到障碍起始侧的局部端点。

4. **实际前缀与有效施加变化。** 首次真实请求差异为 frame 187、约 18.7 秒，当时 candidate velocity course 有效且变换 active。此前 17 个物理字段各 936 个样本精确相等，188 条执行记录相等（187 个完整快照加初始空记录），561 项原 policy 字段比较相等；raw 时钟原点差为 0。共同窗口内 340 个 `applied_action` tick 实际不同，不能解释为仅添加诊断字段。

   进一步从每个非空 block 起始执行快照，用各 tick 当前 quaternion/joint feedback 重算 smoother/PID：B0 1,280 ticks 和 256 个结果快照、candidate 1,270 ticks 和 254 个结果快照全部精确重构 applied、smoothed 和完整结果状态。最初五个 tick 无完整起始快照，明确不声称通过 clone 重建；它们仍有保存的原始/施加目标和独立物理前缀检查。

5. **独立空间评估，不读取 tracker index。** 另写线段最近点投影，以 raw states 和同一 4 点折线重算交接后 RMS/peak、最大/结束 arc progress、两个 spatial checkpoints 和实际目标距离，与作者值在 1e-14 内一致。

| 亲自重算项目 | B0 | candidate |
|---|---:|---:|
| 横向 RMS（m） | .059118433563 | .033199959794 |
| 横向 peak（m） | .118410095388 | .062458558974 |
| 最大/结束参考进度（m） | .717583649730 | .706973981376 |
| 最终目标距离（m） | .195152271892 | .194307433995 |
| 1/3、2/3 checkpoint 首次帧 | 207、234 | 207、234 |
| 实际 contacts | 0 | 0 |
| 最终 XY speed（m/s） | .137902592563 | .126537069920 |

6. **冻结门槛没有回改。** 原进度 gate 为 false，candidate 少 .010609668354 m；其余报告中的 gates 重算一致，仍 REJECT。末段切线与横向距离/目标距离重构终态进度精确成立。共同 frame 255、约 25.5 秒时，candidate 比 B0 多 .014190616467 m。这个共同时间补充解释两臂在不同位置进入 .2 m 圆区，不能替换预先冻结的“各自结束时最大进度”门槛。

7. **CPU 和 closed-row 算术。** 亲自运行完整现有 CPU suite：**469 passed in 24.03s**，无新增仿真。亲自只读重跑 closed-row audit：87 条、原副轴全部精确重构、71 条 candidate 副轴不同。它没有调用第二次 stateful policy，也没有预测未来物理收益。两臂 assessor 的所有内部断言通过，所打印 summary 与旧结果除 utc 外相同；compare 和 goal-event audit 同样精确复现。

源码和元数据保留的 RED/初次 fixture 错误不是本次 reviewer 新跑 RED；本次只亲自确认当前 GREEN 和实际证据。来源 assessor 的 offline-depth 运算仍出现既有非有限深度 RuntimeWarning，随后有限值筛选生效且内部断言通过；不能把该离线数组当在线输入。

## 范围与后续

原 V1 在进入目标区域后立刻结束；末速度均高于 .05 m/s，没有实际目标内停车尾迹、连续五次 low 或离开目标区域的数据。V1 局部跟踪改善成立，但“已停稳”以及正式视觉绕障成功均未验证。V2 如果有完整双臂闭合证据，可作为这一独立审查的追加范围；当前结论不预支它的结果。

机器可读亲自验证记录：`velocity_tracking_review_personal_checks.json`。原 `personal_checks.json` 属于旧 terminal-goal 审查，按作者确认保留。本次只保存新的审查产物。
