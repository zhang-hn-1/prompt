# V2 native goal-settling 独立首次追加审查

结论：**保留 V2 REJECT，也保留 V1 REJECT**。原始物理证据支持共同窗口局部跟踪改善，以及 candidate 在约 2.1 秒后达到五次低速且在目标内的采样资格；同一 candidate 曾真实越出 .2 m 目标区域，因此预先冻结的全程目标内条件失败。不能晋升控制器、宣称更长期停稳或正式导航成功。

本次是同一独立 reviewer 对新闭合 V2 的首次追加核查，没有重做 V1 已通过检查，没有将 V1 审查当作 V2 认证。完整阅读 V2 报告、计划、runner/derivation/protocol/status、assess/compare/zero-tail audit 和实际结果；没有启动物理、网络推断、修改控制或封存文件。新 past-point RGB corroboration 旁支不在本次控制复核范围。

## Findings

- **Critical：0。** 未发现未来真值消费、隐藏改动原生 runner/控制器、造假零请求尾段、越界后仍声称整体通过或倒改 V1 REJECT。
- **Important：0。** candidate 的越界失败如实保留；这些已用前缀形成的新发展/回归测量不冒充新 heldout，不能推出原始绕障导航成功。
- **Minor：2。** 第一，两臂封存 `identity.json` 的 `extra_physics_tail=False` 是继承失真：实际有 30/21 个额外物理零请求 block。报告、实际 dispatch 和 mask 已正确区分尾段，建议追加明确该身份旗标错误而保留封存文件。第二，zero-tail audit 的 bank 范围来自 10 Hz execution snapshot 中 PID 保存的最后反馈，entry snapshot 又保存前一个 tick 的反馈；这些值可报告为 PID 保存反馈样本范围，不能称为完整 50 Hz 物理尾迹的姿态极值。独立 quaternion 重算值见下，建议追加采样/字段定义。二者均不影响 REJECT、停车资格或归因限制。已将两项通知作者，本报告记录发现而不修改原始证据。

## 亲自检查

1. **冻结和真实单环节延伸。** protocol 冻结于 01:08:02 UTC，两臂均在其后启动，闭合状态均 finished/exit 0，最终闭合于 01:18:37 UTC。校验 1,286 条 source/code/config SHA、B0 2,174 个 post-close artifact、candidate 2,116 个 post-close artifact，零 mismatch。V1/V2 protocol 中原生 `benchmark/runner.py` SHA 相同且匹配当前字节。将 V2 derivation 的三个替换反向恢复后，与 V1 runner 原字节及父 SHA 精确一致：仅 protocol/output 名称与既有 `RunnerConfig(diagnostic_settling_budget_s=3.)`。

   当前原生配置仍为总 35 秒、10 Hz、.2 m、.05 m/s、5 次采样、尾段最多 30 blocks。原生代码在名义进入目标后 dispatch 零 action 而不调用 policy；速度反弹或离开目标重置 hold count，collision 优先；沿用 nominal success 时间并另存尾段物理长度。没有新停车 policy、gain、threshold 或 reset。两臂进入目标速度均高于 .05，因此 legacy 是否把低速 entry 算第一个样本不影响本次实际资格。

2. **短 policy 与长 physics 的独立辨别。** 从 episode_arrays 而不是缓存 tracking fields 重建所有实际 dispatch：前缀 mask false，尾段 mask true，尾段全部 10 维零 action；每个 block 五条 physics raw_command_target 与实际 dispatch 精确相同。实际 policy_records/shadow/maps 在名义目标处结束，后续 capture/source/physics 持续；不能把尾段 cached policy diagnostics 当作新的 policy.act 或新追踪输出。

| 实际计数/结果 | B0 | candidate |
|---|---:|---:|
| policy calls | 257 | 255 |
| dispatch blocks | 287 | 276 |
| captures | 288 | 277 |
| physics samples | 1436 | 1381 |
| 零请求尾段 blocks | 30 | 21 |
| 最大连续 low+inside 样本 | 4 | 5 |
| 五次资格 frame | 未达到 | 276 |
| 五次资格 delay | 未达到 | 2.099999953 s |
| postentry 目标距离 peak | .195152271892 m | .202711081499 m |
| 50 Hz postentry 超 .2 m 样本 | 0 | 25 |

   以每个 capture 的真实 XY/velocity 和递增时间重新计算 low+inside counter，每一条 count 与原 trace 一致；出区域和速度反弹都归零，candidate 的资格是最后五个不同 10 Hz post-action 状态，不是复用旧帧。资格 bool/帧/延迟与原 metrics 一致。另检查全部 50 Hz physics 目标距离，峰值与 10 Hz capture 峰值恰好一致，candidate 的越界不是评估插值或 controller index 现象。

3. **实际尾段 execution 重构。** 从每个尾段真实完整 snapshot 及每个 tick 当时 quaternion、aux joint position/velocity，独立重算零目标经 PID/smoother 的结果。B0 150 ticks/30 个结果快照、candidate 105 ticks/21 个结果快照，applied_action、smoothed_target 和全部结果 snapshot 精确一致。这验证原始零请求与未立刻归零的施加动作、机体速度必须分开解释。

   原 zero-tail audit 只读重跑，全部字段除 utc 外与封存结果精确相等：两臂 smoothed/applied main 在约 .62 秒后首次为零，之后实际速度仍有 .19737095/.13993448 m/s 峰值；B0 entry course 投影速度变号两次，candidate 零次。此为测到的真实执行/动力学现象，不能归因为 PID 或平滑器的单独因果效果。

   额外从 goal-entry 状态起直至结束的全部 50 Hz quaternion，以实际 BankPID 的同一归一化和 arcsin 定义重算 bank extrema：B0 **[-.370703702799, +.063928185003] rad**，candidate **[-.237950616877, +.020162756590] rad**。与原 audit 的 PID 保存反馈样本范围不同，原因是样本时刻/密度和 entry feedback 滞后，不是控制字节变化。

4. **原 V1 前缀与 A/B 因果前缀。** 两臂各自全部 V1 nominal 原请求、raw states、17 个非 raw-clock physics 字段都精确复现。追加尾段没有暗改早期行为；这些是已知回归前缀，不能当新 heldout。新 A/B 首次差异仍为 frame 187、有效 current course 事件；此前 936 physics samples × 17 fields、188 execution records（187 完整加共同初始 None）、561 项原 policy 字段比较精确相等。共同物理记录期间 445 applied ticks 实际不同。

5. **新的共同窗口门槛独立重算。** 不读取 tracker index，以真实 states 对同一冻结四点折线作最近线段投影，全部误差/进度序列与原 assessment 在 1e-14 内一致；共同窗口为 frame 180–255，约 18–25.5 秒。B0/candidate 进度 .692783364909/.706973981376 m，RMS .058695211009/.033199959794 m，peak .118410095388/.062458558974 m。两臂空间检查点继续通过。新 V2 gates 重跑精确一致，唯一失败的候选门槛为 all measured postentry inside goal；最终 decision REJECT。

   原 V1 comparison SHA 亲自检查前后完全相同，decision 仍 REJECT。这是前瞻冻结的新停车测量条件，不是回改 V1 结束进度口径或安装候选。

6. **来源/地图/视频和原生专门测试。** 两个作者 V2 assessor 在拦截所有写出的只读模式下重跑，内部来源/CAD/mask/joint/RGB-namespace/clock/map/shadow/future-payload 全部断言通过；完整输出除 utc 外与封存 JSON 精确相等。compare 与 zero-tail audit 也精确复现。B0/candidate 分别 3,460/3,402 identity checks、1,276/1,232 packet-mask 字段、1,542/1,530 primary map 数组、3,084/3,060 shadow 字段；来源更新 58/56 次。两视角视频全部解码为 288/277 帧，各 10 fps，覆盖实际尾段。

   亲自运行原生 runner 中 settling 专门测试：**8 passed, 36 deselected in 1.08s**。按追加范围要求，没有再跑完整 469 项，也没有借用它声称新 V2 完成验证。离线 depth 的既有非有限数组 RuntimeWarning 仍出现，finite 筛选和全部后续断言通过；不影响在线输入边界。

## 审查范围结论

V2 candidate 最终取得原生五采样资格，却已经违反“所有测到的 postentry 位置留在原目标内”，因此不得用最后资格抹去越界，V2 REJECT 成立。B0 在这三秒内最多四次低速资格，只能称“未在已测预算内达到五次”，不能称永远不能停车。对任何一臂，结束之后的反弹、稳定性或更长期表现均未测；没有反事实停车控制器试验。局部 known-pose 固定参考改善不解除来源覆盖、估计姿态、原始整路径与正式 E0/E1/E2 导航缺口。

新机器可读记录：`velocity_goal_settling_review_personal_checks_v2.json`。本次保存新的 V2 产物，V1 原审查和原实验拒绝保持原证据。
