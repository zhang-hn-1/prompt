# 短时动作实测与 Oracle 倒车接续导航（2026-10-03）

## 本轮结果

在此前 2.4 s 碰撞失败的 `hard_0 E1 0004` 上，完成了一条**28.4 s 内进入目标范围、全程无物理接触**的 Oracle 开发实验。最终球心至目标距离 0.133 m，实际路径长度 17.912 m，保持原 35 s 评测规则。没有把参考索引推进、停车或延后失败当成成功：独立用 manifest 中的真实目标和实际 XY 轨迹复算距离，且全部 1705 条评测期间接触采样都可用并为无接触。

它先执行实测通过的倒车转向动作，再从当前实测位置规划并倒车跟踪已知地图路线。它使用特权地图，属于 **Oracle**；尚不是 RGB 视觉导航成功。该场景已被检查，且候选是在它的局部实测之后选择的，结论限于一个历史开发场景，不能报告新的 held-out 成功率。

到达目标时实际 XY 速度仍为 **0.741 m/s**。本次通过的是原规则的“进入 0.20 m 目标范围”，没有验证停稳保持，不能写成“到达后已稳定停车”。

![实际导航路径及真实目标距离](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/oracle_continuation_navigation_comparison.png)

绿色是实际运行，蓝色虚线是 6 s 时根据当前状态构造的 Oracle 路线，红色是原 B0 的实际失败轨迹。右图用相同 manifest 目标评估实际距离；前 6 s 脱困时离目标更远，随后完成了回到目标的过程。

## 先验证测量没有改变 B0

为避免再次把几何曲线当作执行结果，本轮新增了 50 Hz 物理状态、请求/协议/smoothed/applied 指令链及接触时间的对齐记录。先在相同场景与起点重放原 B0 的 24 个动作块：25 个 10 Hz 状态样本的 XY、yaw、世界速度、关节位置/速度、applied 指令、时钟和碰撞标记，与原闭环日志的最大差异全部为 **0**；145 条接触记录的时间、步号、来源及布尔状态一致。首次真实接触仍在 2.340 s，动作块终止于 2.4 s。

这次重放验证的是测量接口与旧请求序列的确定性，不把请求重放冒充重新运行 B0 的在线反馈决策。原闭环运行与完整记录仍保留。

冻结了 [`PROTOCOL.md`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/PROTOCOL.md)：七种动作分别完整重置，序列内不重置物理状态、smoother 或 PID。它们与 B0 重放的初始姿态、世界速度、关节状态、执行状态和 PID 状态全部一致。

## 七种短时动作的实际结果

移动动作请求主轴 ±2 rad/s，副轴请求 -30°、0° 或 +30°；停车请求为零。沿用正式协议、smoother、物理接触判定及现有 roll PID 参数 P=0.4/I=0.1/D=0.7。**副轴请求 ±30° 不是实际 roll 角**；PID 的既有目标转换和后续执行限制继续生效。按请求符号命名，避免预先混淆倒车时的左右方向。

每条最多 6 s，遇真实接触即在当前动作块末结束。预设“可移动生存”要求完整 6 s 无接触且净位移至少 0.25 m；停车不能作为有效移动选项。没有看完数据后缩短时间门槛。

| 请求模式 | 首次接触时间 | 测量结束时间 | 净位移 | 目标距离减少量 | 冻结的局部判定 |
|---|---:|---:|---:|---:|---|
| 前进，副轴 0° | 3.04 s | 3.1 s | 0.662 m | +0.662 m | 接触 |
| 前进，副轴 -30° | 4.78 s | 4.8 s | 1.683 m | +1.369 m | 接触 |
| 前进，副轴 +30° | 2.04 s | 2.1 s | 0.293 m | +0.217 m | 接触 |
| 倒车，副轴 0° | 4.10 s | 4.1 s | 1.299 m | -1.299 m | 接触 |
| 倒车，副轴 -30° | 4.40 s | 4.4 s | 1.483 m | -1.311 m | 接触 |
| **倒车，副轴 +30°** | **无** | **6.0 s** | **2.368 m** | **-1.566 m** | **完整短时可移动生存** |
| 停车 | 无 | 6.0 s | 0.034 m | +0.033 m | 无有效移动 |

六种移动动作中，一种完整生存、五种接触，因而本次预设的“同一状态下存在不同物理结果”覆盖门槛通过。它只说明这些实际请求在这个起点的 6 s 结果有区别，不代表该动作之后一直安全，也不能用七个分支训练并宣称跨场景泛化。

![短时实际轨迹和接触时的地图距离](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/physical_primitive_comparison.png)

原三条 4 m 固定几何曲线的拒绝结论保留。新的实测短时动作有了可执行选项，说明“原曲线库无选项”不等于“机器人在该起点没有可执行的移动动作”。两者是不同口径，没有追改旧门槛。

## 地图距离与真实接触的对齐

对每个 50 Hz 实测位置计算连续世界 XY 到已占据栅格中心的距离，同时计算到占据栅格方格区域的距离。地图 axis 0 为世界 X，axis 1 为世界 Y。物理接触采样与状态用**完全相同的物理步号与时间**关联，没有用插值替代接触时刻的测量。

| 序列 | 首次接触时：到占据栅格中心 | 首次接触时：到占据方格区域 |
|---|---:|---:|
| B0 重放 | 0.403 m | 0.340 m |
| 前进 0° | 0.417 m | 0.348 m |
| 前进 -30° | 0.424 m | 0.354 m |
| 前进 +30° | 0.409 m | 0.359 m |
| 倒车 0° | 0.402 m | 0.332 m |
| 倒车 -30° | 0.401 m | 0.331 m |

无接触的倒车 +30° 序列，最小中心距离 0.670 m、最小方格区域距离 0.620 m。栅格中心、栅格覆盖区域和真实网格表面是不同几何量；这些实测结果解释了距离口径的差别，不能把其中某一条距离阈值改写成完全准确的物理接触检测器。本轮继续使用原 physical-contact-v2 判定，没有放宽接触阈值或修改旧碰撞结果。完整原始数据见 [`assessment.json`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/assessment.json)。

## 一次完整 Oracle 接续验证

局部门槛通过后，另行冻结 [`NAVIGATION_PROBE_PROTOCOL.md`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/NAVIGATION_PROBE_PROTOCOL.md)，只测试一个完整候选：

1. 前 60 个动作块完全复用倒车 +30° 短时动作。
2. 在 6 s 的当前实际状态，用已知地图、当前位置与已知目标调用仓库已有 A*；未知区域视为受阻，规划膨胀半径固定 0.60 m。
3. 以 1.0 m 前视距离接回路径，复用现有几何跟踪器并做倒车的目标/执行符号转换；roll PID、协议、速度边界及 smoother 均沿用。

地图路径未预先宣称满足全部动态约束，是否真正可执行由本次物理运行判断。生成请求只读取当前状态、过去计数和已知地图/目标，不读取未来实际反馈动作或轨迹。

| 同一起点的完整运行 | 结果 | 时间 | 最终目标距离 | 实际路程 |
|---|---|---:|---:|---:|
| 原 B0：固定 RGB 触发右绕 | 物理碰撞 | 2.4 s | 8.859 m | 0.457 m |
| 新候选：倒车脱困 + 地图 Oracle 路线 | **进入目标范围，无物理接触** | **28.4 s** | **0.133 m** | **17.912 m** |

候选前 6 s 的全部 301 个 50 Hz 状态样本，包含 root 四元数、速度、关节、smoothed/applied 和 raw request，与独立短时实验最大差异均为 **0**；60 个请求块也完全相同。接续没有重置机器人或 smoother。独立验收见 [`validation.json`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/navigation_probe/validation.json)。

B0 与候选的路线来源和行驶方向都不同，因此这是一项新的 Oracle 可行性证明，不能把完整改善单独归因于某一个转向符号或 PID 参数。原 B0 使用 RGB 触发，候选使用地图，两者的信息权限也不同，不能用此表声称候选视觉方法更优。它证明同一物理机器人及执行参数在该场景具备完成任务的能力。

## 当前视野仍有哪些限制

利用初始双目真实深度，对实际短时轨迹上的球体包络作了事后可见性诊断：每个实测位置取三个球体截面共 27 个点，排除距初始机器人中心小于 0.55 m 的区域；只有点落入相机视野、深度有效且位于已测表面之前 0.05 m，才算有空闲空间证据。

三种倒车轨迹在这个**初始视角**中，离开初始自体区域的包络采样点均为 **0% 可由任一眼的深度提供空闲证据**；成功的倒车 +30° 分支对应 1005 个这样的采样点。前进 0°、-30°、+30° 分别约 29.5%、39.1%、3.7%。这只是离线、有限包络采样，不能当作完整体积安全证书或 RGB 模型准确率；但它直接暴露了倒车方向的初始可见性缺口。

因此地图 Oracle 的成功不能直接移植成“仅凭当前双目图像就执行同样的 6 s 倒车”。视觉导航需要历史环境记忆、明确的未知区域语义，以及运动中的观测更新；世界模型也不能把从未观察到的后方障碍默认为空闲。本轮没有训练新网络，前几版 RGB 模型拒绝结论保持不变。原始可见性口径和数据见 [`initial_visibility_audit.json`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/initial_visibility_audit.json)。

## 实际证据、交付与后续控制重点

共实际运行 **9 个进程**：1 条 B0 精确重放、7 条固定局部序列、1 条完整 Oracle 导航候选。没有扩大参数矩阵或试第二个导航变体。局部序列有 1653 个 50 Hz 物理样本；完整导航有 1421 个；原始双目 RGB/深度、视频、实际动作链、物理接触和源码身份均保存在实验文件夹。

控制与测量的相关回归测试共 **22/22 通过**。此外重新执行了实际数据验收，核验九个运行均完成、起点一致、B0 重放一致、完整候选的脱困前缀一致、目标距离与接触记录满足原规则。

目前解决的是该困难起点的可执行动作覆盖，以及一条真实完成的地图 Oracle 路线；尚未完成更广场景的 Oracle 验证、到达后的停稳、RGB 感知和传统视觉导航 baseline。下一项控制工作应优先验证终段减速与停稳，再检查同一候选在其他完整场景的适用范围。视觉工作应以可见性和历史记忆为约束，不能直接照搬这个地图 Oracle 的盲区倒车动作。

可观看 [完整导航第三视角视频](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/navigation_probe/episode_hard_0_e1_0004/third_person.mp4) 和 [双目视频](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/navigation_probe/episode_hard_0_e1_0004/stereo_view.mp4)。正式指标与实际轨迹在 [`metrics.json`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1/navigation_probe/episode_hard_0_e1_0004/metrics.json) 和同目录 `states.npz`；离线验收可由实验内的 `assess.py`、`validate_navigation.py` 与 `initial_visibility_audit.py` 复现。
