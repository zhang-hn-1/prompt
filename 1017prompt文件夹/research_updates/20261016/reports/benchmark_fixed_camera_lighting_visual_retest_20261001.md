# 固定相机补光后的视觉导航复验（2026-10-01）

## 结论

相机位置、光轴及物理规则均保持原样。场景预初始化补光使 E1 墙面在原始 RGB 中从约 3/255 亮到约 47/255，但**亮度改善尚未变成可接受的视觉导航能力**。原轻量像素分类器用补光 E0/E2 训练、整场景测试 E1 时，起点 0–2 s 中央真墙仍 **0/19,858 = 0%** 检出。只增加两条预先选定的 hard_0 训练序列后，检出升到 **62.96%**，说明缺少类似障碍样本是失败原因之一；但仍低于训练前冻结的 80% 门槛，且 E1 已知可通行像素误报障碍 **7.88%**，超过 5% 门槛。该候选明确拒绝接入在线控制。

用当前同一源码的传统双目局部策略 V2，在 golden E2、原 35 s/真实接触规则下各运行一次：旧光照 **6.7 s 碰撞**，补光 **5.8 s 碰撞**。两次均不是导航成功；旧版本历史记录的 10.5 s 碰撞来自不同 `vertical_depth.py` 源码身份，不能与本轮单独归因于灯光。相同补光条件下的无视觉 PointGoal 对照在 E2 仍 **14.0 s 到达**（见前轮光照报告），因此当前视觉决策仍使可通走廊退化。

## 冻结实验与数据

第一项复验直接复用前轮 [`pilot.py`](../experiments/fixed_rig_visual_pilot_20261001/pilot.py) 的 RGB 输入边界、标签、网络、随机种子、8 轮训练和整场景两折。原光照的旧模型不覆盖；补光版本单独存于 [`illuminated_replay_v1`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1)。补光 E0/E1/E2 原生运行分别提供 70/30/71 帧，每帧左右两目。训练 E0+E2 测试完整 E1；训练 E0+E1 测试完整 E2。两折 `unknown` 召回需 ≥90%、`free→obstacle` ≤5%，E1 起点中央墙召回 ≥80%，这些门槛在训练前记录于 [`protocol.md`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/protocol.md)。仿真深度和位姿只产生离线标签，模型只读当前 RGB、像素坐标及眼别。

第二项是**唯一**覆盖扩展，事先选定 `hard_0_e1_0001/0002` 两条完整序列，在同一固定相机、`fixed_camera_fill_v1` 下用无视觉动作采集 20/14 帧。两条都因真实接触结束，分别为 3.8/2.6 s；不把它们的失败当成视觉结果。两条序列提供约 31.3 万个下采样障碍标签像素，其中中央障碍约 16.8 万。把它们只加入训练折，golden E1/E2 仍作为整场景测试；网络和门槛不改。方案在采集前写入 [`augmentation_protocol.md`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/augmentation_protocol.md)。这些场景曾在项目历史中使用，故不是全新 held-out。

| 完整测试场景 | 补光原训练：unknown 召回 | 原训练：free→obstacle | 原训练：E1 0–2 s 墙召回 | 加两条训练序列：unknown 召回 | 扩展：free→obstacle | 扩展：E1 墙召回 |
|---|---:|---:|---:|---:|---:|---:|
| E1 | 95.42% | 0.00% | **0.00%** | 94.74% | **7.88%** | **62.96%** |
| E2 | **89.42%** | 3.11% | 不适用 | **89.01%** | 2.16% | 不适用 |

扩展候选在 E1 第一帧左/右墙召回为 74.6%/72.4%，随后多个帧落到约 51%–67%，没有出现连续三帧对、两目均达 80% 的稳定检出。E2 中央已知地面误报障碍从原训练的 0.096% 到扩展的 0.479%，中央地面未知率从 12.8% 到 9.64%；这些局部数字不能抵消整场景 gate 失败。完整混淆矩阵与逐帧明细在 [`原训练结果`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/results.json) 和 [`扩展结果`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/augmented/results.json)。

## 为什么双目亮了仍难直接测距

两目在球体两侧相距约 0.60 m，视差沿图像纵向。用**渲染深度仅作离线真值**，把左目中央墙点投影到右目，再要求右目深度与投影深度在 0.2 m 内一致：补光 E1 起点 0–2 s 的左目中央墙只有 **1,065/9,890 = 10.8%** 在右目同时看到同一表面；旧光照为 **1,061/9,864 = 10.8%**。E2 同期中央已知地面重叠约 **223/4,982 = 4.5%**。这说明补光没有扩大双目几何重叠，左目能看见墙并不等于可靠双目视差可直接恢复其深度。计算入口和逐帧数据见 [`overlap_audit.py`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/overlap_audit.py) 与 [`overlap_audit.json`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/overlap_audit.json)。

另做了一个只读、**探索性的** OpenCV 纵向 SGBM 检查：旋转图像后在补光 E1/E2 起点至 2 s 取七帧，E1 中央墙有效视差仅约 11%–22%，与真值视差相差不超过 8 px 的比例在各帧约 0%–1.5%。这不否定更强的匹配算法；它说明当前简单匹配器不能因为画面变亮就被视为可靠测距。参数和逐帧结果在 [`stereo_probe.py`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/stereo_probe.py) 与 [`stereo_probe.json`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/stereo_probe.json)。

## 在线控制对照与验收

正式 V2 在线输入仅有左右 RGB、PointGoal 和历史/本体信息，不读取上述离线深度。新旧光照 E2 对照共用当前同一 V2 源码哈希、roll PID `Kp=0.4, Ki=0.1, Kd=0.7`、冻结相机、相同 manifest/场景/栅格 SHA、相同初始 XY、35 s 上限及 `physical_contact_v2`。两次均生成动作、视频、物理接触记录和轨迹：

| E2 V2 视觉策略，同版本 | 光照 | 终态 | 仿真时间 | 实际走行 | 目标方向进度 |
|---|---|---|---:|---:|---:|
| [`暗光对照`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/navigation_visual_v2_e2_dark_control/episode_golden_e2_0001/metrics.json) | `benchmark_sun_v1` | 真实碰撞 | 6.7 s | 2.379 m | 2.234 m |
| [`补光运行`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/navigation_visual_v2_e2/episode_golden_e2_0001/metrics.json) | `fixed_camera_fill_v1` | 真实碰撞 | 5.8 s | 1.952 m | 1.766 m |

补光运行到约第 20 个 10 Hz 动作块时，所选局部航向相对目标偏到约 `−0.41 rad`，后续在窄走廊里持续改向并触到下侧障碍。不能把较早终止解释为补光普遍更差；每种光照仅一条完整运行，但足以拒绝“补光后该现有策略已成功”的判断。可直接看 [`补光 E2 轨迹`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/navigation_visual_v2_e2/episode_golden_e2_0001/trajectory_overlay.png) 和 [`双目视频`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/navigation_visual_v2_e2/episode_golden_e2_0001/stereo_view.mp4)。[`verification.json`](../experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1/verification.json) 记录本轮身份、完整序列隔离和终态核验；分类标签、视差及视觉策略共 7 项测试通过。

## 下一控制工作

这次实验确定了两个具体瓶颈：**训练覆盖不足**使明亮墙体仍被错分，且相机侧向基线/遮挡使中央目标在双目中的同表面重叠很小。继续调灯或只调 SGBM 阈值不能直接解决。下一项应保留相机原位、固定此光照版本，以更多几何和外观不同的完整场景训练单目/短时多帧 `障碍/可通/未知` 表示；先用独立完整场景检查 E1 类真墙稳定检出、E2 类走廊误报和有效提前量。通过感知门槛后，再加入显式自体遮挡和未知处理的局部记忆，先验收 E2 不因视觉误转，随后才运行 E1 绕行与完整 E0/E1/E2。当前扩展候选和 V2 都不应冒充已完成的传统视觉导航基线。
