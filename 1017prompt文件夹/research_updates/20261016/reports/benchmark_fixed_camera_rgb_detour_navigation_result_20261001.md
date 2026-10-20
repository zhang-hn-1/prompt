# 固定相机 RGB 触发绕行的实际导航结果（2026-10-01）

## 结论

在相机位置/朝向不变、补光 `fixed_camera_fill_v1`、原 35 秒与 `physical_contact_v2` 规则下，一个**非特权 RGB 触发 + 连续局部曲线**开发候选实际跑通了 golden **E0/E1/E2 三条完整导航序列**。E1 在 **0.1 s** 从当前左目 RGB 触发，实际绕过中央障碍，在 **14.8 s** 到达目标，走行 **8.789 m**，无真实接触；无视觉 PointGoal 对照在 E1 5.9 s 撞墙。E0/E2 均未触发绕行，动作块和对齐后的 XY 状态与各自无视觉对照逐项相同，分别 **13.9/14.0 s** 到达。因此这次 E1 不是靠停车或只改画面得出的“成功”，而是确实改变了执行轨迹并通过障碍。

**但这还不是可发布的通用传统视觉基线。** golden E1/E2 的画面和几何曾用于前几轮诊断；像素触发区、阈值及右侧绕行形状受这些历史开发场景影响。冻结同一模型和控制器后，额外两条完整 hard_0 E1 回归 **0/2 成功**：一条未检出障碍、8.0 s 接触，另一条虽在 0.1 s 触发，但起点附近 2.4 s 接触。当前可以确认“固定相机下 RGB 能触发一次真实的 E1 导航绕行”，不能把 golden 3/3 解释为跨场景泛化成功率。

## 候选从何而来

前轮的轻量像素模型在新光照+两条 hard_0 训练序列上，golden E1 起点中央墙召回 62.96%，低于预定 80% 感知门槛。为检查是否至少能形成**闭环控制事件**，本次只把它的左目障碍像素计数用作开发实验触发器；没有把它改写成“已通过分割模型”。当前图像每 4×4 像素采样一次，在固定图像区 `[40:100,40:140]` 统计模型输出 `obstacle` 的点数，连续两个 10 Hz 帧≥1,000 才锁存。模型在线只读当前 RGB、像素坐标和眼别；渲染深度仅用于前轮离线训练标签，当前仿真策略不读取深度、占据地图、Oracle 参考、episode ID 或未来观测。

第一版触发后使用两个离散右侧路标；E2 14.0 s 成功且与无视觉动作相同，但 E1 在路标交接后实际 yaw 约 −1.8 rad，低速无法快速朝下一路标转向，继续向下滚动，22.5 s 接触场景边界。相关记录在 [`E1 第一版轨迹`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/e1/episode_golden_e1_0001/trajectory_overlay.png) 和 [`首次方案`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/protocol.md)。

第二版只将**目标生成**改为连续参考：触发位置到当前 PointGoal 的前向轴上，右侧偏移在前 3.0 m 平滑增至 2.3 m，维持一段后在末尾 2.6 m 平滑回零；每 0.05 m 采样，单调投影后前瞻 1.0 m。像素模型、两帧触发、绕行方向、`GoalGreedyPolicy` 动作映射、roll PID `Kp=0.4, Ki=0.1, Kd=0.7`、相机及灯光均不变。代码见 [`policy_continuous.py`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/policy_continuous.py)，仿真前口径见 [`continuous_protocol.md`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/continuous_protocol.md)。这一差异把 E1 由离散路标失败变成实际完成的绕行，说明主要控制收益来自平滑目标交接，而非重训感知模型或换低层 PID。

同轮还预先尝试了**一个**更大感受野的三层 U-Net 分割候选，固定 12 轮和光度增强，在完整 E1/E2 留场景测试中 E1 中央墙召回仅 38.56%，E2 的 `free→obstacle` 为 7.04%，均未通过原感知门槛。它没有接入导航，也没有继续调结构或轮数。协议和结果见 [`context_model_v1`](../experiments/fixed_rig_visual_pilot_20261001/context_model_v1/protocol.md) 与 [`results.json`](../experiments/fixed_rig_visual_pilot_20261001/context_model_v1/results.json)。实际导航用的仍是前轮冻结的轻量模型。

## 完整导航结果

| 完整序列 | RGB 触发 | 终态 | 到终态时间 | 实际走行 | 目标方向进度 | 真实接触 |
|---|---:|---|---:|---:|---:|---:|
| golden E0 | 无 | 到达 | 13.9 s | 8.736 m | 8.686 m | 无 |
| golden E1 | 0.1 s | **到达** | **14.8 s** | **8.789 m** | 6.436 m | **无** |
| golden E2 | 无 | 到达 | 14.0 s | 8.811 m | 8.761 m | 无 |
| 额外 hard_0 E1 0003 | 无 | 碰撞 | 8.0 s | 4.217 m | 4.163 m | 有 |
| 额外 hard_0 E1 0004 | 0.1 s | 碰撞 | 2.4 s | 0.457 m | 0.335 m | 有 |

额外两条序列在控制器冻结后预先选定为 whole-sequence 回归，未用于当前像素模型训练（训练 hard_0 0001/0002），但与训练数据仍属同一个 hard_0 场景，**不是严格新场景 held-out**。完整口径见 [`validation_protocol.md`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/validation_protocol.md)。0003 最大左目触发区障碍点数 802，未达到 1,000，故仍按 PointGoal 直行而碰撞；0004 很快达到触发阈值，但障碍过近，固定右侧曲线未在实际接触前绕开。它们分别暴露**感知漏报**和**不按障碍距离/形状选路线**两个剩余问题。

E1 轨迹实测最低世界 Y 约 −2.62 m；第一次穿过中央障碍的世界 X=0 时 Y 约 −2.62 m，真实通过障碍下方后回到目标。图和视频可直接检查：[`E1 轨迹`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/e1_continuous/episode_golden_e1_0001/trajectory_overlay.png)、[`E1 双目视频`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/e1_continuous/episode_golden_e1_0001/stereo_view.mp4)、[`E1 第三人称视频`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/e1_continuous/episode_golden_e1_0001/third_person.mp4)。额外失败的轨迹/视频分别在 `hard_0_e1_0003/0004` 的 `episode_*` 目录，未混进 golden 成功统计。

## 证据与下一阶段

所有五条运行的控制源码和模型 SHA 相同，身份记录均标记 `privileged=false`、原相机、同一 roll PID；每条有原始 `metrics.json`、`states.npz`、`actions.npz`、`collision_audit.json`、双目及第三人称视频。独立 [`navigation_verification.json`](../experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/navigation_verification.json) 复核了原 35 s 规则、真实接触裁决、触发时刻、轨迹和文件身份。E0/E2 未触发时，动作块及 XY 轨迹与无视觉对照完全相同；E1 确实改变动作并实际成功。前轮标签/视差/视觉策略的 7 项回归测试通过，新增脚本可编译并完成 GPU 仿真。

要把这个结果升级为可泛化的视觉基线，优先保持已经在 E1 奏效的**连续局部参考和低层 roll PID**，替换当前固定图像阈值与固定右侧模板：用更多几何/材质独立的完整场景训练并校准前方障碍/未知检测；从观测中估计障碍距离、左右可通空间及制动余量，再据此生成安全侧绕曲线；最后用完全独立的场景序列验收 E2 不误转、E1 类障碍真实通过以及 35 s 到达率。当前 golden 3/3 是一个有价值的闭环可行性结果，额外 0/2 是必须保留的泛化拒绝证据。
