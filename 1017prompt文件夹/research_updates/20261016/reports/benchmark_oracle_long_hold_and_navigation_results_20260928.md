# 延长转弯保持核验与完整 Oracle 导航结果

日期：2026-09-28。已完成4条新开放平面保持试验和10条正式导航运行，全部真实执行。正式导航使用当前版本Oracle、真实位姿和地图、原始35秒预算，不是视觉或世界模型导航。没有修改正式控制器、planner、摩擦或成功规则；本轮交付当前基线完整评测，不声称新控制算法取得提升。

## 一、实际导航结果

| 套件 | 成功 | 原规则碰撞判定 | 超时 | 运行错误 |
|---|---:|---:|---:|---:|
| golden E0/E1/E2 |3/3|0|0|0|
| hard_0 dev全部7条 |3/7|1|3|0|

hard_0的3条E0均成功，4条E1均失败。golden的E0与hard_0中的cluttered-0_e0_0002是相同几何/初态任务，只是ID、split、review说明不同；不把10次运行当作10个独立泛化案例，也不提供合并成功率。golden只有每种难度一个健全性例子，3/3不代表所有E0/E1/E2均可靠。

成功采用正式0.2m首次进入规则，不要求此后驻留停稳；此前局部终点反馈的严格驻留门槛没有偷偷替换正式口径。局部模板、重放参考与终点反馈没有未经泛化验证直接装入正式Oracle。

| 套件/任务 | 结果 | 用时(s) | 结束距目标(m) | 实际路程(m) | SPL |
|---|---|---:|---:|---:|---:|
|golden_e0_0001|到达|22.5|0.195|9.127|1.000|
|golden_e1_0001|到达|32.7|0.198|9.258|0.955|
|golden_e2_0001|到达|17.8|0.188|8.937|0.996|
|cluttered-0_e0_0002|到达|22.5|0.195|9.127|1.000|
|cluttered-0_e0_0003|到达|26.3|0.192|9.694|1.000|
|cluttered-0_e0_0011|到达|18.9|0.170|7.859|1.000|
|hard_0_e1_0001|超时|35.0|5.184|7.901|0.000|
|hard_0_e1_0002|地图碰撞判定|16.1|8.204|2.617|0.000|
|hard_0_e1_0003|超时|35.0|3.151|8.825|0.000|
|hard_0_e1_0004|超时|35.0|9.503|2.494|0.000|

### 必须单列的碰撞口径问题

正式collision字段来自冻结占据地图净空<=0，并不等于物理接触传感器。

- hard_0_e1_0002在16.1秒因地图净空−0.0394m终止；保存的物理接触记录为0。
- hard_0_e1_0004正式被记为35秒超时，collision=0，但从14.3秒开始有物理障碍接触记录，共205个保存状态；第一接触时地图净空仍+0.0472m，最终也为+0.0472m。

因此不能把第四条解释为“无接触但进度慢”。本报告保留原始评分，同时标注实际接触。这个分歧不改变本轮成功数（该条本来就失败），但影响碰撞率解释和失败归因。未在本轮倒改benchmark规则；若修复必须独立版本化并对旧结果重算，不覆盖旧结果。

hard_0_e1_0001/0003全程未进入tracking_infeasible永久停车分支，分别实际走7.90/8.83m后超时，距目标仍5.18/3.15m。不能继续将所有失败都归为第一次换向或“停住不走”。0002/0004包含方向制动，之后出现地图侵入或物理接触。

## 二、转弯保持试验修正了什么

只补4条固定条件：主轴0.5/1.5rad/s × 副轴±30°。每条先5秒副轴零，然后保持30秒；目标变化率、副轴速度supervisor与原配置一致。没有扫描新角度或重新建立完整标定矩阵。

新实验的预注册末段判据包括倾侧、速度及曲率稳定性，历史Round4 gate不变。10–15秒低速窗口仍因曲率波动未通过；20–25秒和30–34秒均通过。高速条件三个窗口均通过。末段30–34秒结果：

| 主轴请求 | 副轴请求 | 实际速度(m/s) | 实际倾侧(°) | 曲率中位数(m⁻¹) | 曲率IQR(m⁻¹) |
|---:|---:|---:|---:|---:|---:|
|0.5|−30|0.180|−21.81|+0.794|0.026|
|0.5|+30|0.175|+24.67|−0.918|0.050|
|1.5|−30|0.545|−15.91|+0.772|0.082|
|1.5|+30|0.536|+18.70|−0.822|0.085|

**此前短窗口低速曲率−1.74m⁻¹不能当作长期稳定能力。**延长保持后，低速+30°约−0.92m⁻¹，与高速−0.82m⁻¹接近得多；负转向两者约+0.79/+0.77m⁻¹。低速早期大曲率包含明显动态过渡。实际速度、倾侧和执行角仍同时变化，不能据四条试验识别独立因果或通用理论公式。

这四条结果不足以验证新的完整驱动模型，尤其未覆盖倒车、换向、中间角度与起始状态。没有为了拿导航成绩直接改V1映射。正式导航使用已有已验证入口，这与用户要求先取得真实完整导航结果一致。

## 三、版本、验证和复现

- 当前源码与2026-09-22冻结包的若干文件不同，旧复跑身份检查拒绝执行，未改旧哈希。新导航前保存当前源码/配置快照和逐文件SHA-256，全部导航完成后重新核验未变。
- golden三个任务的主要指标、实际XY和所有动作字段与2026-09-22历史逐元素一致；这证明这三个案例行为复现，不代表所有源码相同。历史包与本轮包均保留。
- 57项manifest、场景、smoother和runtime合同测试通过。
- 两套正式runner退出码均0，每个任务均发布完整审计包；失败任务作为正常评测结果计入，而非删除。4条保持试验也全部完成。
- 20个第三人称/双目视频均检查帧数、FPS与首尾可解码；不是逐帧视觉正确性证明。原日志中的路径标记渲染警告保留，运行错误计数0。
- 7项历史顶层场景/occupancy/机器人资产哈希匹配。此包依赖当前机器Isaac环境与外部资产路径，不是跨机器自包含安装包。
- 保存的物理接触是正式状态采样记录；本报告未把它伪称为所有50Hz接触事件完整列表。

复跑入口 `reproduce_navigation.py` 先核验冻结源码和资产，目标目录必须不存在。`--suite golden`只跑3条，`--suite hard_0`跑7条，默认all；`--check-only`已实际通过，未重复启动额外导航。未来源码变化时应在独立目录恢复快照，不直接更新旧哈希。

同机复跑示例（需使用全新输出目录）：

```bash
/home/an/miniconda3/envs/ball_wm/bin/python /home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/reproduce_navigation.py --run-root /tmp/oracle_navigation_repeat_20260928 --suite all
```

独立审查已复核指标、重复任务、接触口径与复跑检查；不将单场景bootstrap退化区间当作泛化置信度。

## 四、交付边界

已经有可复现的、带速度限制的Oracle完整导航成功结果，也有全部困难开发任务的明确失败边界。并未完成困难E1、传统视觉基线或世界模型闭环。下一项最高优先级是核验物理碰撞体与冻结地图的安全一致性，并围绕困难E1验证有执行时间余量的参考；不应仅继续调终点停车阈值。

## 五、全部轨迹和视频

- golden_e0_0001：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e0_0001/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e0_0001/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e0_0001/stereo_view.mp4)
- golden_e1_0001：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e1_0001/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e1_0001/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e1_0001/stereo_view.mp4)
- golden_e2_0001：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e2_0001/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e2_0001/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_golden/episode_golden_e2_0001/stereo_view.mp4)
- cluttered-0_e0_0002：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0002/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0002/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0002/stereo_view.mp4)
- cluttered-0_e0_0003：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0003/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0003/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0003/stereo_view.mp4)
- cluttered-0_e0_0011：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0011/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0011/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_cluttered-0_e0_0011/stereo_view.mp4)
- hard_0_e1_0001：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0001/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0001/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0001/stereo_view.mp4)
- hard_0_e1_0002：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0002/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0002/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0002/stereo_view.mp4)
- hard_0_e1_0003：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0003/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0003/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0003/stereo_view.mp4)
- hard_0_e1_0004：[轨迹图](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0004/trajectory_overlay.png) · [第三人称视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0004/third_person.mp4) · [双目视频](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_hard_0/episode_hard_0_e1_0004/stereo_view.mp4)

[保持试验曲线](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/long_hold.png) · [原始导航汇总](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/navigation_assessment.json) · [复跑入口](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_oracle_long_hold_navigation_20260928_v1/reproduce_navigation.py)
