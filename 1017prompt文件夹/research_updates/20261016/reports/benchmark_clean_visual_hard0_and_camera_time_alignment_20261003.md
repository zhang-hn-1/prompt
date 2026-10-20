# 清洁视觉 hard_0 核验与相机时间对齐修复（2026-10-03）

## 研究结果

这轮确认并修复了一个真实的采集接口问题：原配置虽然写着相机10 Hz，但SDK实际约每0.2秒获取一幅新画面，中间一次10 Hz控制观测重复旧RGB和旧depth，同时配上新的机器人位姿。实测图像最大落后当前物理状态0.100秒。该错位会破坏跨帧测距，并使按当前位姿计算的旧图像空间标签存在误差。

修复后的独立实验入口完成了同一冻结视觉基线的五条实际运行。**golden E0/E1/E2保持3/3成功，hard_0 E1 0003/0004仍失败；没有新增导航成功率改善。**五条的请求、最终执行动作、实际运动、关节状态及相对评测时钟全部与各自原清洁B0相同。接口通过不意味着测距或规划通过。

唯一新增LK跨帧几何候选在同步数据上仍未通过，并未接入控制。没有训练新网络，没有启用Oracle地图、renderer depth或盲区倒车驱动策略。当前交付是一个通过实际验证的同步采集入口、一批同步双目完整运行数据，以及明确拒绝的几何前端；通用视觉导航baseline尚未完成。

## 固定条件和信息边界

相机仍固定在base_link的`(0.10, ±0.30, 0.10) m`，不改变安装位置、朝向或镜头。沿用`fixed_camera_fill_v1`、roll PID `0.4/0.1/0.7`、原协议/smoother/supervisor、原接触判定`physical_contact_v2`与**35秒**规则。

实际控制器仍是`ContinuousVisionDetourPolicy → VisionDetourPolicy → GoalGreedyPolicy`。当前已保留的B0由左目RGB像素触发固定右绕曲线；右目保存供双目候选离线验证，**本轮没有把右目宣传为已经控制了导航**。非特权reset与`GoalGreedyPolicy.act`只消费允许的PointGoal/观测，不消费Oracle参考。该基线使用benchmark定位与姿态，因此不是纯视觉定位或端到端世界模型导航。

所有536次完整运行capture逐次确认Start、Goal、OraclePath、ActualPath四个诊断图元不可见。深度与USD相机pose写在`offline_truth`，实际策略的`online_inputs`只有RGB、允许位姿/姿态/本体历史、PointGoal和实际请求。未隐藏场景自身的彩色障碍物。

## 原接口问题：从现象到实际时间戳

先实际重跑清洁hard_0两条：0003左目ROI最大794，未触发，8.0秒碰撞；0004在0.1秒触发右绕，2.4秒碰撞。请求和真实状态与历史运行相同（raw warmup时钟epoch可以不同）。真实首次接触分别7.94和2.34秒，每个50 Hz物理步均有可用接触记录。这排除了将地图净空误当真实接触的解释。

在已有五条清洁数据中，左右RGB/depth几乎每隔一次采样重复，机体位置和姿态仍在变化。0003重新运行的SDK时间戳直接证实：81次capture中，左右目各40次画面落后约0.1秒；首次画面落后0.02秒；重复SDK frame共40次。它不是机器人停住导致画面重复。

本机物理时钟每个动作块实测推进`0.09999999776482582 s`，SDK的10 Hz采集条件是累积时差达到`1/10 s`才更新；回调将累积量在采集后清零。这个微小浮点差在当前运行中产生了每两次render更新一次的现象。依据是安装的SDK源码`Camera._data_acquisition_callback`、实际clock日志及重复帧，不把此结论推广为所有Isaac配置的通用行为。

![实际相机时钟对照](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/camera_time_alignment.png)

## 修复、首次失败及最终接口验收

独立入口[`run_acquisition_v2.py`](../experiments/clean_visual_hard0_intervention_20261003_v1/run_acquisition_v2.py)让SDK消费每个完成的render；实际render和动作仍是10 Hz、物理仍是50 Hz。读取前要求左右相机`rendering_time`匹配当前raw物理时钟，过期时最多补三次**不推进物理**的render，不能获得当前画面则报错。每次刷新核验时钟、根位姿/速度、关节位置/速度完全不变；后续frame/time必须严格递增。

第一版只有一次初始刷新：hard_0 0003与E0通过，但E1首次仍拿到落后0.02秒的图像，被断言在动作执行前中止。这是接口未满足要求，不能算一个导航失败或成功。该记录保留于`synchronized_baseline/e1`。根据这次明确复现，加入有界的时间戳等待后，用同一V2源码重新运行全部五条，不用已中止的启动当正式结果。

最终五条左右目共536次成对capture：过期capture **0**、重复SDK frame **0**、最大实测图像时钟差 **0**；每次仅五个物理tick组成一个动作块。接触可用物理步共2655。相机实测外参旋转矩阵与允许姿态推导值最大误差低于`2.7e-7`；平面运动时相机中心误差p90低于`1.4e-5 m`。初始球心实际高度0.446m，几何候选使用0.400m时有0.046m高度差，已单列，未隐藏该近地误差。

修复只在本轮独立实验入口实现，**原冻结控制器和工程主runtime文件未改写**。新视觉采集应使用该同步入口/相同时间戳约束。旧日志、旧训练数据和旧模型拒绝gate保持原样；没有用新口径倒改旧成绩。缺乏曝光时间戳的旧帧不能直接宣称已按物理状态正确配准。

完整实测核验见[`acquisition_audit.json`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/acquisition_audit.json)与[`camera_pose_crosscheck.json`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/camera_pose_crosscheck.json)。

## 实际导航回归：同一B0与同一初始条件

| 序列 | 实际终态 | 用时s | 最终目标距离m | 实际路程m | 原视觉绕行触发s |
|---|---|---:|---:|---:|---:|
| E0 | success | 13.9 | 0.153 | 8.736 | 无 |
| E1 | success | 14.8 | 0.164 | 8.789 | 0.1 |
| E2 | success | 14.0 | 0.139 | 8.811 | 无 |
| hard_0_e1_0003 | collision | 8.0 | 3.822 | 4.217 | 无 |
| hard_0_e1_0004 | collision | 2.4 | 8.859 | 0.457 | 0.1 |

![实际轨迹对照](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/actual_navigation_regression.png)

灰色原B0与蓝色同步B0重合，因为请求和物理轨迹逐项一致。橙色星号是目标、绿色终点为到达、红叉为真实接触终止；绘图使用起点方向坐标，仅用于展示，不是策略的地图输入。

三条golden是已使用的研究场景，两条hard_0也是已查看开发序列。**新采集不等于fresh held-out场景**，不能凭此报告泛化成功率。golden成功按原目标距离、无接触、35秒判定；这次未新增严格到点停车验收。

## 唯一几何候选：同步通过后仍然拒绝

旧RGB分类器会把大量真实地面判成障碍，单帧地面投影不能直接把这些像素变成可信占据BEV。因此本轮只尝试一个无训练的经典候选：两眼各自将当前角点用金字塔LK跟踪到过去视图；经过双向跟踪、重投影、视差和至少两个过去视图的三维一致性检查，输出空间点。已知CAD base_link遮挡mask用于两端；没有重新启用旧NCC或RGB距离网络。全部参数在[`GEOMETRY_CANDIDATE.md`](../experiments/clean_visual_hard0_intervention_20261003_v1/GEOMETRY_CANDIDATE.md)预先固定，没有在新验证数据上扫描阈值。

将模型估计为障碍的点限定在离球心≥0.55m、世界高度0.25–1.5m，逐点用仅离线的renderer depth检查。冻结要求：三维点误差≤0.3m的比例≥90%，p90≤0.3m，E0/E2不能持续两帧报正前方风险，关键前方障碍需有足够覆盖。下表是真实三维点误差；不是终态预测，也不是轨迹误差。无效真值depth作为失败计入比例；p90只对有限真值列出，不能借排除无效点宣布通过。

| 序列 | 同步后障碍点数 | 真值depth无效点 | 原数据误差≤0.3m | 同步后误差≤0.3m | 有限真值p90误差m | 反事实前方触发s |
|---|---:|---:|---:|---:|---:|---:|
| E0 | 356 | 37 | 29.3% | 48.6% | 5.921 | 无 |
| E1 | 15 | 0 | 0.0% | 40.0% | 1.278 | 无 |
| E2 | 566 | 65 | 49.8% | 53.5% | 4.121 | 2.2 |
| hard_0_e1_0003 | 318 | 8 | 27.5% | 46.2% | 2.793 | 无 |
| hard_0_e1_0004 | 0 | 0 | — | 无可估计点 | — | 无 |

候选在E2仍于2.2秒产生前方误报；0003没有满足连续两帧的触发条件；0004没有可接受空间点。E1序列的B0早已绕行，因此不能用它单独验证“直行接近横挡”覆盖，不过其几何误差本身已不合格。同步改善了部分旧数据估计，但最关键的0003误差≤0.3m只有46.2%，离90%门槛很远。时间对齐不是所有感知错误的唯一原因。

因此**几何候选拒绝；左右曲线消费与改变动作的控制A/B未触发**。本轮不把离线反事实触发写成在线动作、不声称右目已改善控制，也不将未知区域当安全。不强行部署该前端再做一轮已能预见的碰撞。旧NCC、M2和RGB距离候选的原拒绝结论未改变。

完整逐帧点、真值误差和前方事件见[`temporal_tracks_audit_synchronized.json`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/temporal_tracks_audit_synchronized.json)，同期RGB/允许观测/真实物理运行均在`synchronized_v2`。

## 仍需解决的具体问题

0003首先是视觉空间解释不合格：原ROI漏检，新的几何前端也不能稳定测准和及时定位障碍。继续减速或调PID不能据此证明修复了漏检。下一项视觉工作应以**同步数据**检验当前左右目公共可见区域、像素对应与真正的前方/侧方空间证据，再消费曲线；不要再用每隔一帧错位的旧数据直接训练距离/选路模型。

0004另外有动作覆盖问题：它已检测到近障碍，但现有固定前向右绕曲线很快真实接触。此前三条曲线覆盖和物理原语实验已显示，仅在这些前向曲线中换阈值/选侧不够；曾在Oracle地图下有效的后退动作，在当前初始双目视野中没有后方空间依据，不能直接移植为视觉成功。这里需要一条可观测空间支持的可执行局部路径，或者明确将当前姿态下不可观测的动作视为未知。停车只算失败，不能充当导航改善。

当前保留的可复现视觉成果是三条golden实际成功及其数据；尚缺能应对以上困难起点的可靠空间感知和局部路径覆盖。该结论没有把任务转回Oracle调参或要求世界模型先通过。

## 测试、运行证据与限制

本轮10条完整实际运行、1次启动接口断言中止均保留；最终比较使用V2的五条完整运行。最终源码和模型SHA、数据边界、事件gate、接口/模型/控制结论分别保存于[`final_verification.json`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/final_verification.json)。旧源码身份的两个版本保存在`frozen_sources`，第一版原始内容SHA与当时记录一致。

- 本轮实验测试 **12/12通过**，包含真实trace的时间戳/不额外推进物理、三维几何接受与误匹配拒绝、有界刷新与拒绝未来帧。记录：[`experiment_pytest.txt`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/experiment_pytest.txt)。测试先观察失败的记录为`tests_red.txt`和`clock_tests_red.txt`。
- 原工程裸`pytest`因历史实验目录同名模块/导入路径等出现27个收集错误，完整名字和原因见[`project_full_pytest.txt`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/project_full_pytest.txt)，没有声称整个工程全绿。
- 原工程主`tests`目录：1224通过、64跳过、3失败。正确工程根目录单独重查三条：路径相关的`test_reference_motion_reconstructs_curved_planner_primitive`通过；`test_reference_rollout_solves_hash_verified_hard_0_e1_0002_static_input`仍为规划timeout，`test_harness_writes_required_artifacts_and_detects_config_tampering`仍因测试runtime缺`_contacts`失败。当前两项未解决，本轮未修改这些工程模块。完整记录：[`project_core_pytest.txt`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/project_core_pytest.txt)、[`project_failure_recheck.txt`](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/project_failure_recheck.txt)。

实际新视频：

- [E1第三人称](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/synchronized_v2/e1/episode_golden_e1_0001/third_person.mp4) / [E1双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/synchronized_v2/e1/episode_golden_e1_0001/stereo_view.mp4)。
- [0003第三人称](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/synchronized_v2/hard_0_e1_0003/episode_hard_0_e1_0003/third_person.mp4) / [0003双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/synchronized_v2/hard_0_e1_0003/episode_hard_0_e1_0003/stereo_view.mp4)。
- [0004第三人称](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/synchronized_v2/hard_0_e1_0004/episode_hard_0_e1_0004/third_person.mp4) / [0004双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/clean_visual_hard0_intervention_20261003_v1/synchronized_v2/hard_0_e1_0004/episode_hard_0_e1_0004/stereo_view.mp4)。
