# 固定机体遮挡、曲线资格与清洁视觉基线复验

日期：2026-10-03。独立实验目录：/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1

## 本轮结果

**原先成功的左目连续视觉策略，在去掉 Oracle/轨迹诊断标记后的真实相机输入下，重新跑通 golden E0/E1/E2 三条完整导航。** 时间分别 13.9、14.8、14.0 秒，均无真实接触；动作和实际运动状态与历史成功版本逐项一致。这解决了上一轮留下的“历史成功是否依赖渲染提示”疑问，覆盖这三条已知开发序列。

另一项结果是：由已知 CAD 生成 base_link 自体遮挡 mask，剔除了旧双目时间几何回归中的近体假障碍点；但**环境距离仍不准，感知候选继续拒绝**。净空资格模块及测试已完成，因感知失败，没有接入正式导航，也没有把停车作为改善。两个结果严格分开：本轮实际成功的是未改动的旧左目控制器，不是新遮挡修复后的双目控制器。

相机位置/光轴、灯光、roll PID 0.4/0.1/0.7、主副轴限制、物理接触判定和原 35 秒规则均保持不变。未重训网络，未修改 Oracle/planner，未扫描 NCC 或控制阈值。[运行前冻结协议](../experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/PROTOCOL.md)。

## 清洁视觉完整导航

实际运行仍为 `ContinuousVisionDetourPolicy → VisionDetourPolicy → GoalGreedyPolicy`。旧像素模型、左目障碍 ROI、1,000 点阈值、两帧锁存、固定右侧连续曲线和父类动作映射均未改动。右目只记录，不参与这个旧策略的控制；定位/姿态仍由 benchmark 提供，不称纯视觉定位或世界模型导航。

| 完整序列 | 左目触发 | 真实结果 | 时间 | 实际走行 | 终态目标距离 |
|---|---:|---|---:|---:|---:|
| golden E0 | 无 | 到达，无接触 | 13.9 s | 8.736 m | 0.153 m |
| golden E1 | 0.1 s | 到达，无接触 | 14.8 s | 8.789 m | 0.164 m |
| golden E2 | 无 | 到达，无接触 | 14.0 s | 8.811 m | 0.139 m |

E1 实际穿过世界 X=0 时 Y≈−2.623 m，真实绕过中央障碍再到达。不是提前跳索引或停在障碍前。

![清洁输入下 E0/E1/E2 的实际轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean_e0_e1_e2_actual_trajectories.png)

背景占据图只用于事后绘图，不进入在线策略。绿线为实际完整轨迹，E1 蓝虚线为左目触发后生成的局部参考。

三条新完整序列每次采样前核验 Start、Goal、OraclePath、ActualPath 的 computed_visibility=invisible；核验次数分别 140/149/141。全部 50 Hz 物理样本均有可用接触记录且无接触，源码及模型 SHA 匹配，原 budget=35 s、runtime 无异常。

与历史对应 e0/e1/e2_continuous 成功记录比较：

- 全部 action_blocks 与 controls 数组逐项完全一致。
- 全部 XY、yaw、线速度、角速度、关节位置、关节速度、applied action 数组逐项完全一致。
- 原像素模型 SHA 相同：`f775766dfabbd5a48f7289c8a4538c749ec8c6cceb1b6a9654d12a29861015e2`。

这说明隐藏诊断提示没有改变这个冻结控制器在上述三条序列中的动作。历史与当前均是已知开发场景，**3/3 不是新场景泛化成功率**。旧 hard_0 E1 0003/0004 的 0/2 拒绝证据保留，本轮未重跑它们。到达后停车保持也未测试，三条终态 XY 速度约 0.735/0.668/0.725 m/s。

详见[独立导航核验](../experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean_assessment.json)与[实际运行及完整命令](../experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_run_status.json)。

## 机体遮挡：实现、覆盖与限制

读取机器人已知 CAD base_link 网格（366,112 个三角形），按冻结相机光心与 optical-to-base 投影，裁剪相机后方/近裁剪面，并膨胀三像素作为匹配边界。mask 不使用场景 RGB、地图、renderer depth 或未来反馈来构建，没有拟合场景标签；[CAD 层 SHA 与 mask 身份](../experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/mask_identity.json)保留全部来源。

左目 mask 覆盖 117,591/307,200 像素（38.278%），右目 117,567（38.271%）。只覆盖与相机刚性固定的 **base_link**；不声称包含其他随关节运动的部件。没有移走真实机体或改渲染透明度。

![CAD mask 在实际 RGB 上的位置](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/cad_self_mask_on_actual_rgb.png)

红色为排除区，黄色为轮廓；不是新相机视野。已知遮挡从“未知外部障碍候选”中单独排除，不因此把未见空间标成自由。

保持旧 NCC 搜索与全部参数，只改当前角点/块以及过去重投影匹配块的可见资格。使用上一轮 clean_render_v2 E1/E2 的完整已录图像、允许位姿做**离线回归**；renderer depth 仅在事后评分。两条都已用于诊断，不能称 fresh held-out，也不能把回放产生的事件当作改动后的真实闭环事件。

| 在同一旧轨迹上回放 | 未 mask | CAD mask 后 |
|---|---:|---:|
| E2 近体假障碍点 | 16 | 0 |
| E1 近体假障碍点 | 106 | 0 |
| E2 地面假障碍点 | 27 | 24 |
| E1 地面假障碍点 | 8 | 3 |
| E2 全部接受点误差≤0.5 m | 18.89% | 21.33% |
| E1 全部接受点误差≤0.5 m | 15.55% | 51.79% |
| E2 可测场景点误差≤0.5 m | 24.82% | 22.73% |
| E1 可测场景点误差≤0.5 m | 43.14% | 52.73% |

近体指真实深度反投影后距球心≤0.55 m 的几何代理，不是实例分割的刚体身份。假障碍数是重复点观测，不是独立物体。角点预算释放后会选到更多环境点，故点集/分母改变；不能只比较比例就声称同一组场景点逐个变准。

mask 后 E2/E1 分别接受 211/112 个点，可测场景点为 198/110；场景 p90 仍 **3.125/1.539 m**。全部点 p90 为 4.257/1.620 m，远未满足旧≤1 m门槛。在 4×4 采样网格上，mask 下可测像素中落到非近体区的比例为 1.332%/1.430%；这是遮挡边界与近体代理的误遮挡诊断，不是严格实例分割精度。

E1 在这条旧候选轨迹的回放中，连续中央阻挡事件从 2.4 s 变为 **5.1 s**；说明原早期触发有近体/地面假点的支持。不能把这个 5.1 s 描述成新策略在 B0 上的实测触发，因为 2.4 s 后旧轨迹已经分叉。

E2 的假阻挡只在旧策略分叉前 t<2.1 s 的 **21 帧**评估，2 帧阻挡，比例 9.52%。这个窄前缀满足本轮≤10%数字条件，但仍产生 2.0 s 两帧事件，**不能据此宣称 E2 误转问题已解决**；它也不能与旧整段帧比例直接等同。口径和分母在本轮协议中预先说明，旧 gate 未追改。

最终 gate：场景点数量通过，距离准确性失败，E1 4秒前事件失败，E2 窄前缀比例通过；总体 **拒绝**。[原始回归结果](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/mask_regression.json)。

## 曲线资格检查：完成测试，未部署

新增独立资格模块要求候选对已观察点净空≥球半径0.4+既有裕度0.1=**0.5 m**。两条都不足时返回无可执行侧，不能只挑相对更大的一条。无观察属于未知；仅有远处障碍点也不能证明曲线扫过的整个空间自由。即使净空数字够，还需要独立的 swept-region 可通证据；当前视觉流程没有提供这项证据，调用方不能自行假定为真。

该模块只评估接下来前向三米，后续完整曲线仍未认证。对旧实际触发点云复查：

| 序列 | 左净空 | 右净空 | 资格输出 |
|---|---:|---:|---|
| E2 | 0.039 m | 0.028 m | 两条均拒绝 |
| E1 | 0.029 m | 0.010 m | 两条均拒绝 |

见[真实记录点云的资格检查](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/recorded_curve_qualification.json)。这是已有点云的控制入口核验，**没有重新执行这些动作，没有形成新的控制 A/B，也没有改变历史请求**。感知仍失败，故没有把该模块强行接进导航；不能声称它已经提高避障率，也不把返回无曲线后停车算成完成导航。

## 实际测试、分层结论与下一项工作

测试先出现预期失败，再完成实现；整个独立实验目录 bare pytest **13/13 通过**：三项 CAD 三角形投影/近裁剪、四项原几何回归、两项 mask 输入消费、四项曲线资格。见[完整测试原始输出](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/full_tests.log)，并保留 mask、geometry、route 的 red/green 记录。实际三条 GPU 运行均完成，不是旧日志回放冒充新运行。

| 验证层 | 本轮状态 |
|---|---|
| base_link 已知遮挡排除 | 在两条旧完整回归上消除近体假障碍点；其他运动构件与误遮挡边界仍有限制 |
| 双目环境距离 | 继续拒绝；重复纹理/错误对应仍未解决 |
| 不足净空候选的资格条件 | 模块和测试完成；无完整自由空间证据，未接入导航 |
| 冻结旧左目视觉策略 | 清洁输入实际 golden 三条通过，动作/运动状态完全复现 |
| 通用视觉基线 / 世界模型导航 | 尚未完成；hard_0 失败和泛化问题保留 |

下一项应以这次已经验证过的**清洁视觉运行入口与旧成功控制器**作为固定对照，针对 hard_0 的漏检及近障碍路线不适用改善视觉决策。CAD 遮挡排除可以保留作输入预处理工具，但不能继续把当前 NCC 距离输出当作合格空间量。新的环境距离/可通空间证据须用独立完整场景验证后，才允许消费曲线；本轮没有再扫阈值或换网络。

## 视频与文件

- [E1 实际第三人称视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean/e1/episode_golden_e1_0001/third_person.mp4)、[E1 实际双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean/e1/episode_golden_e1_0001/stereo_view.mp4)。
- [E0 第三人称](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean/e0/episode_golden_e0_0001/third_person.mp4)、[E2 第三人称](/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean/e2/episode_golden_e2_0001/third_person.mp4)。
- 每条 `legacy_clean/e*/` 保留原始允许输入、独立评分深度、physics、状态/动作、碰撞审计、每次渲染可见性查询、模型/源码 SHA。
- 旧双目拒绝记录在 /home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2，旧成功策略代码与模型未改动。



## 后续独立采集时钟核验（2026-10-03）

[后续报告](benchmark_clean_visual_hard0_and_camera_time_alignment_20261003.md)通过SDK实际时间戳发现：此处旧序列相机数据约每两个10Hz控制采样更新一次，中间的旧RGB/depth配到了更新后的物理状态。该现象影响旧几何估计和标签的时间口径；保留本报告原数值和拒绝结论，不倒改gate。使用独立同步入口重新实际运行golden三条及hard_0两条，所有536次capture均同步，五条的请求/动作/物理轨迹保持不变。因此golden实际成功与困难场景实际碰撞结论不变；旧测距标签不得未经曝光时刻核验就视作同步空间真值。
