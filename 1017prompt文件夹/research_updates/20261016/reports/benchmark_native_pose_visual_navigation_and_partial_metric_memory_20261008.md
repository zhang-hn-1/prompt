# 原生位姿导航与部分双目测量：阶段证据

研究仍未完成。固定相机、光照和 roll-PID 不变，以下属于独立标注的平坦地面先验/安全起步区任务；不是原始严格冷启动或35秒 E0/E1/E2。

## 实际导航及分层结果

一次完整60秒运行（含14秒观察）已结束并独立核验。600请求、3001物理采样、601采集、121双目推理、两路601帧视频；1913源码身份与6462封存产物已核验。原生 IMU/编码器加冻结滚动积分与显式平面高度先验，位置 RMS 1.50cm、峰值3.55cm、姿态90分位0.982°，通过本任务位姿门槛。初始全位姿仍特权给定，底层 PID 仍使用理想IMU，不能称纯RGB。

旧 RGB 可能集和射线排除得到185/185局部障碍代理覆盖，通过的是 MODEL 来源审计，并非稠密实测障碍或自由空间。该次导航没有消费移动深度几何。机器人实际通过障碍，接触为0、代理最小间距0.655m；但最近真实三维目标距离0.21194m，终态0.53623m，60秒超时，导航拒绝。独立完整参考横向 RMS 0.08583m、峰值0.20987m，不是同参考控制A/B。预封存的20°导航验证因到达门槛失败未触发。

证据：`native_flat_pose_navigation_R1_run_v1` 的原始数据/视频；`native_flat_pose_navigation_R1_sensor_verification_v1.json`、`native_flat_pose_navigation_R1_source_verification_v1.json`、`native_flat_pose_navigation_actual_assessment_v2.json`。评分v1继承的“仅 primitive 改变”末尾元数据错误由独立v2补充纠正；原v1字节、数值、门槛和失败不变。

## 控制因果核验

独立克隆控制器按当前封存输入复现460个请求，逐一精确相同，没有二次调用在线有状态 policy。唯一终点几何容差0.03m候选增加两次换向，在CPU准入拒绝，未运行。

唯一 goal_braking 副轴来源候选只在555/556帧改变请求。单步测量反馈探针输出相同，之后连续保存执行状态的5个物理步也全部 applied 相同，但 PID 内部状态不同。因此只支持“这5步没有实际输出干预”，不支持整段行为等价。保留初步拒绝；未触发新的GPU控制实验，也没有扫描另一容差、PID或副轴阈值。

证据：`native_terminal_factual_chain_v2`、`terminal_goal_bank_source_probe_v1`及 `audit_terminal_goal_bank_stateful_v2.py` 的封存结果；660项CPU测试通过、3个既有警告。

## 移动双目测量

对该次全部121对在线预测作独立离线评估。原严格可见障碍重叠区域251687个查询中，仅33.84%有有限深度，33.29%误差不超过10cm；有限样本误差90分位6.67cm。地面深度覆盖与误差均不足。原稠密来源门槛明确拒绝，没有用只看正确点、动态遮挡过滤或平面先验来改判。

这提示主要缺口是观测覆盖，不能把缺失位置写成自由。随后预先冻结了一项独立“部分正向障碍表面”开发诊断：只取已有真实深度点且左右当前RGB分类均支持的像素，不读取真值选择点，不膨胀成整物体；逐点保留世界坐标、两眼像素、采集时间与来源哈希。全部选择及121条历史观测先封存，再读取真值评分。

结果：74700个选中点，97.767%世界位置误差≤10cm，90分位7.122cm，障碍类别精度99.9987%；49/121个曝光有这类点、4个曝光尚无因果分类器。仅通过开发数据上的部分正测量质量门槛。它不是fresh held-out、在线记忆验证、完整占据/自由安全证明或导航成功，仍需独立完整序列验证才准入控制消费。

首次审计因名义0.1秒曝光与 float32 物理时钟最多1.341微秒差异在读取真值前停止，原v1保留。v2按同帧身份、准确名义曝光、float32实际时钟检查，未修改科学门槛。

证据：`native_moving_stereo_metric_audit_v1/assessment.json`、`partial_metric_surface_memory_preflight_v2/memory_seal_before_truth.json`、`partial_metric_surface_memory_preflight_v2/observations`、`partial_metric_surface_memory_preflight_v2/assessment.json`。

## 尚未完成的消费链

下一项必须冻结这一个部分表面来源的当前RGB分类和测量接口，以完整独立序列验证来源与过去记忆；缺失仍未知，MODEL、平面先验与部分实测分层。通过后才做移动记忆接入的单环节实际闭环，不能继续仅用固定14秒 MODEL 图冒充动态双目导航。终点执行控制仍未解决；不重新扫参数救分。学习记忆/世界模型增量与原35秒 E0/E1/E2 均未启动，本报告不是整个研究交付。

## 独立完整序列：部分来源拒绝（追加，原开发结果保留）

2026-10-07 19:13:20—19:20:48 UTC，唯一60秒 yaw+20 预知命令来源诊断退出0，壁钟447.71秒。600请求/3001物理/601采集/121双目，1964来源、11输入、2066封存产物核验；3000执行步精确重建，两视频601帧/10Hz，接触0。这是独立整段来源验证，动作由封存请求给定，不是导航。

冻结原生模型再次通过本任务位姿门槛：RMS1.540cm、峰值3.540cm、姿态90分位0.8494°。但121次部分正测量均为零，明确拒绝该整段来源候选；不存在可以评分的正向点。旧开发74700点通过不代表新朝向可用，更不能假称记忆为空就是已观测自由。

源几何、当前分类与不可变记忆在读取真值前重建。之后的全部121曝光诊断显示：左眼真实深度中没有任何圆柱表面像素，右眼54次有。未使用静态mask或范围裁剪仍如此。圆柱全表面的几何投影在左眼24次进入理想画幅、右眼55次，说明不能把左眼缺失只归因FOV边界；部分时刻还存在实体遮挡/渲染可见性限制。实际RGB图已亲自查看，固定机体遮挡很重。首要阻塞发生在两眼共同可见性层，不能据此断言冻结分类器或网络本身必然错误。

证据：`partial_surface_source_validation_run_v1` 两路视频/原始RGB、`partial_surface_source_runtime_verification_v1.json`、`partial_surface_native_pose_assessment_v1/assessment.json`、`partial_surface_source_assessment_v1/assessment.json`、`partial_surface_zero_coverage_cause_v1`、`partial_surface_raw_fixture_visibility_diagnostic_v2.json`、`partial_surface_fixture_FOV_projection_diagnostic_v1.json`、`partial_surface_validation_native_stereo_v1.png`。raw可见性v1只有6帧描述，v2全部121帧为最终范围；不挑最好帧替代整段门槛。

下一唯一候选限定在14秒安全起步观察任务：用当前原生估计相机光轴和已知粗目标方向，改变观察请求，最多40个10Hz转向块且估计位移0.5m硬退出，随后原bank请求停车。原0.7m安全区、PID/速度范围/相机/光照/NN不变。目标是核验共同可见性与部分来源，未实施控制导航、未通过新门槛；旧迟发course/camera-heading观察与严格来源拒绝仍保留。候选已冻结规格及4项CPU测试，但实际集成/审查/物理运行尚未完成。


14秒观察与历史RGB支持已实际结束并拒绝；后续结果见 [benchmark_native_camera_goal_observation_and_temporal_RGB_memory_20261008.md](benchmark_native_camera_goal_observation_and_temporal_RGB_memory_20261008.md)。旧 pending 文本按字节保留，不能再解释为当前运行状态。
