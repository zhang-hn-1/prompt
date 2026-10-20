# CAD 去自遮挡后 RGB-D 位姿候选：完整历史序列拒绝

唯一未训练候选实际执行25个source曝光/24相邻pair，6个局部增量被旧PnP规则接受，但累计轨迹在首个pair失效，之后保持未知。累计有效仅初始identity，1/25=4%；运动期13对完整分母0正确，没有finite姿态误差样本。**拒绝接入导航，不启动此候选在线运行**。近乎零的初始点误差不是位姿预测性能。

## 边界与唯一结构

设计/计划 `2026-10-07-cad-masked-rgbd-odometry*` 在导出/预测前冻结。旧 strict stereo-ORB失败在几何三角点不足8；新结构复用已经真实执行的 Fast双目metric depth、CAD current-joint mask、旧固定ORB/temporal descriptors/旧PnP，不修改旧三角点规则或旧拒绝。Fast旧严格depth gate仍拒绝，这不是depth准入证明。

预测输入只有固定native K、上一/当前曝光时间与RGB ORB xy/descriptor、上一帧Fast depth/bilateral CAD-valid mask。深度 .5..3米、disparity>1、±1disparity precision≤.1米、固定rig到球心距离≥.55米；根距离在固定rig坐标计算，不用world GT位置。CAD三关节来自旧公共proprio，生成mask不读root pose。完整ORB descriptor patch不能接触CAD mask；没有补洞。取最近像素depth，用subpixel feature ray；该近似未经独立误差认证，保留限制。

时间Hamming64/ratio.75/mutual，PnP8landmarks/inliers、inlier比例.5、100次/.999/seed3072、reprojection1.5px；完全复用旧参数，不扫阈值。每次当前←上一相机变换求逆累积，失败永久unknown、不identity补、不重置、不晚期GT对齐。当前depth/worldscene/IMU/gyro/GTpose不进增量接口；输入poison/futuretime在fixture验证。确定性代码没有拟合网络。

## 完整序列与实际数据

来自已结束的 nominal_cap_history_run，122物理camera captures中的25网络source曝光0,5,…120，0–12秒，2Hz网络source时域。其他97camera帧不被偷偷插入depth；pose诊断是完整25-source序列，不是122RGB全部有效覆盖。原raw RGB/mask/depth保存，25五字段输入另存 cad_rgbd_odometry/inputs/，不含世界pose。先预测完整并保存SHA/completion，再运行独立GT评价。

这是已有 cap/mask/physics标签已看过的历史development序列，不能称fresh held-out；不是新GPU/采集、不是在线接口验证、未改任何执行动作。初始GT opticalpose只用于事后评价gauge，逐帧knownpose仅评价标签，不能反馈到predictor。

| 冻结门槛/分母 | 实测 | 结论 |
|---|---:|---|
|完整累计有效≥90% |1/25=4% |失败 |
|≥6s完整13pair中≥90%平移≤.05m/旋转≤2° |0/13 |失败 |
|同分母finite p90≤.05m/2° |没有finite |失败/不能估计 |
|累计3D RMS≤.05、peak≤.10、旋转p90≤2° |仅t0 trivial≈0 |不能证明精度，覆盖已失败 |

24pairs：17不足landmarks、1不足inliers、6PnP增量accepted（逐条身份见predictions.json）；首次0→.5秒仅4共同metric点，累计链当场拒绝。接受的六个局部增量均发生在动作起步前的旧观察/落地晃动阶段，不能拼接为完整轨迹。≥6s全部13pair没有合格输出，没有在线导航准入。

## 有界层级原因核验

单独 source25层级计数，不改变任何预测/gate。6s ORB71features/28可用depth；7.5s ORB68features却confidence合格0；8s68features/18depth，8.5s54/15，9–11s仍有37–55features但depth confidence支持全0；11.5s43features中21有confidence，全部因原.5..3m范围不合格。12s33features而confidence0。

因此不是“图中完全没有特征”，而是当前固定图像和depth域下**可跨帧匹配且具有可靠metric depth的共同支持不足**；还包含一个inlier一致性失败。不能直接归咎于PnP增益或随意降低8点条件。部分时刻confidence之前已损失全部支持，CAD/precision不是那些时刻的直接原因；此有限诊断不证明所有视觉方法不可行。没有扩大深度范围、重选特征数量、晚期重启、换网络或把renderer depth做观测。

## 实际测试、身份与接续

新增4fixture ModuleNotFound真实RED→GREEN，连旧7VO/PnP共11pass.29秒。实际runner exit0，25payload+75raw/source/mask+6code/spec共106SHA，predictioncompletion标记无truth读入；auditor exit0，pose gates拒绝和分母见 cad_rgbd_odometry/assessment.json。逐层诊断 exit0，support_layer_diagnosis.json，完整run/log/predictions、旧失败全部保留。原candidate cap1.0终距no-regression失败/完整goal未触发，旧source/IMU/M2拒绝均保持。全379suite是早前证据，不称本次重跑。无新视频（本历史数据真实12.1s两视频在原run）。

接续不能拿t0零误差或六个局部increment称估计位姿成功。下一项保留**原.3限速的当前源baseline**，只将已局部验证的CAD过滤移植到原完整6.6米/35秒诊断，并在同实时masked源上跑history只读shadow，补齐真正移动、遮挡与记忆消费的完整任务数据。它不是被拒绝cap1.0的完整goal准入，不改变控制/速度或放宽终距，不将历史memory无增量改写为改善；只测试已闭合sourcefilter在原完整goal的行为与失效层次。original fullgoal旧mask已完成不重复同矩阵。仍knownpose/groundprior/controlledunknown，不能报告pureRGB或正式 E0/E1/E2。

## 独立终审与作者追加（保留前述正文）
ONE fresh gpt-6-astra/high C0/I0/M0，personal11fixture、106/106SHA、25input白名单/曝光、ORB/depth门控/计数精确重建、首4landmarks/1of25覆盖、24pair六accepted/17点不足/1内点不足、运动13pair0finite、六有限增量GT误差重算。评审没有重跑GPU原数据/网络/RED历史，也不评价下一个cap.3完整baseline。原记录cad_rgbd_review_final.md/json。接受其科学REJECT裁决，无修复/调参/第二评审。
作者后续实际完整H390suite22.17秒通过（独立log rgbd_and_cad_complete_full_suite.log），source_support_and_common_landmarks.png已作者查看；新增图/此390suite是作者证据，未要求评审重新审查。旧正文“379仅前阶段证据”按当时范围保留，不倒改。
