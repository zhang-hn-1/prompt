# 历史深度条件的学习几何预测：来源、接口与待运行状态

本文件是阶段记录，**整体导航研究未完成**。截至2026-10-07 06:48 UTC，尚未加载官方权重、尚未进行本候选GPU推理，也没有本候选预测准确率。正式35秒E0/E1/E2未触发。

## 本项实际解决什么

之前实际相机转向已达到10°，但近距离两眼重叠减少，双目深度仍不满足准入；独立圆柱外推也因空间块敏感性被拒绝。本项仅尝试一个学习预测层：当前RGB与过去可靠3D锚点共同条件下补出当前深度。它是预训练的空间补全先验，不是已经完成的动作世界模型、视觉定位或闭环导航。不重写旧结果，不重新运行已拒绝的DA3、UniDepth、Hero/Fast或圆柱模型。

输出必须全部标为predicted。过去的几何birth时间、点ID和来源帧保留；当前外观匹配不能把旧深度变成新观测，也不能将预测自由空间当作observed_free或安全证书。当前位姿是仿真已知位姿的特权诊断，估计位姿阶段仍未通过。

## 唯一候选与官方接口

选用[官方Any2Full](https://github.com/zhiyuandaily/Any2Full)、[论文](https://arxiv.org/abs/2603.05711)和[公开权重](https://huggingface.co/zhiyuandaily/Any2Full/tree/main/checkpoints)。固定代码commit `780edf6be2bf9260f1cce8e4ddbcb52d7bcdae9e`；唯一ViT-L变体、stage1、FP32/eval、无DA额外初始化、无denoise、无训练、无分辨率/阈值扫描。权重repo revision `5c8271e2e38b55abc2fc2292c9d390ed3b443507`，2069029116bytes，SHA256 `ea2809c9c490bdf37292f55e773fc5a935bcfc024cbd1eee6268b3ee3b744b41`。

官方流程含**当前输入稀疏prompt的逆深度仿射校准**，不能宣称完全无拟合。该校准不读取GT；预测适配器拟记录实际校准前后值，不增加第二次偏移修正。原生640×480图像与depth一同CCW90，官方内部变成700×518，再恢复原分辨率与native方向。轴向depth单位不改变。checkpoint先静态GLOBAL审计，只允许 `weights_only=True`；不执行官方CLI中的不安全pickle加载路径。完整state严格一致、推理前后tensor摘要尚待实测。

## 已封存的真实开发输入

仅复用已参与历史分析的235/245/255帧，**不是fresh held-out**。6眼输入先落盘封存，当前RGB、过去world点/ID、当前相机矩阵、mask、birth/sourceframe均有SHA与来源记录，未加入renderer depth或完整occupancy。

| 帧/右眼 | 旧深度prompt | 原图比例 | birth年龄 | 当前轴向深度范围 | 旧XY格数 |
|---|---:|---:|---:|---:|---:|
|235|235|0.0765%|5.5s|1.1486–1.2578m|5|
|245|229|0.0745%|6.5s|1.0327–1.1439m|5|
|255|190|0.0618%|7.5s|0.9056–1.0196m|4|

三条左眼输入均0prompt，明确NOT_EXERCISED，不造零深度假推理。锚点主要沿障碍边缘，输入深度跨度有限；合法输入不等于全图外推可靠，也没有校准不确定度。

实际输入图：
![当前RGB与旧几何锚点](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/any2full_current_rgb_old_prompt_inputs_v1.png)

作者亲自查看此图；红点只表示旧几何与当前外观证据，不表示新的双目深度。

## 冻结评估与首次审查

先封存实际预测，评分器才能读取独立当前GT。全部非空眼共同验收，不挑最好眼：完整native像素域中，预测0.5–3m、当前CAD排除、距root XY>.55m、world高度.05–1.5m，至少100点；三维端点误差≤.10m的比例≥.95，缺失真值计入分母。另以真实当前可见近障碍域定义独立分母，覆盖≥.90、p90≤.10m、至少100点。7×7 prompt halo内外分别报告；halo外至少20正确点/2个XY格，复制prompt不算补全。任何失败拒绝预测层，不接controller、不换补全网络救分。

首次独立skill review C0/I1/M0：发现旧stride2仅评估25%像素，75%错误可逃过覆盖门槛。作者增加反例先RED，再改为完整480×640网格，5个metric测试GREEN；旧源码字节与protocol v1保留，effective v2封存修复。修复发生在模型推理/当前候选GT评分前，门槛没有放宽。

审查者亲自核验10focused测试、36protocol/8input/16source SHA、6眼输入精确重建及source180点ID/birth、官方CPU resize/仿射fixture；没有读取checkpoint/GPU/仿真。作者全回归v2为533PASS/24.84s。

后续作者的截断历史负例发现旧helper对零条历史点调用OpenCV空remap而报错。旧helper和已封存输入保持不动，独立v2仅增加empty返回，先RED再GREEN。用v2对6眼实测输入重建精确一致、24future-clock拒绝与6历史截断返回unknown通过。没有重新生成输入或改变候选动作/分数；这不是在线shadow或导航验证。

## 实际运行证据与未完成项

所有路径在 `experiments/stereo_memory_navigation_20261004_v1`：输入 `any2full_memory_inputs_v1`；官方CPU API记录 `any2full_predicted_geometry_preflight_v1/official_geometry_api_cpu_preflight_v1.json`；effective protocol `any2full_memory_protocol_v2.json`；first-review closure `any2full_memory_author_review_closed_v1.json`；源守卫 `any2full_sealed_input_causality_v2.json`；原始RED/GREEN/full-suite日志均保留。

原始官方公共权重下载实际收到652214272bytes后发生IncompleteRead，820.00s，旧partial/日志保留。唯一同artifact官方优化传输恢复于06:39:24UTC开始，foreground session85006/hostpid1324239，900s上限，尚在运行。没有因传输失败换模型/改权重，也不能把下载失败写成模型预测拒绝。

准确率、实际GPU latency/内存、完整state核验、预测接受/拒绝均**待执行**；新的controller/fresh采集/formal导航均未准入。下一入口为权重恢复状态与 `run_any2full_memory_prediction_stage_v1.py`，仅在完整权重SHA验证后运行三条非空右眼推理；预测seal后运行 `assess_any2full_memory_geometry_v1.py` 与实际图表。失败保留命名空间，不自动反复运行。


## 2026-10-07T06:56:38.050385+00:00 Same-artifact weight verified; one safe-loader recovery ACTUAL STARTED
Officialtransport recoveryclosed835.568s exact2069029116bytes/SHA. FirstpredictionV1 administrativeweights_only loadrejected NumPy scalar beforeNN,5.324s/0 learnedcalls/oldnamespace/logpreserved. StaticALL6GLOBALS exact; best_rmse np.float64/f8 proven; CPU explicitnp.scalar/dtype/Float64DType allowlist withweights_only=True PASS493state1.583GB; unsafeFalseNEVERused. ONEsafe-loaderrecovery sameweight/input/currentpromptaffine/gate, independentV2namespace/protocolV3/scope preservedfirstreview. Actualdriver STARTED foregroundsession49303 at06:55:51UTCpid1342828/max600,expected3NNrightcalls/emptyleftskip; checkbeforeduplicate. No fresh/controller/formal. Newemptyhistoryfull534PASS24.97s. Next assessorV2 AFTER predictionseal, actualplotV2/all-eyes/weightdigest/evidence/report. WholeNOTcomplete.
OldprefixSHA 3d4039921ce6af87f731e012bc6254f98295d0f5f40398d5a38742de12bed4d7; oldprefixbytes 6075


## 2026-10-07T07:03:37.041965+00:00 实际预测与评分结束：拒绝预测层

三条右眼实际forward .63234/.45742/.45592s；整个恢复推理driver17.2983s，峰值GPU2.227808768GB(十进制)，493完整state逐值相等；前后tensor SHA均fc697368f9a57db3072809d23f08741dad70d9a5ffe74e58beae83ef8cf4fa97。三条左眼EMPTY没有forward。预测seal先于评分，所有权重/输入/protocol源码SHA一致，无GT输入、无新训练、无新物理运行。

|右眼帧|预测eligible|正确/不足/有限错误|完整precision|真实近障碍coverage|真实近障碍p90|halo内precision|halo外precision|
|---|---:|---|---:|---:|---:|---:|---:|
|235|159449|59704/60958/38787|37.44%|60.39%|.16585m|95.32%|36.16%|
|245|159463|87932/44956/26575|55.14%|76.43%|.11666m|96.64%|54.15%|
|255|160934|101449/27622/31863|63.04%|76.10%|.11345m|96.75%|62.25%|

全部三眼失败，不选最好眼。halo外新增正确点56403/84332/97876、14/15/11个XY格，只能说明产生了更多正确点，不能取消大量错误。剔除不足真值的诊断precision仍仅60.62/76.79/76.10%，且独立真近障碍coverage/p90均失败，所以拒绝不只是缺失GT造成。实际prompt残差p90 .00302/.00309/.00259m；官方当前prompt校准局部有效不代表外推通过。diagnostic仿射系数仅复算实际记录输出，不对GT拟合或修改预测。

实际完整图作者亲自查看；第三列为预测而非观测，第四列0–.20m颜色截断的全图GT端点误差，图含CAD供直观对照，计分已排除CAD。
![实际三条预测与独立误差](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/any2full_predicted_geometry_actual_panels_v2.png)

最终REJECT_PREDICTED_GEOMETRY_LAYER。没有进入fresh补采、在线shadow、controller或正式35秒评测；本候选失败后不换补全网络/denoise/训练/阈值救分。整体任务尚未完成。实际结论是历史锚点附近准确、全图预测扩展不可靠；下一项工作回到可靠观测在运动中丢失的时间与较早采集窗口，不继续补全模型调参。作者最新534PASS24.97s、24futureguard/6truncation与实际GPU/标签评分分别记录；reviewer只核验先前CPU/源接口，未亲自运行GPU。

OldprefixSHA 5dc746a12786ba3bdc4d9f253332434c9d3dfe7c14975e2cb99027a23dde5c64; oldprefixbytes 7084
