# 一个明确分离的学习预测层：稀疏历史深度条件补全

当前实际观察/双目源拒绝、圆柱补全可辨识性拒绝已关闭。不以新网络挽救旧Hero/Fast gate，也不把新产物写成双目观测。现有DA3Metric/UniDepth旧完整实验已经做过，不能重复。

新问题：在当前双目失去共同视野，但一只眼还能通过当前RGB确认旧3D锚点时，**旧可靠几何+当前单眼外观条件的学习补全**能否提供可验证的预测障碍层？这不同于双目匹配/低阶圆柱外推/纯时序MVS；输出全部标predicted，历史点保留birth/frame/time，不刷新成当前depth真值，不写observed_free。仍knownpose诊断，估计位姿/闭环/学习世界模型完整成绩未准入。

官方可用性核验2026-10-07：MetricAnything论文虽有Sparse Metric Prompt，官方prompt teacher checkpoint仍TBD/Coming Soon，不冒充可调用接口；student不可偷换成completion。DA3曾在本项目实测拒绝。选择ONE正式提供RGB+稀疏depth推理与公开权重的Any2Full，不启动PromptDA/其他模型比较或fit。

只用官方Any2Full commit780edf6be2bf9260f1cce8e4ddbcb52d7bcdae9e，vitl单一变体、官方init_scailing=True/stage1/no optional DAinit/no denoise/min1e-6/max1000/originalinternal518nearestprompt+RGBbicubic/strict state。固定seed20261007，FP32/eval，原生世界向上CCW90旋转RGB与prompt一致，再官方unresize+恢复native。官方init_scailing是模型规定的**当前输入prompt的逆深度仿射校准**，必须如实记录，不能说没有scale拟合，也不能读GT做校准。空prompt眼NOT_EXERCISED，不造零prompt“成功”。torch checkpoint先静态GLOBALS审计，weights_only=True允许tensor安全加载，禁止改False或执行任意pickle。模型/来源身份/前后tensor digest完整保存。

首个development仅复用已封存qualified_hazard_navigation_run_v1的235/245/255双眼RGB/currentpose/masks/appearanceproof。旧current_anchor_innovation已证明右眼有235/229/190可靠旧锚点、左眼0；这些数据已参与设计，绝非fresh。源构造复用固定NCC>=.8/std>=5/完整7x7所有bilinear tap无当前CAD/年龄<=10s/当前clock；输入限定currentRGB和由旧world/currentpose得到当前axialdepth。每个非零prompt需保存旧pointID/geometryclock/sourceframe/当前证明。相同像素重合取nearest positive Z的确定性覆盖，不能加入rendererdepth、过去不可见点、完整occupancy、未来姿态动作或GT标签。不在新网络前插值prompt/删GT坏点/拟合额外偏移。

先意义fixture RED→GREEN核验native旋转/axialdepth不变、稀疏pixel冲突、当前/历史clock与future payload保护、输入source身份/unknown、empty不触发假推理。然后6眼输入/状态先落盘封存，官方权重一次有界获取(2069029116bytes SHAea2809c9c490bdf37292f55e773fc5a935bcfc024cbd1eee6268b3ee3b744b41，900swall)、CPUAPI/完整state核验，首次新分支review，最多6眼请求（empty眼跳过，预期3实际GPU推理），预测seal BEFORE任何本候选GT评分。不做128/256/518分辨率扫描、weight/denoise/校准门槛扫描。传输/API行政失败保留独立字节、仅确凿接口修复可一次same-candidate恢复；不能以模型换型/后验bias救分。

冻结预测验收：全部非空眼不挑最好；current predicted .5..3m/native当前CAD/outside-root.55/body-height.05..1.5输出点，对当前独立GT端点三维误差<=.10m的比例每眼>=.95，不足入分母；至少100样本/眼。单眼预测不假造右correspondence。另全体当前真值可见近障碍域正确覆盖>=.90/p90<=.10m/100样本，不得通过把障碍预测成floor/远处躲分母。报告anchor7x7 halo内与外分别、halo外新20点/2格；只复制prompt不算补全。没有可信uncertainty头，不能把输出合法/相互一致当安全证书。任何失败即拒绝此预测层，不接controller、不跑formal、不再换completion网络或再训练救分。

若仅development通过：最多补2条完整fresh来源序列，在拟合前/预测前封存、相机/光照/控制不改，结果全量评分；不是邻帧随机拆分。通过后才做只读在线prediction layer，三类观测/历史/预测隔离与future替换、shadow on/off不改B0。然后最多一个“预测障碍只收紧风险路线”的消费候选，不能把预测free写成观测/保证安全；固定参考A/B通过再原35秒formal、RGB+估计位姿和学习记忆增量仍各层独立验收。

本项没有开启新的物理动作；整体任务未完成。已有perfect-visible-depth/startup-prior矩阵、全Oracle标定及所有拒绝不重做。

官方来源：[Any2Full代码](https://github.com/zhiyuandaily/Any2Full)、[论文](https://arxiv.org/abs/2603.05711)、[官方checkpoint目录](https://huggingface.co/zhiyuandaily/Any2Full/tree/main/checkpoints)。可用性区别来源：[MetricAnything官方发布表](https://github.com/metric-anything/metric-anything)。
