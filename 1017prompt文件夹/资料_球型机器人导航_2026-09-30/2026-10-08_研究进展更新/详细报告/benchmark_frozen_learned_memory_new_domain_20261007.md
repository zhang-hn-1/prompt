# 冻结学习记忆在新任务中的适用性核验

结论：拒绝将旧 Mmem1 网络接入当前导航。当前估计位姿闭环已通过独立条件任务，但学习增量未通过。原训练、原 far/near 拒绝与正式 35 秒规则不变。

本次只运行已有冻结权重，不训练、不改阈值，不改变机器人动作。D3 整条序列此前已用于导航、位姿和来源诊断，属于历史开发数据，不是 fresh held-out。它只能检查是否值得安排独立在线验证，不能证明泛化。

## 实际结果

|指标|原始点历史基线|冻结 Mmem1|
|---|---:|---:|
|TP|521|645|
|FP|71|64|
|FN|717|593|
|TN|143|150|
|precision|0.880068|0.909732|
|recall|0.420840|0.521002|

共 1,452 个 cell-time eligible 行，不是 1,452 个独立空间格或独立序列。虽然误报、漏报数均下降，但预先冻结的 precision≥0.95、recall≥0.80 未达到；TP≥25 通过。不能用相对改善取代绝对准入。决策为 `REJECT_FROZEN_HEAD_IN_CURRENT_RAW_MEMORY_DOMAIN`，没有启动学习 shadow 或改变导航。

## 覆盖与错误层次

整条序列的真实障碍 XY footprint 有185格。首帧只有6个 occupied 格属于查询候选域，最后42.5秒只有13格；最后仍有172个真实 occupied 格不在 eligible 域内。网络只能对原始查询记忆生成的候选格分类，无法凭此恢复完整障碍范围。当前成功导航实际使用独立标注的静态视觉几何假设与缓存参考；这与 Mmem1 的 raw-query CellMemory 输入域不同，不能直接将网络输出替代当前模型栅格，更不能把预测写成已观测障碍或自由空间。

错误首先属于稀疏候选覆盖与可靠性准入层，而非本次 roll-PID 或到达判定。当前判别头不是生成式世界模型，没有动作条件的未来空间预测，本次不报告世界模型增量成功。不再换网络、训练更多 epoch、扫阈值来救这个候选。

## 因果与实际证据

特征构造读取保存的当时 RGB/CAD、双目预测查询、估计曝光位姿和过去实际请求；没有读取动态真实 root 轨迹或占据标签。完整特征与预测先封存，独立 scorer 随后才读取占据标签。研究者此前看过导航真值，因此明确限制为历史适用性检查。只读离线核验不是在线接口验证，也不能作另一个计划的真实轨迹标签。

冻结权重 SHA256：`7dbffaecc56147fc30cc648569f29eb2cb4c6da5aca204f43fc5fd5a816d55eb`。阈值0.5，21维特征、旧 mean/scale、35秒 raw-query TTL 保持不变。评分前再次核验5个计划/代码/权重身份一致；feature/predict 实际进程61147退出0，独立 scorer 退出0。本人核验这些实际产物，没有将他人评审当作实际 GPU 结果。

代码、完整逐帧 confusion/覆盖、封存与日志均在 `experiments/stereo_memory_navigation_20261004_v1/`：`build_frozen_learned_memory_D3_features_v1.py`、`predict_frozen_learned_memory_D3_v1.py`、`assess_frozen_learned_memory_D3_v1.py`，相应 features/predictions/assessment 文件夹及 `frozen_learned_memory_D3_pre_score_protocol_v1.json`。本报告只概括数据，不替换原始逐帧记录。

下一步保持当前传统显式记忆闭环冻结，验证一个预先规定的小初始条件变化；明确其仍依赖地面先验、理想陀螺和初始位姿标定。新任务通过不代表原始 E0/E1/E2、完整来源可靠性、实时性能或世界模型通过。
