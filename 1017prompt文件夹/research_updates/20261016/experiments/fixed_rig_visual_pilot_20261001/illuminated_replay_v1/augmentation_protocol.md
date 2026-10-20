# 补光条件下唯一一次训练覆盖扩展（采集前冻结）

新增数据仅限 `hard_0_dev_manifest.json` 的 `hard_0_e1_0001` 和 `hard_0_e1_0002` 两条完整序列。使用冻结双目位置与光轴、`fixed_camera_fill_v1`、无视觉 `GoalGreedyPolicy` 和冻结 roll PID，原 35 s/真实接触规则。每 0.2 s 保存一帧左右原始 RGB 与离线渲染深度标签；真实深度和 pose 不进入网络。已有补光 golden E0/E1/E2 数据保持不变。

只训练一个扩展候选：复用 `pilot.py` 的像素标签、网络、随机种子、损失、batch、8 轮和两折评测。两折训练集分别为：golden E0+E2+两条 hard_0，测试完整 golden E1；golden E0+E1+两条 hard_0，测试完整 golden E2。黄金场景整体隔离；hard_0 场景曾用于项目历史诊断，因此测试不能称为 fresh held-out。

原门槛不变：两折 unknown 召回均 ≥90%，free→obstacle 均 ≤5%，E1 起点 0–2 s 中央真墙障碍召回 ≥80%。任一失败即拒绝，不调网络、不补扫参数、不接入在线动作。与无扩展候选对比只作归因线索，不在同一测试集上继续调参。
