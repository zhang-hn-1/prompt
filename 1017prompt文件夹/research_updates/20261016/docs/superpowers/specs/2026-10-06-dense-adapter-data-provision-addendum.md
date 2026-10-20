# 训练数据前置资格拒绝后的单次补正，非第二候选

V1数据清单封存但没有产生loss/gradient/optimizer/update：加载时assert labelcount>100在E1第168帧42有效像素退出。逐个OLD输入核验32中6空(E1 196双眼、hard20/23双眼)，原32固定输入不能满足非空监督；另4眼仅39..71标签。完整记录保持，V1预拟合数据资格REJECT，不倒改32-input设计或宣称训练完成。

本补充在任何fresh模型成绩/标签读取之前做一次数据供应补正。模型结构/学习率/300attempt/8iters/max416/所有sourcegate不变，300步预算尚未使用；只拒绝原32输入中的6个空监督，26非空来源按原序round-robin。低样本眼保留，不从困难中删坏预测；不加新frames、不重复fresh采集/网络/层/超参搜索。真实双目partner/mask/visibility决定trainlabel非空，不是模型成绩或heldout筛选。误加的>100是训练代码额外断言，改为design非空要求，不改变正式评估每帧>=100的门槛。原旧V1状态running是try覆盖不到preparation failure的行政缺陷，另terminalaudit明确失败；不覆写旧字节。

候选最多ONE实际300 optimizer updates/ONE finalweight，没有任何先前学习权重，不能宣称V1物理/网络性能失败。新版source/26清单/全部old标签和仍未解码freshwhole32s的身份重新预封存，原freshseal保持；各新文件独立v2命名。新fit一旦真正梯度/优化开始后失败即拒绝，不再补正/第二fit。此为有界自主管理裁决，代价是32-input原设计未兑现、模型域仅有限有观察的fit样本；完整fresh评估分母全部保留，不能据训练loss宣布来源通过。
