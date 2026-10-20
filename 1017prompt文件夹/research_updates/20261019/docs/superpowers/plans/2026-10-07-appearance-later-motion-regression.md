# 接续：冻结15s外观参数在后来运动曝光上的历史回归

外观10样例全通过，视觉锥不能解释精确距离已拒绝，不再更改几何支持阈值/ROI/网格。继续验证同一已冻结外观候选，不换特征，不进行新训练：C1既有完整运行155..265步5（23曝光/46眼），全native当前RGB/CAD，模型参数只来自15s源150；每个新外观假设出生为其当前查询时刻，旧颜色模型出生15s，完全没有新depth或XYZ。后续query clock<=当前clock，不能读取未来payload，不改旧geometry出生/TTL。

先seal全部46当前mask与过去参数来源，再读当前离线GT；同前面95%precision/90%真实hazard coverage，>=100两类样本才exercise，无真实hazard为空则NOTEXERCISED而非高分。分别报告全部练习和未练习例；不忽略失败。旧轨迹已用于其他分析，全部只能历史回归，不是fresh whole sequence、在线接口或导航成功。失败明确拒绝当前外观消费，不能扩大色阈值救分。通过后下一实际工作是fresh whole-sequence曝光覆盖/只读在线外观观察与潜在障碍bearing接口，不把拒绝几何模型接入planner。

预标签分母明确：true-current-hazard>=100的曝光必须exercise；若输出不足100/为空则失败，不能因预测回避而归NOTEXERCISED。只有true<100可NOTEXERCISED，仍保留输出precision。此说明在任何本实验GT评分前追加；旧bytes 1192 SHA 7190998d1cfaedac9d812eaaa8ef57240437fa3bbe2f3e5d1c5b6809f1e934f1。
