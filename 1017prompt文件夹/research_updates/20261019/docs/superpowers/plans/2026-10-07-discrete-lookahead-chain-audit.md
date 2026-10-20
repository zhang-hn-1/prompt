# 实际请求链与离散lookahead几何：有界历史核验

动射线唯一265反证仍阻塞，拒绝/不跑新sourcecandidate。接续传统tracking层，不修改源模型/网络/阈值/PID。

从R1封存current actor输入140..264重构同版本完整policy，每帧只act一次；原NN/pose/seed/currentpacket时间不变，无未来、rendereractor或随机拆帧。先要求125次原请求逐元素精确一致，否则停在身份/输入错误。抽取当次actualtracking diagnostics、lookahead请求距离/累积离散距离/target、曲率/BankMap原角度与protocol裁剪、有效bank范围、path_index/方向/速度；此前数据是历史开发，不能叫fresh或在线。

唯一几何问题：原_outgoing_endpoint把固定距离lookahead取到下一whole waypoint，是否产生明显距离超调、目标跳变和曲率/指令差？使用同一参考/同一方向，只在当前几何上计算连续线段的精确距离目标，不调用第二有状态policy，不改变方向处理/当前索引。该几何反事实不宣称改善实际轨迹。优先原函数输入输出的自一致性，不能先假定bug。

通过要求：原请求全精确、输出有效性清楚、几何差可复现；如果角度裁剪导致有效动作等价，不能作为改进候选。仅有确切可改变执行且不破坏cusp的证据后，另立唯一lookahead消费候选/CPU几何测试/同版本局部与完整对照，不并改映射/增益/速度/规划/来源。
