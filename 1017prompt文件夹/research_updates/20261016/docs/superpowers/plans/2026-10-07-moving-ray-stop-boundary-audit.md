# 唯一停止边界的移动射线反证可行性

原R0/R1 frozen14 source/nav/pose拒绝、原60秒预算保持。只读历史R1 frame265已经实际取得的0..265每5帧NN pointPacket和当时估计pose，不能读266以后、未来动作或renderer作为actor。

唯一问题：把同一固定.20米反证规则继续应用于已封存14秒核心，会不会在26.5秒当时消除重建起点阻塞，同时保持来源覆盖>=.95、面积<=6、被反证真实占据核心0？只保留原核心，不追加新色彩核心/拟合模型。按当前及全部过去射线反证，再用原.20inflate、原PersistentCoreGrid、原.870710678 planner查询当前estimatedroot和固定goal一次；不运行controller.act、PID、物理或改变任一阈值。

actor产物及射线UIDproof先封存，之后GT评分。四项必须全通过才考虑单独动态记忆候选；结果只是历史可行性，不是fresh、导航、纯RGB、实时或完整3D安全。失败后拒绝，不换规则/扫margin/借助真值删除点。即使通过也保留R1姿态门槛失败，并需完整新序列独立验证。
