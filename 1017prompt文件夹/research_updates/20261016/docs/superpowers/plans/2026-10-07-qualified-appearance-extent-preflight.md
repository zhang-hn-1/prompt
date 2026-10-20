# 立即接续：合格双目点锚定的单眼表面范围假设（唯一低阶候选）

实测C1碰撞相关表面在0/60/120/145/150左眼可见，却与右眼没有共同可见表面；180有1603个共同可见点，原模型仅1点有效。故仅保留原seed/改交接无法补轮廓。继续一个CPU前置实验，而非再训练深度网络或改动作。

问题：在已取得准确seed150的时刻，可否用其颜色外观从当前和过去RGB中识别更完整的障碍表面范围？这只能给出appearance-based potential-hazard mask，是模型假设，不是双目深度、占用真值、自由空间或可执行控制。

冻结：只用C1确认源150及其左右像素（worldZ>.05<=1.5）、两眼RGB、当时CAD masks。两眼seed颜色合并；OpenCV RGB→Lab；中心逐通道median，尺度max(8,1.4826*MAD)；三维标准化距离<=3。阈值不看GT调。源不足100直接NOTEXERCISED。不换特征/阈值/分割网络，不进行分数驱动迭代。

在decision_clock=源150时，只查询过去完整曝光0/60/120/145/150左右两眼，共10例；所有clock先校验<=decision_clock，未来payload测试必须拒绝。每个mask只在CAD外，是出生于当前决策时刻的外观假设；旧RGB曝光不能回填成当时已有障碍观测。source源150被用于拟合，所有样例development而非fresh heldout。

先封存10个mask/参数/来源SHA与clock，再读独立renderer标签。全native480x640，不稀疏采样。类别真值：finite axial .5..3m、worldZ>.05<=1.5、距当前root>.55；预测mask所有CAD外像素计入precision分母，缺失GT算insufficient不能忽略。每个非空预测precision>=.95，当前可见hazard coverage>=.90；empty也是fail/notexercised，不能只挑source帧。只评价外观表面类别，不宣称.10m深度或体积精度；与旧深度拒绝不同层且不能改旧gate。

通过才进入来源可追溯的多视角范围/形状歧义表示与在线消费设计；不通过明确拒绝，不换网络/阈值重试。当前已知位姿仍特权，15s出生与budget仍局限。该小前置实验不能冒充导航、空间占用、纯RGB估计位姿或学习世界模型通过。原35正式E0E1E2尚未触发，整个任务尚未完成。
