# 唯一精确前视点消费候选：先执行目标、后固定参考对照

用户自主授权覆盖该局部控制研究，无阶段审批。移动射线候选拒绝；原source/pose/nav拒绝不变。125个历史R1请求链逐元素复现，109有效tracking中旧lookahead距离overshoot最高.299689m；精确参考目标差最高.433889m，仅证据，不先称bug或改善。

唯一候选：只在第一次正常reverse→forward被原方向handler接受后，将_outgoing_lookahead_target改为当时位置到当前方向连续线段的最近投影，加原adaptive lookahead距离并在同一现有参考内插值；不改path_index/参考/curvature公式/BankMap/增益/速度/命令范围/五次确认/反弹规则。反向与首次方向未切换之前原方法委托，有限同方向区间，不越cusp；不读取未来真实状态/请求。原whole-waypoint方法保留。

先CPU TDD：直线精确距离、圆弧折线内插、前后向与cusp截断、冻结路径/索引、非法state；再同一次历史factual链诊断当前边界反事实输出，不能把反事实后的旧实际轨迹作为标签。要求新protocol目标有实际差，若全部等价则拒绝不跑仿真。不存在“保证更稳定”假设。

候选若CPU准入，做唯一受控fixed-reference pair：使用同一封存frame140参考，明示历史已知参考/模型诊断，保留原固定14秒观察前缀、原rig/light/caps/PID，同初始条件；原参考终点作为局部目标，最长35秒含14前缀。当前真实独立到终点<=.2、完成当前方向段、相同参考独立进度/横向RMSpeak、contact/collision、有效请求和smoothed/applied、两臂物理+smoother+控制前缀事件之前一致；不以path_index跳转宣称完成。候选通过要求真终点<=.2、真正完成、无碰撞、横向peak<=.2、RMS<=baseline且不可通过拖慢进度获益，时间<=baseline或baseline未到达时同预算真实进度不低；pose<=原.05/.1/2单列。

先first scoped review/fullCPU/源身份封存后实际运行最多2条；历史参考fixed控制诊断通过不代表新RGB闭环或原35E0/E1/E2。只有合理局部通过才将同一改动在完整60秒任务另立对照，不重复原失败pair或扫描lookahead参数。失败则明确拒绝，归因执行层，禁止再次换controller/model/threshold救分。
