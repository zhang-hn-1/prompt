# 唯一负反证记忆消费者：在线完整序列准入

旧pool和正支持来源候选仍REJECT。新的局部来源消费者以历史开发837core/34反证/0of205真占据被反证为设计证据，不把这条数据视作fresh。

唯一候选：0–14秒固定摇摆的全部29actual源窗口和原强双眼raw核心因果缓存；14秒统一汇总核心，对全部当时已知原NN双目pointPacket与估计pose，query pixel采样最近邻<=sqrt(2)、端点opticalZ比核心大于固定.20作MODEL反证。反证记录实际端点UID/左右帧/时刻/距离；只删MODEL假设，不能写已观测自由，遮挡后方或缺深度保持未知/假设。NN旧测距域拒绝不回改，也不将.20当统计置信界。冻结14秒池后不刷新，类别模型原2秒出生单记，hypothesis汇总约14秒。

因果/代码：NaN/future/framefraction/非刚体/不匹配pose时间先拒绝；历史缓存拷贝；actor不能读renderer/footprint、未来请求、真实动作标签。不调用旧policy.act/shadow直到14秒，不ghost tracking、不内部物理/smoother/reset，初始pose单次gauge/idealgyro+编码器及flatfloorprior原权限保留。相机、照明、planner、请求cap+.5/- .3、rollPID .4/.1/.7/supervisor .3、参考guard.6/planner .8707107固定。

对照：两条fresh whole sequences，同起点[-2.2,.15,.45]/goal[2.2,.15,.45]，相同14秒观测前缀；R0在14秒固定最后当前strong set，R1使用上述已冻结历史+负反证消费者。只更换来源记忆组件，planner/tracker其他环节全同。R0/R1是新条件任务局部对照，不擅称正式Oracle B0。检查实际物理/关节/smoother/源输出/请求前缀一致；NN/source不同则明确非严格配对，不用不相等seed声称唯一收益。

先TDD/历史preflight来源覆盖>=.95/面积<=6/class各贡献视图>=.95/冻结route+localkino，反证真占据核心数=0仅post-seal诊断，不能GT选点。后first scoped review/全CPU/源码+配置freeze，才能跑最多2条实际整序列；每条完整60秒总预算（包括14秒prefix），无contact/collision、真实goal<=.2、过障碍/轴进度、source95/6/pose RMS3D<=.05 peak<=.1 anglep90<=2、反证真实核心0、map无measuredfree、真实nominalguard及latency/video；分层独立报告。未通过无重跑/阈值/网络/窗口/PID扫描。14秒未停稳/unsafe起步则停收并拒绝，不延长等待、reset或触发循环。原35秒E0/E1/E2仍未触发；只有完整条件任务准入后考虑更大场景/学习增量，不能称原严格冷启动通过。
