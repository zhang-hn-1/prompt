## 13:47UTC nominal support CPU comparison — final stage closure

先完整读 reports/benchmark_nominal_support_and_planner_consistency_20261005.md 与 docs/superpowers/{specs,plans}/2026-10-05-nominal-support*.md。一个独立封存CPU候选：只在planner使用DriveModel子类，将V1曲率裁到BankMap相应方向low/high共同数值域，不装进tracker，不改PID/speed/guard/.6/rig/五次停稳/35秒gate。forward范围[-.8272253275,.7815023661]，reverse[-.7978797555,.7672796845]，不是动态物理上限。历史/当前baseline导航失败、RGB/停止/M2拒绝保持。

11fixture callableRED7fail4pass.29s→GREEN11pass.25s，完整H227pass9.43s。两次不同cwd/relative source或行政写入错误在science之前，无运行结果；第一missingfile log封存前被正确RED覆盖，工具失败转录另存，不当原pytest证据。152sources/14187旧与实际inputs于13:57:06.100102UTC封存；明确一次sharedgridroute+两次HybridA*，同完整frame175/map17.4999996088s、当前pointgoal/XY/yaw/acceptedforward，execution175身份另存无未来物理输入。

唯一completed CPU comparison13:57:43.268009→13:57:51.059837exit0。原参考175 pose/command/direction/expanded2434精确复现；候选2482nodes。原10primitives/F1R1F8、path3m/objective4.075/localgoal.018737/两换向/7域外，候选12/F4R8、path3.6/objective4.7/localgoal.174290/一次换向/0域外。两臂raster samples均0occupied/outside，共同名义域candidate三次mapquery各曲段均no curvatureclip。原代价/路径只允许+.3m的冻结门槛均FAIL，所以CPU资格REJECT，不触发物理A/B，不放宽、不续扫。当前共同175PP+.622023→-.400175、两map均no curvature/speedclip，只是参考当前几何查询，非fullscheduledraw/protocol/smoother/applied或candidate实际未来。

定义来源postresult只读核验：activev9表与v2bank_mapping JSON逐字节相同，v2fit/measurement用Savitzky-Golay XY几何k*sign(main)定义body-named k，bank输入是请求参考，不能叫root Euler yaw/arc响应。当前V1消费者/静态payload明确，但历史2026-09-12生产者身份未证，当前probe同时有legacyyawratio与newXYgeometry；不借当前源码认证历史算法。生产者来源独立SHA保存，不重跑fit或标定，也不倒改旧source/report/gate。

作者13:59:48.329266UTC核152/14187及计数/gate/时序，assessmentSHA188a81f06e8796ae459c38856aed84c14776a3e55b51ec8642ba8acd3bc44aef，无science重跑；PNG已查看，无新视频，旧actual35s视频上下文。ONE fresh-context独立审查见nominal_support_review.json，评审正文和作者追加分开；作者收尾/hashcheck另署，不称二审。全部源码/REDGREEN/失败/完整现状/两名义路径/数据/图/身份保留，无无关Gitcommit/delete/upload；stage完成以nominal_support_final_check为准，整个研究NOTcomplete，automation ACTIVE。

下一项有界工作：追踪已经归档成功Oracle time_planning/hybrid与当前local direction_root length-cost的真实消费入口/继承/units/速度来源，针对同一个已知完整175快照做独立冻结的路线时间预算核验。距离/倒车代价不能代表实际35秒执行仅是待检验假设；本轮没有证明它是实际导航主因。先明确steady nominal与交接延迟/实际XY速度条件，不能把静态时间认证为动态期限。仅证据充分才试一个已有时间代价消费改动，保持原V1/mapper/PID/speed/map/rig/guard，不将已拒绝cap夹带安装、不同时更换曲率来源。不拿旧B0未来轨迹给新计划标签，不重复本轮支持域扫描；任何新物理候选另封存独立CPU资格后，同起态原35s actual进度/goal/RMS/peak/contact对照。RGB+estimatedpose/唯一学习记忆或世界模型/正式E0E1E2仍待分层准入，不报新导航成功率。
