# 动作条件双目短时视觉预测：一次拟合、完整fresh验证


## 2026-10-09T05:49:40.393053+00:00：唯一动作条件双目短时预测器已拒绝；真实控制消费试验直接接续

两个fresh偏移起点整序列均实际35秒正常关闭，分别350请求/351capture/1751physics/2101可用contact且0接触，双目/第三视角视频各实际解码351帧10fps。E0仍timeout，目标余3.641251515m、真实轴进度5.25874999981m；E1仍timeout，目标余6.033791707m、轴进度.570451804m。它们是预先冻结的预测验证采集任务，不冒称新正式E0/E1成绩，不把stage exit0称导航通过。

四条历史整序列仅拟合一次23维动作条件视觉运动ridge残差模型；两个fresh先完整rawseal，模型固定，两个整序列预测全部seal之后才算未来LKteacher。当前/过去图像flow、当前IMU/encoder/实测关节与已知请求，无未来feedback/applied/GTpose/GTdepth。teacher是压缩视频近似2D对应，只对inside且固定CAD排除的可用查询评分，缺失unknown，不是3D/自由/占据真值。

|fresh|眼|移动meanEPE px|持续旧光流baseline px|ratio|p90 px|判定|
|---|---|---:|---:|---:|---:|---|
|E0|左|1.996958|2.458731|.812190|3.831605|PASS|
|E0|右|1.968521|2.273417|.865886|3.818085|FAIL|
|E1|左|2.787094|2.897738|.961817|5.021830|FAIL|
|E1|右|3.936395|4.019301|.979373|7.641839|FAIL|

四组最低100移动查询/10移动帧覆盖均满足，但冻结每眼每序列ratio≤.85、p90≤5必须同时满足，因此唯一候选总体REJECT。E0局部收益不能泛化E1；没有refit/换网络/放宽门槛，不启动该模型在线shadow或动作消费。它仅是低阶下一帧视觉运动增量，不是完整导航世界模型/深度网络/新导航成功。独立review C0I0，所有原封存、特征shape(349,2,540,23)、模型身份、按原540批次重算预测逐值一致、四组指标独立复算；四CPU单测含未来图像/请求替换及日志截断前缀不变。作者实际查看误差图，视频均解码但未声称逐帧观看。产物 action_video_forecast_fit_validation_v1/assessment.json、closed_forecast_summary_v1.json、fresh_forecast_errors_v1.png、model.npz及全部inputs/predictions/teacher/seals保留。

继续做了有界历史因果诊断：risk age原v1因错找metrics.json在任何map分析前exit1，原失败文件保留；独立v2改用真实native_pose_assessment_v2 scored_end，E1 209/350地图含>现有10s raw TTL旧正格、peak36，E2 0/197。当前实际类RepeatedExposureMemoryGrid本就允许两个capture确认后静态持久化，此结果不证明TTLbug、错误占据、失败因果或自由空间，不能据此直接删格，更不能统一解释E2碰撞。独立review C0I0。

转向具体当前控制消费证据：first_tail_handoff_audit_v1实际核验E1第一参考，当前native position/旧参考/同帧地图复算guard逐值一致，不二次act、不用未来。15.5s endpoint距离.4696694029m、剩余arc.4696378868m，唯一reject reason为remaining_arc_under_lookahead，之前activeforward/no pending/tracking，随后rebuild第二参考并制动/倒车。这只证明重建条件，不预判bug。

唯一传统控制候选 endpoint_tail_retention_v1已预先冻结并实际启动session38736，stage endpoint_tail_e1_status_v1.json。仅第一版all-forward局部参考、当前forward/no pending/tracking、唯一短尾拒绝且endpoint>.2m时retain；.2复用原goal半径，hazard/projection拒绝不旁路，后续参考旧逻辑，原planner生成器、memory、mapping、PID、速度、动作范围、换向五采样与反弹清零、相机光照都不改。旧第一方向planner callback globals保持，源代码只替换原boundary一次guard调用，不二次act。CPU先RED后4GREEN含组合边界，独立用原frame155调用retain且reference version/state不变。独立审查metadata Important启动前修正并保留原preparedprotocol，最终C0I0。

单条实际E1/原35秒/350请求，接触或真实goal终止，timeout1200后45秒清理，不重试。比较同initial条件的native E1真实历史B0，不能假定render RGB完全相同或声称完整因果A/B，关闭后核验prefix物理/RGB；旧同流只读shadow未来不冒充替代轨迹。验收采用启动前冻结B0第一参考几何，真实endpoint≤.2、final轴进度至少增.5m、firstref横向RMS/peak不退化、0接触和有效请求改变；实际goal/35秒导航层另报。整体未完成，没有外部阻塞，不停止自动续做。精确入口：检查session38736/status，不重复两fresh采集/单次fit/历史矩阵；关闭后独立评估、记录接受/拒绝，不能调条件救分。


## 2026-10-09T06:07:23.321150+00:00：尾端消费候选实际拒绝；局部制动与续接几何已核验

单条 endpoint_tail_e1_run_v1 实际正常关闭（stage session38736 exit0）：原35秒350请求/351采集/1751物理/2101可用contact且0接触，双视频各实际解码351帧10fps。导航仍timeout，目标余6.259873528m，真实起终点轴进度.351923269m，相对实际历史native E1 B0减少.401677132m。第一参考106实际位置样本：横向RMS .049298415m/peak .090236544m，投影最大真实arc1.801338829m，最近参考端点.297272459m，未达到冻结.2m；B0为96样本、RMS .051091008m/peak .090260627m、arc1.629941808m、端点.467058191m。接触、横向误差、实际介入和请求改变检查通过，真实端点/最终进度增益两项失败，唯一候选REJECT，不重试、不放宽资格/.2/停止界。

介入15.5/16.0s确实改变参考消费。但实际B0和trial第一请求差frame89早于介入155，初始左右RGB不同403908/404134通道值；分叉前14非raw物理字段相同。因此历史对照是必要实测比较，不冒称同RGB完整因果A/B，也不能把整条最终退步唯一归因于尾端改动。评估器预关闭审查发现首参考最后执行区间漏终点capture，测试RED→GREEN后修正为每个实际version1区间的i/i+1并受评分cut约束；未改验收门槛。独立关闭review C0I0，原2212与评估后2215封存逐项核验。作者查看实际轨迹图，未声称逐帧观看全部视频。

继续实际执行只读停止链算术核验 audit_endpoint_tail_stop_rule_v1.py，exit0，不二次act、不重放policy、不读未来/GT反馈、不做新物理：当前native反馈在16.3s速度.224516791m/s，剩余waypoint长度.333547261m大于原V1停止距离+.1=.327201257m，仍tracking；16.4s速度.244040850m/s，剩余.316957847m小于停止界.361253573m，实际goal_braking/请求零，此时全局目标还距4.933576379m。原局部参考制动规则数值解释停止，并非已经到达全局目标；不证明停止距离物理保证或认定实现bug。165已安装参考版本2，明确不拿它反推版本1制动。独立review C0I0。

16.5s重建的hazard/raster检查实际均通过，唯一短尾拒绝仍在；尾端候选因tracker已goal_braking而不符合预先冻结资格，不是障碍屏幕触发。原风险age审计不支持删旧点/修改TTL来救这项结果。

继而执行唯一4次同地图名义几何查询 probe_tail_continuation_geometry_v1.py，exit0：当前native反馈/map在155/165两时刻，各比较原不限制首方向与既有minimum-two-forward几何诊断，map/target/.8707106781m margin/四点模型不变。155原几何reverse起步成功338nodes；限制两forward成功740nodes但之后仍reverse，且155实际retain，诊断新计划不是当时实际执行。165原计划reverse起步成功340nodes，与实际计划全部waypoints逐值复现；限制两forward不可达4nodes。此结果仅名义可行性，不准入强制前行/新动作候选，不放宽empirical域或margin。独立review C0I0。

具体尚未解决：局部有限参考的安全终点制动与下一段可执行续接相互制约；较早可行的两forward也仍需要后续reverse，不能通过延迟重规划等同解决。传统双目已知/估计位姿各0/3到达，估计E2真实碰撞，唯一低阶双目短时预测器拒绝保持；整体研究未完成，没有外部用户阻塞，不声称新的世界模型或导航成功。

接续应以已封存的同版本在线日志构建“下一段续接可行性事件”的只读接口：明确何时当前参考仍可跟踪、同方向续接何时丢失、制动请求与下一参考方向何时分叉，输出当时可用输入，禁止读取未来标签或实际反馈动作制定未来计划。只预先定义一个事件/一个候选，若没有可执行物理证据则先拒绝，不能继续尾端阈值/Oracle/网络矩阵。当前所有仿真/fit进程已关闭，不重启已完成矩阵、fresh采集或唯一fit；下一实际入口是读取 tail_continuation_geometry_probe_v1/assessment.json 和 endpoint_tail_stop_rule_audit_v1/assessment.json，建立事件接口及当前输入隔离测试，冻结单项闭环实验之前不改actor。


## 2026-10-09T06:16:01.509500+00:00：只读局部参考停止事件接口与真实在线单条运行已直接接续

未停在尾端拒绝报告。local_endpoint_event_v1离线接口及3项CPU测试完成；独立审查发现时间校验晚于payload读取Important，用trap测试先RED后独立local_endpoint_event_v2先校验时钟，旧v1及旧v2因果证据按字节保留；新增独立live adapter，合计5项CPU测试PASS。verify_local_endpoint_event_causal_v3实际exit0：155–164未来native feedback/当前之后policy rows替换或截断输出逐值不变，原request bytes unchanged。读取的是current native＋上次完成tracker cursor/reference，不重做projection；零请求仅当时构造的停止假设，不声称当次真实after-act请求预测，离线证据不称online接口通过。旧源码/测试原字节身份恢复核验，新版本另存，无倒改原结果。

经独立prelaunch Critical0 Important0，2105源码/92输入哈希全验，Minor旧stage2/旧时间身份启动前修为当前单条，已实际启动 run_local_endpoint_live_e1_stage_v1.py，session66787，status local_endpoint_live_e1_status_v1.json。每实际循环在唯一policy.act之前读取current snapshot并立刻写local_endpoint_live_before_act.ndjson；结果不参与request选择，每循环先清空避免陈旧字段。原native-SGBM B0感知、memory、planner、rollPID/gains、速度/动作、换向规则、rig/light、TaskH先验不变，不使用已拒绝tail候选或短时视觉模型。

仅1条原35秒/350请求；真实contact/goal/35秒原终止规则，wall timeout1200+45清理、无重试。此项只验证真正在线的事件接口；轻量shadow on/off测试只证明调用/不可变性，不冒充同RGB同physics全程A/B。实际导航结果另报，online接口通过不代表模型/控制/正式导航通过。当前真实运行进行中，整个研究未完成。接续：先检查session66787/status，若仍运行不重复启动；正常关闭后执行 assess_local_endpoint_live_closed_v1.py、实际验证封存/clock/raw/视频，记录接口与导航分别结果，然后按证据冻结单项参考续接研究，不回到网络或尾端阈值矩阵。
