# 当前关节机体mask接入实际RGB局部导航

局部任务通过：同起点、同2m目标、原35秒预算，在22.6秒进入目标，终距.193028772m，实际接触0。仅源点筛选改用已局部核验的动态CAD mask（与原base mask取并集），网络RGB/深度输出、当前源记忆策略、planner、roll-PID、.3未知速度cap、相机、光照、参考交接均不改。已执行一条候选，未重试或扫阈值。整体研究未完成。

## 控制身份与可归因范围
PriorRGBPolicy(memory_mode=current)，同继承/act函数与原current基线，派生runner保存逐项逆替换后与旧源码字节相同的校验。新mask仅由当前公共17D proprio最后三关节9:12和已知机器人CAD计算。网络仍读取完整左右RGB，在旧mask下产生冻结Fast输出/原LR置信度。候选随后只删除被新增左查询mask或对应右伙伴mask覆盖的端点；不新造点、不修改预测值、不读取真深度或环境图。每5capture更新一次，缓存源沿用产生时刻mask、关节、相机位姿和source/query时间，不以之后状态补算旧mask。

knownpose仍是明确特权层；flatground/safe-start是独立任务先验，不写观测自由。未知探索是有显式风险的仿真任务，不宣称安全认证或纯RGB+估计位姿成功。原完整E1/严格source gates保留未通过。

## 同实时原始源只读对照
226次旧base-mask current consumer与执行候选共享同一次网络输出/图像/时刻/实测位姿，只是sourcepacket筛选不同。独立策略与地图对象、拷贝输入；runtime逐次assert主策略返回动作、guardedtracker边界、物理clock、完整世界位姿/线角速度/关节/关节速度不被shadow改变。CPU on/off fixture通过；未单独执行完整物理shadow-off对照，所以不把该断言夸大为全面运行隔离证明。旧mask影子请求始终未执行，没有它的真实轨迹、终距、碰撞或成功标签。

同流45次原始请求不同，首次14.5s/frame145：新mask hazard4格、retain、请求+.3/.341；旧mask hazard5格，原continuous_observed_hazard_clearance拒绝导致rebuild，零请求。证明同源过滤确实影响局部规划/请求消费；后续shadow路径是反事实计划，不是实际轨迹。

与之前实际current基线相比：整条1131物理样本的17个非raw字段、227执行状态和226请求完全一致，终距/时间/路径同；但46推理帧92眼RGB没有逐字相同、最大MAE2.4615，预测0/46逐数组相同。因此实际任务没有提升，不能把候选成功写成相对旧执行基线的新导航改善，也不能把影子零请求推导成它真的停住或失败。相同物理结果与同流影子请求差异是两个不同层次。

## 来源收益与代价
同流旧源101261个端点，候选97980，删除3281（每个都经左右新增mask因果校验）。离线实测真深度诊断：删除2084个≤.1m准确端点、863个>.1m错误端点、334个非有限真值端点。新保留89297个≤.1m/97980；旧91381/101261。不能说删除点全是机体或全错误，膨胀边缘/右伙伴遮挡会损失真实场景支持。以机器人CAD拒绝不可信对应是合法自身排除，但不能靠删点宣称完整来源/地面覆盖通过。原90/99/.1门槛没有追改或重新准入。

## 实际运行与核验
UTC20:17:13.487→20:21:06.592（2026-10-06，上海次日），退出0。226act/227capture/1131physics/46双目网络调用；两完整视频各227帧10Hz解码，0曝光时差、0接触。实际路径2.332308m，目标方向进度1.828553m，横向RMS.039155/peak.104423（同一冻结直线几何评价，不依赖path_index）。这里只是进入.2m目标门限，不是终点稳定保持。

四fixture先importRED后GREEN4pass3.28s；完整379suite19.09s。源/配置1255项预封存，关闭后原始产物与源共2984SHA核对；作者独立重算46包/当前mask1012字段、226地图1356数组、旧影子2712字段一致，未来payload trap拒绝。含CAD实测geometry证据属于上一独立报告，不在本次重新标定。

报告数据：selfmask_rgb_navigation_assessment.json、selfmask_rgb_current_baseline_comparison.json；原始selfmask_rgb_current_run；图selfmask_rgb_navigation_actual_and_shadow.png（作者查看）；视频stereo_view.mp4/third_person.mp4。运行日志中的隐藏BenchmarkMarkers材质警告没有改变marker可见性或时钟，所有采样可见性assert通过。离线真值缺失的matmul警告保留，非有限真值不算准确/错误，但不删除原源记录。

## 准入与下一项
局部目标通过；来源过滤因果接口通过；导航客观改善未证明；完整目标/估计位姿/学习记忆/正式E0E1E2未通过。保留该有当前传感器支撑的机体排除作为独立源版本，不将其包装为可用全域深度模型。

下一项在这个源版本上只验证10s历史来源记忆相对current的消费，ONE相同2m/35s实际history序列，另有同实时masked来源的current只读shadow。唯一改变memory_mode，网络/源mask/控制/PID/速度/planner不变，旧history失败保持。要求真实目标/contact/runtime门槛，且与当前实测22.6s/.193结果无退化并记录真实执行请求差异；若失败拒绝该记忆增量，不继续TTL或网络扫描。同流shadow不提供替代真实未来标签。该工作完成后根据证据接续完整目标或预算/路径消费瓶颈，不能因写报告停下。
