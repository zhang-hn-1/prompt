# 唯一控制候选：来源事件替代固定观察等待

证据：Any2Full扩展已拒绝；较早原B0源145/150局部99.94–100%正确、8次严格双眼历史留存≥97.26%通过。这不证明完整覆盖/安全或提前轨迹标签。本项只验证来源事件的控制消费，避免继续换网络/调PID/阈值。

同一新runner生成B0和C1各一条真实35s完整2.6m E1局部诊断，和旧qualified_hazard_navigation_v1的manifest/scene/start/goal/协议/rollPID/mask/light/Fast/source处理/planner相同。B0保留18–24资格和ObservationHandoff。C1的唯一功能改动为交接就绪接口：连续两次相邻2Hz(相邻5个10Hz采样)当前hazard≥100、XY格>=2、连续5个10Hz实测XY速度<.05且当前clock；确认事件替代固定t>=18，原有单源记忆在确认当前packet出生，不合并源/刷新时间。12s前完全相同观察请求，12s后等待零请求，反弹清零与五次确认不变；事件后同一nominal policy动作。资格采集与动作交接由同一个事件决定，避免动作启动而仍无合格源的矛盾。不是两个独立策略改变。

不改planner/参考/映射/gain/unknown main cap .3/动作范围/10sTTL/rig/light/collision physical_contact_v2。已知位姿仍特权；flat-ground/startprior始终不是观测，未知探索风险保留，不称安全保证。禁止未来GT动作、索引跳转、强迫提前反转、阶段内部reset。source-only gate不读取GT；GT在控制源固定后只落盘，评分在episode/predictions seal后。

先 meaningful RED→GREEN：原12s前请求、事件前零请求、原五次低速/速度反弹清零、事件只启一次、nominal10D数组不修改、未来事件clock拒绝、B0 identity/config一致。核验两variant唯一区别位于就绪接口/source birth时刻；首次新分支skillreview后修C/I再actual。各900swall、每臂一次，不换阈值、网络或重新跑若干最好seed；行政接口失败仅确凿最小修复保留旧字节。

真实验收独立path_index：完整实际root轨迹、goal距离、contact/collision、progress与source-point clearance(必要条件非安全证书)、source精度/出生clock、handoff事件及applied动作。严格局部候选接受需C1实际到达原2.6m诊断目标且计分内无接触/碰撞、历史可靠源端点.10/95%门槛、已观测hazard≥.6m必要clearance无违例。B0同条件完整运行；分叉前对比请求/物理/joints/smoother/policy/source/曝光身份，RGB或源不同则限制单因素因果范围，不凭名称声称严格paired。停车、index跳转、延后collision均失败。有效执行与改善必要，但短诊断通过不等于原6.6m/8.8m正式导航。

两臂后无论结果，四层独立报告：局部source/接口/控制/正式未触发。失败只一次因果核验，不能继续扫交接阈值救分。正式原35s E0/E1/E2只在实际固定任务与身份移植准入后，且纯RGB估计位姿仍另层。整体未完成，原所有REJECT不回写。
