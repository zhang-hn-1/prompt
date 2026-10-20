# 持久假设可见性接口：在线验证与参考消费诊断

## 结论

实现并实际运行了只读PersistentHypothesisContext。当前看不到、CAD遮挡、黑区、非黑投影和旧静态假设仍在保留被明确分开；没有把投影非黑当成真实表面或已测自由。四秒在线验证通过同调用不变核验，动作与旧同初态前缀相同。本轮没有控制改动、来源新成绩、世界模型训练或正式35秒E0/E1/E2。

旧B0本来已保留60秒MODEL假设，未发现“深度为空自动删障碍”的实现错误。不凭接口完成强造控制改善。持续点替换只覆盖43%–45%的拒绝、新来源逐时刻89.85%拒绝及其他历史结论保留；整个研究未完成。

## 冻结语义与测试

假设身份由出生包SHA标识，是来源身份，不是已认证的物理对象身份。出生voxels独立复制，60秒寿命从原出生计算，不因当前深度缺失、投影非黑或新接口调用刷新。

当前原生640×480 RGB/CAD与估计相机USD矩阵把假设点投影到两眼，分类outside_projectable_frustum / CAD_unknown / black_RGB_unknown / nonblack_projection_only。最后一项仅必要图像支持，不能证明点的深度、可见无遮挡、同一表面或占据；前三项均不证明物理不存在。过期仍与缺观测分开，无动作API、无几何/source/free资格。

五测试RED缺实现→GREEN，独立review五测试通过0.32秒，Critical0/Important0。唯一Minor为stage封存角色旧称SOURCE-only，启动前用独立metadata说明改为online context接口，无行为改动。依赖SHA、三个AST、当前clock先于payload、rigid pose、输入/快照复制和外层finally覆盖均核验。

## 实际在线四秒

session81393，父3976998、子3976999；2026-10-08 22:55:24–22:56:35UTC，退出0，墙钟70.44秒。一条完整reset序列，无重试/拟合/延长预算，固定相机/光照/PID/网络。40零请求、41capture、201physics、9NN更新；第20帧当时生成的非空出生包含501voxels，此后20/25/30/35/40五次LIVE调用。

接口读取本次当前/过去RGB形成的出生包及当前估计位姿/RGB/CAD，在教师depth读取之前调用。未加载旧D3出生包作为在线输入。每次原出生包、clock与完整物理snapshot均保持；241可用接触检查无接触。初始GT规约、理想bodygyro、地面高度先验仍为明确特权，未认证一般估计位姿。

与旧A2同+.15初态、同四秒零请求的context-off前缀比较：40请求完全相同；41状态的time/position/quaternion/velocity/angular_velocity/joints/joint_velocity七字段完全相同；201样本19物理字段中17完全相同。两个raw字段仅初始化原点差−0.02秒/−1物理步，整段恒定，未写成全19字段逐字一致。

两段视频实际解码41帧、10fps，作者查看2秒双目截图，未声称完整观看。四秒均为零请求，实际在线验证仅覆盖启动/当前投影接口，**没有在线移动到视野外的闭环验证**，也不是新导航成功。

![本次2秒双目视角](../experiments/stereo_memory_navigation_20261004_v1/persistent_context_C1_run_v1/stereo_view_frame020.png)

## 输入变动与时延限制

物理状态相同不等于RGB逐字相同。C1/A2第0帧左右RGB平均通道绝对差1.0028/0.8411；2秒为0.2224/0.2576，4秒0.2253/0.2607。第0帧已经发生差异，此时新context尚未创建；两次出生hypothesis分别501/477voxels，不能声称原NN/假设包全同。没有隔离渲染时序、抗锯齿或网络噪声原因，不改相机/光照/网络救分。

九次NN采集source总处理墙钟中位2.707秒、最大4.202秒，包括其他管线工作；没有单独记录context在线耗时，不把这些数称纯网络或纯context耗时。四秒模拟完成耗时70.44秒，不代表10Hz实时部署性能通过。正式35秒规则仍是原模拟计分时间。

## 历史移动轨迹：哪些状态真的不同

在旧D3五个冻结时刻，仅用不可刷新出生526voxels、当时估计位姿/RGB/CAD做封存回放，没有GT深度/实例/未来反馈或再次policy调用。2秒左右非黑投影为525/526（左另1CAD）；10秒496/503，其余为CAD/黑未知；20、30、40秒两眼526点全部在projectable frustum外，但旧60秒hypothesis仍active。

这是历史离线结果，不是本次四秒在线视野丢失验证。outside frustum可能包含侧方、视野角度或后方，不据此一概称机器人已通过障碍。它与上一轮教师足迹附近像素在这些时刻为零相符；非黑投影本身仍不是教师精度成绩。

## 同轮继续：当前参考是否仍需旧假设

接口后继续核验同五时刻已安装参考与MODEL支持的几何关系，无planner重调用、无改变动作。只使用当前及已安装参考，通过原project_remaining求最近几何投影；不是认证时间进度或控制A/B。

|时间|当前root→固定目标弦距MODEL最近距离|剩余安装参考距MODEL最近距离|
|---|---:|---:|
|2秒|0.04448米|无当前计划|
|10秒|0.04453米|0.89956米|
|20秒|0.00401米|0.90035米|
|30秒|0.52826米|0.84478米|
|40秒|0.97233米|0.95128米|

固定目标弦仅诊断，不是planner去掉记忆后实际会执行的路线。20秒hypothesis全部视野外时，直连目标的弦仍穿过其支持，而安装参考绕开；说明“视野外因此删假设”缺乏合理依据，不能说每次看不到就已无关。30秒剩余参考最近距低于名义0.87071米；最近投影不等于真实沿参考控制误差或安全证书，不据此独立宣告碰撞。旧D3真实接触0/42.9秒到达保持。

初版假定2秒已有frame020局部计划，文件缺失退出1；失败记录保留，独立v2标明无计划，不用未来6秒计划回填。四个后来文件保存time_s为policy.last_plan；目前未核验它是否等于真正参考几何生成时间，不能只凭该字段把旧整条几何称新生成计划。下一项需分开参考SHA/版本、首次生成、当前日志刷新与实际剩余段，然后审查30秒距离的几何/控制含义；不能预认定是tracking bug或调PID。

## 分层状态和接续

只读当前可见性接口已实际在线验证；视野外状态仅历史回放；参考影响仅几何必要诊断。原来源和唯一持续点替换仍拒绝，暂无有证据支持的控制候选，不启动正式导航。接续明确当前安装参考身份/生成与刷新语义，核验是否实际消费60秒MODEL与当前/历史观测，定位其可能需要干预的环节。不能清除视野外假设、放大支持、改TTL或把接口工作包装为导航改善，也不重跑已成功D3完整实验。

## 证据入口

H=`experiments/stereo_memory_navigation_20261004_v1`：persistent_hypothesis_context_protocol_v1.json、persistent_context_RED/GREEN_v1.log、persistent_context_online_author_review_v1.json；persistent_context_D3_historical_v1/assessment.json、reference_MODEL_relationship_v2.json；persistent_context_online_status_v1.json、persistent_context_C1_run_v1/artifact_manifest_post_close.json、online_context_interface_verification_v1.json、baseline_raw_clock_scope_v1.json、input_variability_and_latency_scope_v1.json、双目/第三人称视频。无本轮Git推送，无外部用户阻塞，研究仍在接续。


## 同轮继续：参考身份与距离口径实际核验

作者实际CPU检查全部369份原local_plans：几何版本1始于6秒，保留至22.4秒；版本2始于22.5秒，保留至33.4秒；版本3始于33.5秒，保留至40.9秒，41/41.5/42/42.5秒再生成版本4–7。10/20/30/40秒文件time_s是检查更新时间，不是其路径生成时间。runner、tracking_tube_planner、guarded_reference和typed_hypothesis_grid四份当前源码均与D3当时身份SHA一致。没有第二次调用policy。

30秒旧guard日志连续剩余路径距离为0.893387米，之前诊断的0.844782米使用出生MODEL点中心；二者差0.048605米来自不同的坐标载体。使用当前栅格中心与当时估计root复算，四个时刻均与原guard日志相差小于1e-10米。30秒更换true/estimated root不改变该最小值，因此不能把这项差异直接当tracking失败或低安全裕量bug。参考保留检查连续/栅格门槛0.6米，规划栅格门槛0.870711米；职责不同且均为必要筛查、没有3D安全认证。旧标签数组承载163个MODEL格，并非163个真实观测障碍。

新增reference_identity_audit_v1.json、reference_guard_original_log_scope_v1.json和reference_distance_coordinate_audit_v1.json，以及两份可复现CPU脚本。初次环境探查用python命令退出127（该别名不存在），改用已配置ball_wm解释器后退出0；未改变实验数据/参数。下一步须针对实际reference消费边界做有来源的诊断：当前consumer只有不可刷新MODEL，没有持续观测进入map，先验证MODEL与当前/历史实测的独立角色和冲突处理，不能从这次距离口径解释直接推出改guard/PID或删障碍候选。
