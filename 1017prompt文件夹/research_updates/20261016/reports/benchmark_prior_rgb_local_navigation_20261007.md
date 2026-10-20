# 双目估计深度的实际局部闭环：当前观测与历史记忆

2026-10-07；整体研究未完成。用户已批准固定相机、平地任务先验、起始安全区和受控未知探索这一独立任务。本轮实际运行两条完整物理序列，没有换网络、拟合、扫阈值或修改旧门槛。

## 结果与适用范围

| 同起态2米局部任务 | 当前/最新来源 | 10秒历史来源 |
|---|---:|---:|
| 终止 | 到达 | 35秒超时 |
| 仿真时间，含6秒初始观察 | 22.6 s | 35.0 s |
| 真实终距 | 0.193029 m | 0.597168 m |
| 障碍物理接触采样 | 0 | 0 |
| 真实沿起点→目标方向终态进度 | 1.828553 m | 1.404166 m |
| 真实轨迹长度 | 2.332308 m | 1.764329 m |
| 到目标直线的横向 RMS / peak | .039155 / .104423 m | .024017 / .069361 m |
| 相机采样 / 50Hz物理采样 | 227 / 1131 | 351 / 1751 |
| 实际双目网络推理 | 46 | 71 |

**当前观测臂通过事前冻结的窄局部到达标准；历史候选本次未通过，拒绝准入。因RGB重置输出不同，不能把性能差值作为纯记忆增量的因果效应。** 历史臂虽然横向偏差更小，但没有实际到达，不能把停走导致的偏差下降写成改善。上述直线误差是同一任务的描述量，不是沿局部参考的tracking误差。

这是 golden E1 原场景/起态上沿原目标方向截取的2米任务；不是完整E1导航成功率，不是RGB+估计位姿，也不是冷启动盲区安全证书。当前已知相机/机器人位姿为仿真实测真值，深度则来自真实左右RGB推理。原完整场景/35秒规则与旧深度、位姿、M2及导航拒绝结论保留。没有训练学习记忆/世界模型。

## 实际执行链

冻结 `ball_stereo_front020_up005_v2`（X=.20,Y=±.30,Z=.05 m）、原mask/照明。冻结Fast B0双RGB、416/8迭代/1.5px左右一致性；每5个10Hz采样推理一次。两臂均为已知位姿投影→有来源PointSupportMap→既有局部曲线与Endpoint tracker→原roll PID/protocol/smoother/supervisor；只有地图是否保留多个过去来源不同。主轴未知探索±.3请求、辅助轴实际速度限制.3、PID增益与方向锁存规则完全相同。

任务平地与起始安全区是独立bool层，**没有写成观测free，也没有跟随机器人移动**。当前控制主要依据保留unknown标签的探索规则和观测hazard筛查；这两层先验存储不是全3D可通行认证。当前足迹及未来运动没有被本轮证明充分可见。无全图/Oracle参考在线输入，运行器为离线评价生成的occupancy与Oracle路径不传给policy；原marker在传感器捕获时隐藏。

世界端点使用产生RGB时的实测相机位姿，原query/source时间与像素ID保留；重复消费不刷新来源年龄。最大缓存年龄.4秒。当前臂中也会出现“historical”代码：它表示最新缓存包比查询时间旧，**不表示当前臂累积了多个来源**。因此current臂的peak历史hazard41不可解释为10秒记忆效果。

GT深度在RGB控制包固定后才另存offline_truth，原始RGB输入schema没有深度/真值标签；它用于下面的事后诊断，不用于推理、点筛选或控制。两臂各完整重置一次，序列内部不重置物理/smoother。

## 记忆为什么退化，以及因果结论的限制

真实请求在frame135/13.5秒首次分叉；此前676个物理采样的17个非raw字段、136份执行状态精确相同。共同226动作块中91块请求改变、455物理tick applied改变，因此该候选不是行为等价的字段调整。

13.5秒：current保留原参考且继续前进；history有32个hazard格，原参考连续与栅格clearance筛查不通过，局部规划返回 `observed_obstacle_blocks_route`，请求变为零。历史臂随后长时间停车，并在后面再次被阻断；不能把首次分叉自动认定为完整导航失败的唯一原因。

独立重置后的RGB/网络输出并非逐字节相同。事后诊断最初对此作强断言，实际失败，失败源码与日志保留。同源离线诊断固定history臂状态/旧参考/最新来源，current-only重建也未通过筛查，因此不能把实际第一次阻断唯一归因于记忆；两臂RGB差异是必须保留的混杂。另重放current臂完整已有来源前缀，历史地图筛查也通过，故两个同源检查均未支持“13.5秒纯记忆导致分叉”。该frame左右RGB平均绝对差1.353/0.675像素强度、confidence差异4.168%，有限共同深度平均差.023016m。当前与历史之间23个额外hazard格不是独立证明它们导致此次首次停车。结果另存boundary_diagnosis；它不是新实际在线A/B，也不能据此断言历史障碍全部是假障碍。没有用GT删除历史点或重新调TTL来救分。

原始当前/历史来源与实际控制产物均保存。事后当前像素真值诊断：current累计107984端点，其中92894点三维误差≤.1m；history90459/82435。有限真值条件下误差p90分别.056618/.057278m。current预测floor106点全部对应真值非floor；history112点中110对应非floor。大多数端点属于非地面表面，条件误差较低不代表盲区有地面覆盖。少量hazard落在真floor（2/5点）也不能解释所有阻断。原严格完整分母/99%/90%来源门槛不由这些事后不同轨迹统计替代。

## 运行、测试与核验

预封存1245份源码/输入身份后实际执行，UTC19:18:27→19:28:10，两臂exit0。预飞TDD literal RED→GREEN，完整H测试366 passed /12.55s；不重复物理矩阵。实际身份检查5270项通过，四条视频完整解码227/227/351/351帧、10fps。左右render clock与实际capture clock差为0；网络/规划期间断言物理时钟与状态不前进。

作者个人离线重建117份来源包与576帧地图，3456个地图数组精确一致，两臂各有未来包payload陷阱通过。它是产物重建，不是新采集/模型预测。评估脚本在第一次运行前修正了runtime_error契约：实际metrics使用0.0/1.0，而非None；此修正不改变科学门槛或运行行为。辅助因果诊断第二次因NPZ零维frame未转换到int失败；第三次的“同history最新包current-only应通过”断言被实测否定，保留三份失败源码与日志，撤回该假设而不调整门槛；第四版报告实际同源筛查结果。未修改已预封存执行源码。

模型实际wall延迟p50/p95：current .953/.972 s、history .938/.963 s；policy p95 .149/1.185 s。仿真暂停等待计算，**不是实时10Hz导航**。这也是下一阶段部署预算需要处理的实测缺口。

原episode派生元数据的 `oracle_path_length_m=2` 只是短任务占位，原metrics中的SPL/reference_length不作为正式导航指标；没有求得短任务Oracle最短可行路径。用真实位置/终距/接触/时间独立验收。

## 产物和接续

本轮实际轨迹：

![真实轨迹](../experiments/stereo_memory_navigation_20261004_v1/prior_rgb_navigation_trajectories.png)

左右源图与来源地图：

![左右源图与地图](../experiments/stereo_memory_navigation_20261004_v1/prior_rgb_navigation_sources_and_maps.png)

H=`experiments/stereo_memory_navigation_20261004_v1`：`prior_rgb_navigation_protocol.json`、`prior_rgb_navigation_status.json`、`prior_rgb_navigation_assessment.json`、`prior_rgb_navigation_author_replay.json`、`prior_rgb_navigation_boundary_diagnosis.json`、`prior_rgb_navigation_source_diagnosis.json`；原始两个 `prior_rgb_*_run` 目录含全RGB/预测/世界包/地图/局部计划/状态/物理/接触/执行与视频；评估、复核、诊断源码和失败日志保留。

接续不再训练另一深度网络或调整记忆TTL。先保留current窄局部可达结果。下一项恢复原完整目标、同版本current控制35秒诊断，同时在同一真实来源流上只读计算history请求，隔离重置RGB混杂；shadow不改变B0动作、不把另一请求计划与B0未来轨迹当标签。随后才依据实测决定具体局部消费改动。RGB+估计位姿、可信显式记忆、学习增量和原35秒E0/E1/E2全层结果仍未完成。每小时自动接续保持ACTIVE，整体未完成。

## 独立终审及作者裁决

ONE fresh readonly reviewer：Critical0/Important0/Minor1；个人5fixture2.44s、四份预封存执行/测试SHA及主要数值核对。完整366测试/5270身份/117来源576地图/未来陷阱/视频为作者证据，reviewer未重复；上游runtime不重新审计，新增完整目标shadow阶段不在本轮review范围。

Deferred minor：旧辅助诊断stdout“causal guard difference verified”措辞过强；保存日志不改写，本报告/JSON才是结论，明确纯记忆归因不成立。这个非数值minor不进入改代码修复流程。

作者裁决：numeric runtime_error契约按0.0无错误读取（若错会误判到达）；NPZ零维frame恢复为int以匹配实际runtime输入（若错会破坏产物重建）；撤回RGB精确身份假设并保留混杂与三次失败（若仍把它忽略会错误归因记忆）。上述均不改实际序列、旧gate或候选。整体仍未完成，继续同源只读shadow与原完整目标诊断。
