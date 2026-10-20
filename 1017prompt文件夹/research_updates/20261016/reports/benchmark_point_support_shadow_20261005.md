# 稀疏来源记忆与完整扫掠支持：2026-10-05 01:38UTC

**记忆保留了更多点，但没有补足这条路线的地面支持：两臂完整333格始终全部未知，消费者拒绝。** 这一次只读实现和实际回放已完成；没有新导航成功率，整个研究未完成。

## 问题与固定预算
复用上一阶段原E1的完整181双目对/568冻结causal保留点与已知位姿层，不重采、不重新推理、不训练、不改变控制。本轮190源码/参考/点输出身份在2026-10-05T01:47:24.005734+00:00封存，362地图在2026-10-05T01:47:26.673320+00:00全部保存后才重读离线标签；这些标签此前已用于诊断/设计，是历史开发验证，不能称fresh held-out。
两臂都接收**完全相同的causal当前query点估计**。current仅本帧query，无以前的空间记录；history保持原query时间10秒。两臂深度都可能用过去眼图，不能把current称单帧双目或把以前current/causal深度增益混作本轮空间记忆增益。
图范围metadata origin[-6.099999994,-5.799999994]、123×117/.1m仅坐标extent，不输入任何occupancy内容。每点仅world/sigma/query-source帧眼索引时间及已知pose_sigma0进入mapper；旧RGB candidate/gate拒绝保持，颜色格只是**RGB来源候选endpoint代理**，始终没有certified_free/模型填充区域/行动授权。

## 新消费语义与因果约束
不做ray-carving、点间插值、脚下seed、真值筛点或把空白写free。继承world-Z地面0与旧.05m floor、.05—1.5m nominal hazard范围：floor需abs(z)+sigma_m+pose_sigma≤.05且总量≤halfcell；hazard覆盖同格floor，原query时间TTL内保留。sigma_m是像素扰动敏感性，**不是正确对应或误差概率保证**；knownpose_sigma0也不是估计位姿性能。忽略分类/图外记录仍保留，未来/坏时钟不读payload、不先污染状态，重放旧query不刷新时间。
使用前一轮**采集前已封存**的preflight绝对world_xy参考，完整6秒转弯+自身6秒停车601点；不读取未来真实轨迹/反馈来构造计划。在所有181观测时刻检查同一参考，作用是固定空间支持诊断，不能称每一当前状态都适用的新运动方案。连续线段到格中心距离检查，旧.7m半径+grid半对角不变；未知/图外和hazard均阻断，正的floor代理也只是2.5D必要条件而非完整碰撞体积认证。

## 实际地图与空间分母

|地图|6秒记录/正确/无标签|末态记录/正确/无标签|峰值floor格|峰值hazard代理格|末态hazard代理格|完整扫掠最少未知|
|---|---:|---:|---:|---:|---:|---:|
|current|4/4/0|2/2/0|0|4|1|333/333|
|history|252/248/0|249/200/46|0|20|17|333/333|

history在6秒仍保留4个已测错误记录，18秒仍3个错误、46无有效renderer标签、247过去query（上一记录一致）；没有用这些离线标签在线删掉它们。旧源误差、body-ID不足/nearestpixel+亚像素ray标签限制保持，不把249记录当249实体或地图面积。
起点disc187格、转弯300、匹配停车225、whole333；每臂181次、每一个phase都0次通过，所有required格没有floor/hazard端点，只有unknown。尤其6秒开始运动前的起点187格完全未知；18秒事后保留记忆也未解锁。不能拿运动已经实际走过或物理0接触反写这些格为观测自由。
更直接的空间证据：**全部568原始保留点的nominal XY，没有任何一个落在这333格中**，在height/sigma分类之前已是0；所有点world-Z为0.936991—1.015382m，原.05地面带内0点、被新floor敏感性规则额外删掉的nominal floor也0。因此这次不是仅把可用floor筛得过严或安全半径稍大；当前记录的是约1m高的表面点，所需近场地面/路径空间没有被这一点表示提供。**这限于冻结ORB与此观察序列/endpoint表示，不能推出所有像素、所有方法或此固定相机永远不能导航。** 稀疏点不覆盖整格体积，射线经过也不自动认证整球扫掠。

## 独立标签的有限风险代理
仅在362地图落盘后读旧bodyband occupancy和上一阶段query有限renderer标签。floor格始终0，所以floor与已知occupied交集false-free为0是**空集合的平凡结果**，不是安全验证。history hazard代理与旧occupancy自由格交集峰值11、末态10；末态另7格与occupied一致。current交集峰值3、末态1。这些只是离散2D位置/高度定义/源表示代理，不能自动认定每个交集都是真实误障碍或真实物理碰撞；定位跨格、body-ID和完整3D均不足。已测错误3点在记忆中仍存，是真实污染证据，但不能把10代理交集等同10错误物体。
本次mapper没有学习预测区域；no-imputed-regions=0，future truth无输入。独立body-ID unavailable、ORB全金字塔支撑未认证、scalar sensitivity不是真界、原renderer/网格/曝光适用范围都沿用旧限制。

## 测试、身份与实际运行
六个手算fixture RED6→GREEN6；首次missing module collection error另留，然后空模块出现6项真实缺功能失败。134完整H CPU测试2.64秒通过：稀疏点间未知、uncertainty不能写floor、遮挡hazard/TTL/冲突/不刷新、NaN/future/atomic、连续中段+自身stop+图外、future packet prefix。原模型/相机/所有历史report/gate保持，stage190预封存输入与此前1131新产物/文档在开始前匹配。
actual consumer replay181帧/362地图真实运行exit0，所有packet/maps/source-records/完整missing indices与摘要保存；作者重建、未来替换/截断、旧SHA与独立审查范围分别记录，不能把实现通过继承为观测准入。图已实际查看；绘图默认cache不可写而转/tmp只影响cache，没有采集或计算失败。
没有新仿真/动作/模型推理/视频；[前一条原E1完整双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/active_observation_run/stereo_view.mp4)和[第三视角视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/active_observation_run/third_person.mp4)是复用来源，不假称本轮新运行。正式35秒E0/E1/E2、RGB+estimated pose、学习/世界模型均没有本轮结果。

![任务空间与endpoint记录](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/point_support_spatial.png)
![全部181时刻支持与記忆](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/point_support_time.png)

## 关闭分支与下一项实际工作
关闭“冻结ORB+本次主动观察+保守endpoint支持能够解决严格冷启动”的分支：来源总gate已拒绝，本轮任务空间必要条件也独立拒绝。正确表面点增加不能代替路径支持；不再补动作、放大点/射线成free、扫sigma/半径/TTL/网络，亦不声称只有一个物理根因。具体阻塞是**没有任何可用近场floor端点来支撑运动前起始disc及自身停车扫掠**；严格unknown-blocked语义下不能据此发导航动作。
回到总任务中独立的控制/系统基线。下一项有界工作：核验现有成功Oracle的当前实际class/继承/act/执行/PID/rig移植身份，然后最多一条**原35秒完整E1 Oracle已知地图诊断、RGB/来源记忆只读shadow**。沿用已冻结成功控制器/参考/增益与速度配置，不做同轮调参或让shadow改变动作；旧身份不全就限制历史比较，先CPU/入口核验再启动，实际完整路径/到达/接触/时延/相机数据/版本保存。只读shadow只用当前/过去图像与已知本层位姿，不能把Oracle全图/路线/未来反馈写入RGB地图或评分为纯视觉。
这项是明确的Oracle辅助上限与完整在线观测接口验证，**不是解除RGB源/地图gate，不是新纯视觉导航成功率，也不启动正式E0/E1/E2矩阵**。它提供真实完整导航与观测序列，结束把总项目持续卡在同一局部点/停车审计的循环；其后视觉闭环仍需独立来源/未知风险任务条件验证，已知地面或safe-start若必要必须单列prior而不冒充观测。估计位姿与唯一学习增量继续按总体计划分层，不能跳称完成。全任务未完成，automation ACTIVE，常规不发完成通知。


## 作者初次完整复核与文档身份范围
2026-10-05T02:01:03.899957+00:00作者复核完成：1248去重旧/预封存身份、362地图逐项重建、181净化packet等价/原FeatureMemory有效点等价/未来payload抛错及截断、4phase冻结raster计数与独立已有连续projection格集合、1448个地图/phase直接索引分类一致。所有gate仍拒绝。输出日志被验证器明确排除，不重复上一阶段自输出散列问题；最终封存另存。
旧采集协议记录的active-observation计划全文件SHA与现在不同，是**上一阶段收尾追加了关闭段**。本轮没有略过该文件：原采集前prefix SHA `75204b3a1f4ca47abcb13c8d7767d2f0d409bffe27bf23e2b41bb9d0e4ac9888`精确匹配并另存；现完整doc SHA `da551acb3e9abd74b47cf769ea9b166c3190c0a86ba3a67c4726fcc83787c3e3`也与上一final文档同步记录一致。所有其他旧源/输入/产物全文匹配，不能混称这个追加文档的全文一直未变。
补加一个独立文档prefix/已记录closure完整身份fixture RED1→GREEN；原六测试文件190预封存身份已恢复逐字节原样，新fixture分独立测试文件，保留临时合并版本。分文件首次缺pytest import的1 fail/134 pass、前两次identity核验失败日志均保留；修正fixture import后135全CPU测试2.68秒通过。mapper/runner/协议190/362地图/评价/gate未变，没有重采或重推理。
一次作者只读验证把不变四phase几何重复1448次，作者主动Ctrl-C(退出130)该核验进程，保留源副本/日志；改为4次几何+1448地图索引核验，随后的完整作者验证退出0。这是删除重复验证成本，不是实验被中断或少检查地图；独立审查尚待完成，不能用作者验证器自动背书。


## 最终独立审查与作者封存
一次fresh-context只读审查 `/root/point_support_review` 完成，Critical/Important/Minor均0。独立135 CPU测试2.67秒通过；1248旧/预封存身份和审查时577新产物身份匹配；重建362地图、逐条核验181净化packet及历史FeatureMemory记录，独立复算4段连续扫掠187/300/225/333格，1448地图/phase全部未知；复算568点高度与空间位置、402使用过去source、末态249/203/200/3/46记录和occupancy代理计数。181真实prefix未来payload替换/截断对记录及四个地图数组均一致，两图视觉检查；封存/输出/标签顺序、预采集参考与当前/历史报告身份一致。
审查正文SHA `26dd1e2c0d36dd244e8651953799ee92d938056f68ea84e4eddf01eaed9a0f58`，本段为随后作者追加，不能冒称独立审查覆盖追加后的全文。审查不执行作者验证器/仿真/模型推理/新实验；旧科学全矩阵复审、所有像素覆盖、固定rig永久不可导航、全3D认证和未来Oracle表现不在范围。审查的一次断言误假定10Hz时间可十进制精确相等，改为逐字节档案时间一致后通过，未改变任何数据或候选。完整证据保存 `point_support_review.json`；随后作者最终复核与文档同步单独记载。来源与消费者继续拒绝，全研究未完成，自动续做保持ACTIVE。
