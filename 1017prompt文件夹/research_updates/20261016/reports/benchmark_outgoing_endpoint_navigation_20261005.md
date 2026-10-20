# 出段终点前瞻：原35秒真实导航验证

2026-10-05 11:46UTC有界阶段。唯一冻结查询修正已真正进入执行：相对上一条cusp轨迹真实进度增加 **1.320864m**、目标剩余距离减少 **.794760m**，少一次换向循环；但横向RMS/peak更大，因此原五项限定改善仍拒绝。相对较强Guarded历史对照，五项有限改善均通过；相对Root对照进度/目标项失败。实际导航仍35秒超时、距目标 **3.961658m**、物理接触0，不能准入成功导航baseline，也没有新E0/E1/E2成功率。全RGB/估计位姿/学习世界模型研究未完成，automation ACTIVE。

## 控制器身份与唯一变化

W=`/home/an/Documents/ChatGPT/球型机器人导航`，H=`experiments/stereo_memory_navigation_20261004_v1`，原项目只读。复用已经完整读的使命/主计划/ledger冻结前缀，完整bytes及outgoing_document_sync SHA再次精确检查，完整重读最新阶段报告/最新闭合段；宽输出截断不冒称fresh全文阅读。上一条cusp实际finished/exit0与上轮finalcheck通过，无未续进程。

本轮spec/plan `docs/superpowers/{specs,plans}/2026-10-05-endpoint-navigation*.md`，既有自主授权/隔离H；unbornGit以源SHA+ledger保留身份，不无关commit/delete。上轮outgoing_lookahead.py SHA `f0abeb4cd5d335be472fe43fefb4ee9e96bb21b9b3d8b1fca5495881e3e469c7` **保持原字节**。EndpointLookaheadPolicy继承CuspProjectionPolicy/RootDirectionPolicy，act/root规划未覆写；reset只装配tracker。真实innerMRO OutgoingEndpoint→ConfidentSpeed→MappedBankOracle→CuspBoundary→Oracle。原act/换向handler、当前未接受投影hold、五次XY<.05与反弹清零、合法参考替换、rootcost.5/reverse1.25/node3000/primitive.3、BankMap/PID(.4,.1,.7)/速度逻辑/aux速度.3/协议/smoother、来源记忆及unknownrisk不变。

runtime与上条只有机械policy import/construction、协议/输出名字及完成提示差异，diff=`H/endpoint_navigation_runtime_diff.patch`。相机仍X=.20m/Y±.30m/Z=.05m、640×480、原mask/照明；左右当前renderer depth+knownpose构点，显式特权上限层，**不是纯RGB或estimatedpose**。外层privilegedFalse表示不读取全局Oracle reference/map，不能洗掉depth/pose特权。脚下先验不写成观测自由；旧RGB/M2/停止包络拒绝不倒改。

唯一query先消耗当前waypoint的出段到终点，再检查下一条出段方向；不进相反edge，不改latch/强制提前反转。目标改变触发原速度schedule是同一策略的响应，非新速度策略。上轮字面RED13fail2pass、GREEN15pass/fullH210证据保留，本轮同15fixture **fresh PASS1.40s**，未重跑旧科学矩阵/RED或拟合模型。新脚本AST和实际MRO核验，137源/11342旧输入于 **2026-10-05T11:49:58.157440+00:00** 封存；原所有数值gate与guarded协议完全一致。

## 实际启动、失败保留与数据

共**2次进程启动、1条完成的物理序列**。第一次受限sandbox 2026-10-05T11:50:05.220470+00:00→2026-10-05T11:50:11.344764+00:00，GPU/Vulkan/CUDA设备不可见、SDK缓存lock报错，初始化SIGSEGV(-11)，没有NPZ/JSON采集、physics或metrics。原空输出namespace/status/full日志保存preinit_failure；不计导航失败/成功。独立gpu_launch addendum事前声明唯一环境重试、控制/source/gate不改，经自动权限审查获既有GPU/SDK缓存访问后启动同一已封存runtime。不是掩盖重跑或调整候选。

有效运行 **2026-10-05T11:51:16.855484+00:00→2026-10-05T11:54:05.194954+00:00** exit0，原6秒观察前缀包含在35秒计分内，350请求/351双方capture/1751physics/2101接触采样，其中0contact。双视频各351帧、10fps、无计分外尾步，hiddenmarker/曝光时间当前0age；marker primvar SDK警告保留原log，不因error字样虚报运行失败。所有close后PID反馈与产物纳入post-close manifest **2762文件**，pre-close manifest范围单独披露。

真正在线规划4次成功（6/14/17.5/33.5秒），其余正常guard保留；不是拿旧日志回放冒充新导航。33.5因original_raster_clearance拒绝保留而重建，2次remaining_arc_under_lookahead重建。没有本轮 observed_obstacle_blocks_route失败，也没有原cusp25/25.5秒再次循环的真实因果标签；后续路线/来源改变是新执行的结果。

## 实际分叉与因果接口

相对cusp第一真实raw分叉 **17.5s/frame175**。此前175完整请求、876physics×17nonraw、176完整执行PID/smoother snapshots、176双方source/history packets精确一致，raw原点差 **0秒/0tick**。不是声称与另外两臂都到此共同前缀；guarded/root已在各自阶段分叉。

17.5秒共同当前位置/同新version3参考，F0出段query从起点到idx1，实际PP+.622023；原cusp主轴1.5/aux-.010311，本次原schedule主轴.5/aux-.403104。后续 **102raw块/875真实applied ticks** 不同，因此不是仅把±35°提前写成±30°的等价改名。分叉以后全部成绩、轨迹、参考、bank来自本条自己的实际数据；不拿旧cusp未来标签或上轮188what-if当预测。

作者完整核验时间 **2026-10-05T11:55:27.055915+00:00**：137源/2762实际hash、350policy当前输入→真实request/diag/map/reference重建精确、351双depthpacket/350单次history移位、3有限过去state checkpoint未来payload trap/truncation一致。1745tick用实测当前bank/aux位置速度复现确定性PID/protocol/smoother/supervisor，首5tick未clone。此为**实测反馈执行一致性，非无真值递归物理预测**；3checks不是3条freshphysical。

## 独立几何与原验收

固定oracle_rgb_shadow_reference仅离线评估、所有臂同project函数，与候选path_index无关。没有切换投影或靠停车制造误差改善。

|实际臂|终态真实进度m|距目标m|横向RMS m|横向peak m|contact|35s结果|
|---|---:|---:|---:|---:|---:|---|
|Guarded B0（较强历史诊断）|4.855576|4.156980|0.174067|0.475266|0|超时|
|Root cost（已拒绝）|5.389775|3.671378|0.572390|1.105993|0|超时|
|Cusp projection（唯一因果对照、已拒绝）|3.707562|4.756419|0.135045|0.260893|0|超时|
|Endpoint query（本次唯一候选）|5.028426|3.961658|0.171278|0.384271|0|超时|

原五项：进度增益≥.05m且距目标下降≥.05m且RMS/peak≤对照+1e-5且0contact，分别对三臂应用，无放宽。对**cusp**前两项及contact过、两项横向不回归失败；对**guarded**五项全过（进度+.172851/目标-.195321/RMS-.002789/peak-.090995）；对**root**进度和目标项失败。三臂联合有限gate拒绝，导航goal≤.2原35sgate拒绝。五项比较不等于全面baseline认证；所有实际臂均未到目标。

独立4m空间检查点从cusp27.4秒提前到本次18.6秒（不是索引跳过），2m同13.9秒，6/8m未实际通过，6m点最小距离1.147126m。真实路径长度6.474272m也包含倒车/横向摆动，不能拿它代替独立进度5.028426m。

![四臂真实轨迹与冻结评估器](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/endpoint_navigation_comparison.png)

## 换向、bank与完整失败分段

本次只有两次accepted交接：20.1 F→R、23.7 R→F，分别五个当前XY速度(.028922,.012088,.018995,.030709,.036772)与(.038903,.023859,.010169,.002447,.010876)m/s全部<.05；保留反弹清零，不用关节过零替代。方向制动38块=3.8秒，旧cusp74块=7.4秒。停稳确认仍是合理代价，不能把延迟当bug或删规则。

真实90negative raw ticks/139negative applied ticks/230negative main-joint ticks（全条分母1750，主轴joint按采样位置[1:]计）；三层不等价。version3 idx1R→idx2F长.298971m，在自己175→335参考寿命内最接近终点.033647m、有向投影max.342228m；是真实运动通过附近的连续几何证据，不据索引/accepted另造二元primitive成功gate。寿命内其它F运动的signed progress末值为负也不表示倒车整段失败；该范围negative ticks不专属某primitive。

bank PID仍是base/root bodyY世界Z分量asin(2(wx+yz))，不是壳滚动Euler累计角或完整atan2 Eulerroll。下面用实际PID原始反馈，heading以**当前有效tracking.reference_body_heading_rad**并逐行与冻结当时reference/projection_idx对应，不用episode数组旧baseline字段。无效braking/earlyreturn参考heading标None，数据1750ticks保留PIDraw/protocol中间量原文件 `endpoint_navigation_run/feedback/bank_feedback_golden_e1_0001.json`。

作者首版描述性heading sidecar误用了legacy policy_reference_heading_rad（恒零），保留audit_endpoint_heading_feedback.py和heading_feedback.json作为**拒绝的测量口径v1**，不当可用结果；独立v2只修当前字段来源，逐valid行验证knownref/bodyheading，source/control/gate不改。报告下表采用v2；不是第二条物理实验或再次调候选。Euler yaw与worldZ angular integral分别保存，不认定相等。

|实际阶段（秒）|独立进度Δm|进入→退出横向m|横向peak m|实际Euler yaw Δrad|bank误差RMS rad|PID自身clip|
|---|---:|---:|---:|---:|---:|---:|
|0.0→6.0 observation_settling/forward|0.033762|0.000001→0.000747|0.036983|-0.001437|0.017257|0.00%|
|6.0→13.5 tracking/forward|1.659951|0.000747→0.227996|0.229698|-0.873520|0.139230|11.20%|
|13.5→14.0 goal_braking/forward|0.187760|0.227996→0.220157|0.227996|-0.076911|0.102640|0.00%|
|14.0→15.9 tracking/forward|0.871873|0.220157→0.169713|0.221518|-0.262506|0.109151|0.00%|
|15.9→16.2 goal_braking/forward|0.175602|0.169713→0.139932|0.169713|-0.015065|0.043182|0.00%|
|16.2→16.3 tracking/forward|0.058442|0.139932→0.127750|0.139932|0.000617|0.170798|0.00%|
|16.3→17.3 goal_braking/forward|0.461697|0.127750→0.082861|0.127750|0.016709|0.053228|0.00%|
|17.3→18.1 tracking/forward|0.213388|0.082861→0.143297|0.143297|0.015638|0.350380|75.00%|
|18.1→20.1 direction_change_braking/forward|0.154605|0.143297→0.268844|0.275639|0.068397|0.109113|0.00%|
|20.1→21.9 tracking/reverse|-0.206000|0.268844→0.256834|0.293736|0.048570|0.415646|68.89%|
|21.9→23.7 direction_change_braking/reverse|-0.148003|0.256834→0.155513|0.256834|0.049335|0.072332|0.00%|
|23.7→35.0 tracking/forward|1.565348|0.155513→0.384271|0.384271|1.338789|0.126795|73.81%|

20.1→21.9 R的真实Euler yaw **+.048570**、worldZ积分 **+.060030**，本次方向正响应已出现；旧cusp R19.7→21.6为-.053950/- .057851。窗口/状态/请求历程不同，不能由这些差直接标定符号、PID增益或模型因果。R current profile heading误差.103072→.063200，但bank误差RMS.415646/peak.610090、PID夹边68.89%，说明短时强侧倾仍未充分跟上动态请求。

23.7接受F时profile heading误差.292202rad（旧cusp对应.460381，但已非同状态受控A/B）；直到35秒仍是普通同向tracking，真正全参考进展+1.565348m，不再停在障碍前。末段横向从.155513涨到.384271，phase body-heading误差peak.528624/末有效.056954，bank误差RMS.126795、PID夹边73.805%，有误差但不能仅归首次交接。234有效moving/tracking请求中协议clip **127/234=54.2735%**、V1范围外 **103/234=44.0171%**，曲率绝对中位.812188。26/30/33.4秒实际请求k1.641341/2.185785/1.660542、BankMap夹边，实际速度约.183/.170/.193。直接把clip提前规范化等价、删除速度supervisor或改PID尚无单因果证据。

计划wall p95 2.814430s、capture部分p95 0.303882s；因果在线不等于墙钟10Hz，零接触不认证3D/动态/unknown安全。formal/RGB/pose/learning均不在此次准入范围。

## 保存证据与下一项边界

![新实际视频帧：上排外视、下排固定双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/endpoint_navigation_actual_video_frames.png)

原视频 [外视](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/endpoint_navigation_run/third_person.mp4)、[双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/endpoint_navigation_run/stereo_view.mp4)。全部来源、预seal/runtime diff、fresh15测试、startup失败/环境addendum、实际350/351/1751数据、PID、两视频、assessment/motion/v1拒绝v2反馈字段/verification、图/源SHA保留。未修改旧source、报告或gate，没有无关提交/上传。

目前确定查询bug已修且有真实行为变化，较强历史对照有限gate过；仍拒绝导航和三臂联合gate，不装成已经成功的baseline。下一个有界CPU工作应复用**本次237→335同方向实际轨迹**，分开planned/body profile k、实际PP几何目标、BankMap当前速度支持夹边、PID动态bank与真实XY几何曲率/航向响应，核验33.5原栅格guard失败是否来自真实位置偏离或已测地图来源。只读已有data/knownref，先明确欠转弯/时间预算/来源哪个环节限制，不能预先称PID错误。若已有观测证明唯一传统消费改动有必要，另封存后最多一条实际对照；不立即继续扫增益/减速/裕量，也不重做标定/世界模型。RGB+estimatedpose、唯一学习记忆/世界模型和原35秒E0/E1/E2仍待分层推进，全任务继续ACTIVE。本heartbeat无第二候选/新physics/进一步search。

## 独立审查与作者收尾

ONE fresh readonly reviewer `/root/endpoint_navigation_review`（gpt-6-astra），0 Critical/0 Important/1继承Minor，裁决可关闭本有界实验并保留失败，不准入成功baseline。审查bodySHA `28799514b5b030e991060fa8f682d5b26bbf7f257fb1d3d842cc3ec61b93f45e` 原字节保留；本append/fresh authorclosure/check不冒称二次审查。

Minor：`assess_endpoint_navigation.py:88` 沿用 `rejected_root_is_causal_comparator_only=True` 命名，实际本阶段唯一因果对照为 **cusp_navigation_run**；root_direction_run是另一个已拒绝历史比较，guarded为较强对照。实际B读取路径、correct causal_comparator string、各自数值gate和报告均正确。保留sealedsource/assessment原字节，另 `H/endpoint_navigation_comparator_alias.json` 给出权威字段解释；未来assessor需修正继承字段，不将旧boolean当控制器身份。该Minor不进入本query控制候选修复。

审查者亲自137source/11342oldinputs/2762actual/31packageextra+reportSHA精确；15fixturefreshPASS1.36秒；重算4臂几何，first175/876×17physical/176fullexec+source/history/maps/provenance/raworigin0、102raw/875applied改变，两五低速接受及90/139/230负tick、234currentheading/12phase/1750bank/smoothed/applied对应，检查两PNG像素。审查者首个比较float64 smoother与float32 physics严格不等，核验生产storage后按float32保存层全1750精确，bankmax1.67e-16/applied精确；只是readonly审阅算术修正，不改数据/运行science。

完整350policy/351bilateral/350history/1745measuredexec/3futuretrap仍是作者执行证据，reviewer仅source/log/result/hash检查，不运行planner/sim/inference或作者closure。18项declined逐条理由/作者裁决/误判成本保存在endpoint_navigation_review.json，分别保留RGB、估计pose、学习、正式rate、跨场景、递归physics、futureproof、safety、real10Hz、PIDcausal、primitivegate、不同后续状态、startup归因、v1reject、更多run、genericAPI、未来行政、整使命scope。作者另readonly确认automationACTIVE，无scheduler变更。

两个环境启动尝试/一条完成physical与v1legacy测量拒绝/v2来源修复透明披露；旧阈值/源/输入/数据不改。作者close/check仅append行政证据与freshhash，结果须以H/endpoint_navigation_final_check.json为准，整研究NOTcomplete/automationACTIVE，无后续science/physics调参。
