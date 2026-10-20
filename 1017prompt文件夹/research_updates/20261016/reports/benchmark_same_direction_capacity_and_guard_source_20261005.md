# 普通同方向跟踪：曲率能力与33.5秒重规划来源核验

2026-10-05 12:46UTC有界CPU阶段。复用endpoint_navigation_run，完成唯一预封存实际数据审计，无新仿真、控制候选、planner搜索、policy.act、拟合或推理。找到**名义规划与转向映射支持不一致**及其真实饱和证据；后半段bank已接近目标，仍无法输出要求的纠偏曲率，不能再把PID夹边直接等同“roll没控住/增益不足”。33.5秒重规划来自实际位置变化与原栅格化规则，不是33.0→33.5新障碍观测或未来地图。旧35秒超时、所有旧gate拒绝保持，整个RGB/估计位姿/学习研究未完成，automation ACTIVE。

## 范围、版本与方法

W=/home/an/Documents/ChatGPT/球型机器人导航，H=experiments/stereo_memory_navigation_20261004_v1，原项目只读。复用此前完整读的使命/主计划/ledger前缀，经当前endpoint_document_sync全bytes/hash精确检查；最新完整报告及闭合段读完，宽输出截断不当fresh全文证据。上条实际finished/exit0/finalcheckPASS，无遗留仿真。

独立spec/plan docs/superpowers/{specs,plans}/2026-10-05-tracking-capacity-audit*.md。只读237≤frame<335的98正常forward tracking（version3，23.7→33.5，未把335新参考当旧参考）。逐当前valid/source字段核验projection_index/profilebodyheading/actualroot，并重查BankMap.query每输出与实际mapping记录精确。无legacy baseline heading、制动陈旧诊断、二次有状态policy调用。原query/PID/速度/supervisor/rig/light/五低速/源地图/未知策略不改。

6字面CPU fixture：直线、半径.5的±有向圆弧、机体自转与XY直线分开、静止不可估计、非有限输入拒绝。可调用旧域混淆scaffold RED **4fail2pass/.16s**，唯一measure_turn实现GREEN **6pass/.12s**。分别测三点外接圆signed XY curvature、Euler yaw变化/实际polyline弧长、world-z角速度积分/实际弧长；这些**不是同一个量**。1秒不重叠窗口（最后.8秒），真实弧长≥.05m及两半弦≥.01m/全弦≥.01m才几何可估计；阈值预冻结，只是描述性测量资格，非机器人稳定/控制成功gate。

离线窗口使用完整实测位置，包括窗口内后续实际样本，**不是在线当前可读输入、模型预测或另一计划的真实标签**。10/10窗口可几何估计、0被删，第一短位移/-9.56大曲率也保留；可估计并不等于稳定圆周运动/硬转弯能力。

实际路径探索时误猜3个文件名和宽rg输出截断，仅read错误未启动科学；首次heredoc W-relative路径在cwdH失败，未创建脚本或协议，exit2保留prewrite_failure.log。改回正确cwd写入，无改测量/控制/gate。本轮 **144源/14148旧与实测输入** 于2026-10-05T12:56:02.888173+00:00封存，一次完整审计2026-10-05T12:56:18.580804+00:00→2026-10-05T12:56:19.612774+00:00exit0。没有重复原矩阵/全部导航。

## 名义模型、请求与真实bank

98条均为同一个计划支持区间，projection索引2→6的原命令main+.5/aux-π/6，V1 nominal body k **+.8800519662/m**。当前BankMap速度加权正向端点只有 **+.7962012→+.7966033/m**；**98/98名义值已高于该端点**，实际纠偏PP还要求约1.3→3.1/m，**98/98 production map夹边、98/98 protocol clip**。这说明当前consumer使用的两个模型假设在该区间不一致，不靠调raw±35→30可以修复。

BankMap实际实现是按low/high速度行各自逆插值/夹边，再混合bank；表中的加权曲率端点是描述符，不能泛化为算法完整可行域或动态最大曲率。80/98实测速度低于BankMap low endpoint，实际代码保持speed clamp；这尤其限制把其当低速/换向动态模型。映射范围不是经过本轮新fresh校准的物理极限，不能据此改旧测量gate或封死实际可能的大曲率。

真实目标bank始终-π/8≈-.392699rad。23.7初bank+.027549rad，误差确有明显动态滞后；23.7→24.7 bank均值-.022188/RMS误差.372614，24.7→25.7均值-.203914/RMS.196893，不能断言“PID一直正常”。但27.7→28.7 bank均值-.392692/RMS.006156；31.7→32.7均值-.394213/RMS.001566；33.4当块bank均值-.393705/RMS.001022，已经接近目标。与此同时请求k1.66/m仍被限制、投影距离到.425525m。

全98×5=490tick PID自身clip 377/490，速度supervisor 5/490，两个执行限制不同。PID clip多不意味着实际bank角永远达不到，也不能由低supervisor比例断言删限制无影响。只是增加PID增益或移除aux速度limit都尚无单因果支持。

## 真实XY弯曲与机体朝向

|实际区间秒|XY实走弧长m|三点XY k 1/m|Euler yaw/XY arc 1/m|worldZ积分/XY arc 1/m|实际PP k中位1/m|真实bank均值rad|bank误差RMS rad|
|---|---:|---:|---:|---:|---:|---:|---:|
|23.7→24.7|0.060811|-9.561248|0.526317|0.202115|1.456835|-0.022188|0.372614|
|24.7→25.7|0.176164|-2.195759|0.373958|0.209345|1.341968|-0.203914|0.196893|
|25.7→26.7|0.173025|0.022276|0.725283|0.656694|1.700929|-0.351667|0.051245|
|26.7→27.7|0.156413|-0.795331|0.795099|0.793979|1.937853|-0.409902|0.017648|
|27.7→28.7|0.159063|1.341540|0.708792|0.726732|2.152353|-0.392692|0.006156|
|28.7→29.7|0.164112|0.946246|0.732956|0.720064|2.689565|-0.394603|0.006437|
|29.7→30.7|0.168754|0.293992|0.782681|0.783077|2.245127|-0.410147|0.017517|
|30.7→31.7|0.176372|0.938617|0.795092|0.803005|2.707160|-0.401219|0.009469|
|31.7→32.7|0.185172|0.915487|0.787094|0.783750|1.512603|-0.394213|0.001566|
|32.7→33.5|0.151768|0.668563|0.788167|0.784743|1.606556|-0.394808|0.002175|

25.7→26.7 body yaw/arc+.725283而XY k+.022276；26.7→27.7 body+.795099、XY **-.795331**。后续27.7→28.7 XY+1.341540而body+.708792；31.7→32.7 XY+.915487而body+.787094。实际晃动/横向运动会使短时轨迹与body heading不同；三点circle是粗窗口几何，不把其当稳态物理最大值/即时曲率或拿这个差证明滑动/符号bug。只有接触/滚动速度分解才可证明滑动，本轮没有新做那项。

计划k+.880来自原V1 body模型，PP几何要求更大，BankMap反算的是其自身body响应表；本轮不把几何XY k和映射body k串成错误的性能预测。连续同方向段出现实际bank接近目标但请求超支持的证据，足以拒绝“单调加PID就能解决全部导航”的假设，不能证明PID优化完全无益。

![实际请求、bank、XY与body量分开](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/tracking_capacity_measurements.png)

## 33.5秒guard与可追溯观测

将诊断时钟固定在335，旧reference version3早在175已知；位置330/335和map330/335都来自当前或过去。四次**只读必要几何screen**，不是planner搜索/另一控制轨迹，不拿335数据预测330未来。真实330/335 screen逐字段与原record精确一致。

|实际位置frame|观测地图frame|原guard结果|
|---:|---:|---|
|330|330|保留|
|330|335|保留|
|335|330|拒绝original_raster_clearance|
|335|335|拒绝original_raster_clearance|

所以本次 .5秒触发的局部原因是位置/投影改变，map版本交换并不改变这一screen结果；这是这四个输入组合的因果接口证据，**不是证明观测变化对完整导航永远没影响**。

实际335离参考连续投影 **.431061m**（未过原.5m门槛），剩余arc1.136545m（未触原.5m门槛），suffix/bridge连续最小hazard距离同 **.640678893m**，≥原.6；原rounding EDT最小 **.583095189m**，<.6，故拒绝正确执行冻结规则，不能删除raster check、放松.6或说“没有碰撞所以是假碰撞”。这不是碰撞终止，是真实重规划。

两个失败sample分别属于suffix首点和bridge终点，是同一个连续投影( .236793445, -2.645012981 )，round到格235/174中心( .199999332,-2.599999982 )，偏移 **.058137568m**。最近已观测hazard格232/179中心(-.100000668,-2.099999982)，grid距sqrt(.3²+.5²)=.583095189，continuous投影到它.640678893。只隔这一栅格化差异，不宣称新几何规则可提高导航/安全。

hazard provenance code4历史占据，**timestamp13.099999707秒/frame131**，不是新335观测或地图先验。沿正式norm≥.55m/root、.05<z≤1.5、rint cell过滤，从原sensor_packet131找到**3个真实深度点**，高度.528326846→.643834203m，source SHA `8d5c8f7d0baee43a6c7939b2d9c8c87555f4e782d4815c226d2177e9af1217a6`。当前floor的10秒TTL不用于删除hazard，SpatialMemory对hazard保存最后实测时间并持续占据；不能把20.4秒过去观测说成当前RGB看到或未经测量的先验。点来自特权depth/knownpose层，不是双RGB匹配成绩。

![实际位姿、旧参考与原栅格guard证据](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/tracking_capacity_guard_witness.png)

## 验证、拒绝结论与下一项

作者2026-10-05T12:59:10.556788+00:00 freshhash/scalar/chronology检查144源/14148inputs、98唯一current rows、10几何窗口、98双clip、4screen/2source witness、旧navigation拒绝，assessment SHA `71bf22100510566736b3e45621cf28f6654fa0230a0e61ad20c218543020dd86`。无policyact/planner/sim/fit/candidate，verify不重新运行完整scienceaudit。图像已视觉检查；本轮无新视频，上一条真正35秒两视频与所有源/REDGREEN/失败/audit/数据/provenance/图保存。旧报告/源/验收不改，无无关Git提交/上传/清理。

本轮不安装控制候选：不存在新的确定PID/supervisor实现bug；guard行为符合原规则，模型范围不一致有来源证据但不等于可用动态新模型。拒绝单PID增益/删速度限制的直接归因，拒绝将栅格screen失败当虚假碰撞，拒绝把当前BankMap端点认证为所有状态的物理曲率上限。原35秒endpoint超时3.961658/进度5.028426/RMS.171278/peak.384271/contact0及三臂联合gate拒绝保持；较强guarded有限gatePASS也仍非导航成功。

下一项有界工作应针对**规划名义响应与实际执行consumer的一致性**：先审查原V1/BankMap的曲率定义及命令域，明确可声明的steady nominal适用性（不当短时动态模型）；在一个已知完整当前快照上验证一个只改变名义曲率响应来源的候选适配器/必要screen，冻结原搜索/mapper/PID/gains/speed/rig/guard，并同时比较原计划几何、真实请求是否不再必然夹边、goal/clearance/路径长度及换向数。最多一次有界CPU对照，不先跑物理或扫阈值；CPU资格通过才另阶段最多一条原35秒同起点对照。禁止用当前速度预测全部未来响应或旧实际轨迹充当另一计划label，也不强制拒绝所有reverse/等停稳循环。本stage不再search/sim。RGB+estimatedpose、唯一学习记忆/世界模型和原35秒E0/E1/E2继续分层未准入，整个任务ACTIVE。

## 独立审查与作者收尾

ONE只读第二人阶段审查：Critical0/Important0/Minor1。工具拒绝新fresh-context线程（agent thread limit reached），因此复用上一阶段审查者，保留旧上下文；不称fresh-context。审查本人6fixture PASS .12s、144source/14148inputs/13extras/bodySHA逐值一致，98current rows、490feedback(377PIDclip/5supervisor/80speedclamp)、10窗口公式、330/335真实guard与frame175已知version3、frame131三个源点/时间/高度/栅格距离及两图核验。四个cross-map screens与正式BankMap query完整复现属于作者执行证据；审查未重跑audit/guardmatrix/act/search/sim/fit/fullsuite，行政收尾另由作者执行。

Minor M1 deferred：完整测量条件包括 arc≥.05m、两半弦长各≥.01m、总弦长≥.01m、有限数据。总弦长条件在预封存helper与本报告中存在，但原spec句子和protocol数值字段漏列。十个实际窗口全通过，既有结果不改；旧source/spec/protocol/body原样保留，下一个协议须完整枚举，不能由本追加倒改旧门槛。重分级保持Minor，因为本次结果/消费者没有改变；重建者应以冻结helper及此明确补充为准。

审查17项暂不判断范围逐条由作者裁决如下（非新验证）：

1. Fresh-context reviewer independence：Agent limit prevented fresh reviewer; retained-context second-person check is explicitly weaker. 若错代价：Correlated prior-stage assumptions.

2. New physical curvature envelope：No new calibration; current endpoints are mapping descriptors only. 若错代价：Overstated physical capability if promoted.

3. PID gains as unique failure cause：Early lag and later target tracking do not isolate gains; no PID remedy admitted. 若错代价：Misdirected controller tuning.

4. Removing supervisor or auxiliary limits：No counterfactual action experiment; fractions cannot establish benefit. 若错代价：Unsafe or ineffective limit removal.

5. Slip or rolling-contact identification：XY/body descriptor disagreement alone is insufficient; no slip claim. 若错代价：Incorrect contact-mechanism attribution.

6. Instantaneous or steady-state curvature：Ten windows remain finite-time descriptive measurements. 若错代价：Transient measurements mistaken for steady model.

7. BankMap feasible set from weighted endpoints：Original per-level inverse/clamp then bank blending retained; weighted range not general feasibility. 若错代价：Incorrect candidate eligibility.

8. General map-change irrelevance：Four screens concern one known reference and two local positions/maps only. 若错代价：Underestimating observations elsewhere.

9. Raster replacement safety/navigation gain：Original .6 raster rule retained; no replacement or navigation benefit admitted. 若错代价：Unsupported safety claim.

10. Current visibility of historical hazard：Source131 proves past privileged observation only. 若错代价：Historical memory mistaken for current visual detection.

11. RGB-only or estimated-pose admission：Privileged-data audit does not reopen old RGB rejection. 若错代价：Leakage into visual baseline.

12. Learning/world-model or recursive prediction：No model fit/validation; measured successors are offline descriptors only. 若错代价：Untrained model falsely admitted.

13. Navigation rates or baseline admission：No new physical sequence; old35s failure unchanged. 若错代价：False navigation success.

14. Episode/environment generalization：One archived interval; scope limited accordingly. 若错代价：Overgeneralized research conclusion.

15. Future nominal-response adapter：Unimplemented proposal; next independent frozen stage must establish eligibility. 若错代价：Premature deployment of unsupported geometry.

16. Closure hashes and live scheduler：Subsequent author administrative verification and readonly ACTIVE snapshot are separately attributed. 若错代价：Author closure mistaken for reviewer verification.

17. Whole-mission completion：Broader RGB/pose/learning/formal evaluations remain unfinished. 若错代价：Premature automation termination.

作者readonly scheduler当前ACTIVE已保存；整项研究未完成。任务源码/REDGREEN/失败/产物/图/原视频/SHA保留，user留存与安静要求优先于skill清理/无关Git提交/逐项chat通知。审查后作者追加与fresh hashcheck另署，不冒称第二次审查。
