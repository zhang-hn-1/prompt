# 规划名义曲率与执行映射支持：唯一候选CPU对照

2026-10-05 13:47UTC自主阶段。完成一个真正运行的同快照CPU对照：原规划完全复现；唯一规划域限制候选消除了所选曲段的映射域不一致，但路径变长、原规划目标函数更差，**预封存CPU资格拒绝**。没有运行该候选的物理导航、没有新成功率，不改旧35秒结果或放宽门槛。研究整体继续ACTIVE。

## 执行范围与身份

工作区 /home/an/Documents/ChatGPT/球型机器人导航；H=experiments/stereo_memory_navigation_20261004_v1；原项目只读。上阶段使命/主计划/ledger此前已完整读过的前缀经tracking_capacity_document_sync全bytes/SHA一致复用，本轮完整读最新report/plan/closure。大输出截断和失败猜名不冒称完整阅读。上一阶段finalcheck144源/14148inputs/29产物/5docs通过，无上条未结束仿真。独立spec/plan为docs/superpowers/{specs,plans}/2026-10-05-nominal-support*.md；执行brainstorming、writing-plans、TDD、executing-plans/系统诊断流程，用户已有自主授权优先于重复审批。保留现有隔离H与unbornGit SHAledger，不无关提交/删除。

真实链仍为 EndpointLookaheadPolicy → CuspProjectionPolicy → RootDirectionPolicy → GuardedReferencePolicy。CPU搜索使用冻结 direction_root_planning.plan_kinodynamic_path；请求使用冻结direction_root_cost.prepare_request/snapshot_problem，原root方向来自175交接前已接受forward。实际跟踪器仍make_endpoint_oracle的OutgoingEndpoint/ConfidentSpeed/MappedBankOracle/CuspBoundary/Oracle组合。唯一新SupportLimitedDriveModel是原DriveModel子类，仅供本次planner.curvature调用；**没有安装到tracker.drive_model**。继承speed/inverse/braking及原配置/数据原样。

名义反算并不直接等于真实动作：实际tracker先计算PP、原V1inverse，再由BankMap重算倾侧请求，叠加原yaw feedback、协议裁剪，后续PID/smoother/supervisor才执行。本CPU对照没有调用act，没有模拟执行链或输出新的applied指令。

## 定义核验：同名body不等于同一测量

原V1静态JSON主轴命令域±.5/±1/±1.5，aux±30°，测量时间2026-09-12；DriveModel.curvature对aux和命令幅值插值，speed是其原响应表，planner把该曲率用于signed-arc理想圆弧。**没有证实当时原采集器源码身份**：当前probe_drive_dynamics.py同时保留legacy Euler-yaw/signed-speed函数及新版XY几何估计，不能把当前脚本当历史原始生产者。结论只对当前冻结V1数值及其消费者成立，不能把历史记录的k认证为严格XY动力学。

本轮追查active BankMap表，与oracle_bank_mapping_20260929_v2/bank_mapping.json逐字节相同。其fit_mapping.py调用measurement.geometry：XY Savitzky-Golay一阶/二阶导数，默认31采样/.02s，speed≥.08，k=(vx*ay-vy*ax)/speed³；冻结稳态窗口中位数乘sign(main)，命名curvature_body_1_m。**这里body表示正反行进符号规范，不是root Euler yaw/路程，也不是world-Z角速度/速度。** 标定输入是请求bank，实测bank仅另记诊断，不是副轴关节目标的直接响应。当前正式query按low/high各自逆插值并判断clip，再按实际速度混合bank，不能用加权曲率端点当其完整可行域。

这项生产者源码核验在CPU结果后完成并独立封存nominal_support_definition_audit.json；没有重新运行fit/标定、改变候选或门槛。上阶段数值结果/原报告保持，涉及“body曲率”测量来源的解释以这里的明确定义补充为准。两个来源的同名值不足以证明共同的动态物理模型；本候选因此只做数值域筛选。

## 唯一候选与冻结实验

推荐候选只有一个：原V1名义curvature按main符号，裁到BankMap同方向low/high两个数值支持区间的**交集**。不读取当前速度外推未来，不拟合M3，不调PID/速度/相机/clearance。原有效域内值保持；命令/speed/braking/辅助动作范围不改。可保证输入落在两个表域时正式query各层不发生曲率clip，不代表速度域准入、bank追踪、物理最大曲率或短时恒曲率运动。

|方向|共同曲率下界1/m|上界1/m|
|---|---:|---:|
|forward|-.8272253275|+.7815023661|
|reverse|-.7978797555|+.7672796845|

测试7failed/4passed/.29s的callable no-cap RED，唯一实现GREEN11pass/.25s；完整H CPU回归227pass/9.43s（已有SDK库，无新增安装）。字面边界数值与真实BankMap查询验证正反向极端、原可行值/速度保持、原模型隔离、0/NaN/Inf、交集为空拒绝，以及实际42种primitive命令的consumer域。它们是CPU接口测试，不是重新采物理矩阵。第一次cwd错误导致两源码重定向失败与缺文件pytestexit4，未创建source/未science；该失败不是RED。最初失败test log被正确RED覆盖发生在封存前，保留工具输出转录prewrite_failure.log，明确不是原pytest日志。第二次行政relative Path(.)找spec失败、无文件改写；seal随后完成。这些错误与科学运行分开。

唯一现状输入：endpoint actual frame175/map17.49999960884452s，当前XY/yaw/pointgoal和已接受forward；完整执行snapshot175另存身份，未作为未来物理预测。准备脚本只抽record175/当前观测，已知ref175仅比较原规划，不取176+实际轨迹或335未来guard作标签。输入权限在协议单列。原35秒/相机安装/照明/五低速与反弹/mapper/PID/原速度策略/guard保持。

协议封存时刻 2026-10-05T13:57:06.100102+00:00。152sources/14187旧与实际输入。只有一次共同grid route构造和**两次Hybrid A***（原+一个候选），原node3000、primitive.3m、sample.05m、reverse1.25、switch.5m、local target2.4m、clearance.6m。spec缩写的“两搜索”以预封存协议的明确Hybrid/shared-grid计数解释，未追改已封存文档。无其他搜索/参数/seed。

CPU资格事前冻结：原参考精确复现，候选success/≥2点、同local goal误差≤.2m、原raster sample无占据或越界、全部选中名义curvature在共同域且三次正式map查询不clip、换向次数不增、路径和原名义目标函数均不超过B0+.3m、并严格减少所选域外曲段。不看结果放宽，不由path_index造成功。

## 实际对照结果

总审计 2026-10-05T13:57:43.268009+00:00 → 2026-10-05T13:57:51.059837+00:00 退出0，原搜索13:57:45.227981→13:57:48.106192，候选13:57:48.121312→13:57:51.056495。原175的pose/command/direction在1e-10内精确复现，expanded2434与归档相同。后续物理真值不用于候选。

|指标|原规划|唯一候选|
|---|---:|---:|
|status|success|success|
|扩展节点|2434|2482|
|曲段数|10|12|
|名义路径长度m|3.0|3.6|
|名义原目标函数m|4.075|4.7|
|名义local goal误差m|.018737071|.174290166|
|内部换向次数|2|1|
|reverse曲段数|1|8|
|选中域外名义曲段|7|0|
|采样primitive占据/越界|0|0|

候选不是同路径上修饰数字：原方向F/R/F（1F、1R、8F），候选4F/8R，先向下绕行再较长倒车返回local goal。未知仍为明确探索风险；原.6栅格不碰观测障碍只是必要screen，不认证3D/盲区/动态停止安全。原目标函数含reverse1.25与switch.5，减少一次换向仍无法抵消更多reverse与长度：目标函数增加.625m，路径增加.6m，均超过事前只允许+.3m的门槛。

当前175的相同位置、body yaw与同当前速度.339178215m/s，旧endpoint query几何PP+.622022716，候选-.400174820；两者当前BankMap曲率clip均False，speed clip均False。它们是已知候选参考的当前几何/映射描述：没有完整schedule/方向brake/PID/protocol/smoother/applied重建，不能写成实际动作改善或新的转弯轨迹。原首方向与候选首方向均forward，不强迫提前反转。

7项资格通过：原身份、候选搜索成功、local goal、raster、名义共同域、换向次数、域外曲段减少。2项失败：路径≤3.3m、原objective≤4.375m。**CPU资格拒绝，候选不进入物理A/B。** 不扫描边界、不改变目标函数/节点预算/验收来救本候选。

![仅名义几何与共同模型域的同快照对照](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/nominal_support_comparison.png)

## 验证层与保留结论

作者 2026-10-05T13:59:48.329266+00:00 fresh SHA/计数/原身份/时间与gate核验152/14187，assessmentSHA 188a81f06e8796ae459c38856aed84c14776a3e55b51ec8642ba8acd3bc44aef，未重跑两搜索。两阶段源/输入/旧报告/gate保持；plot实际查看，Matplotlib unwritable默认缓存自动写/tmp成功，不是运行阻塞。CPU测试、唯一源码/原scaffold、失败转录、current175/execution、协议/原和候选path/当前query/结果/图/producer来源均保留。本轮没有新视频，上一条actual35s视频仅上下文。

分层结论：源/代数域接口验证通过；完整局部CPU资格拒绝；动态转移/物理跟踪未测；视觉深度/估计位姿/学习世界模型未新增；正式导航未触发。原endpoint35s timeout/goal3.961658/真实进度5.028426/RMS.171278/peak.384271/0contact与原三臂联合拒绝保留，较强guarded限定gatePASS仍非导航成功。没有通过模型层自动升级导航层。

下一项具体阻塞是**名义路线选择的代价没有代表35秒执行预算**，并非继续削减曲率端点能自动修好。已归档成功Oracle有独立time_planning/hybrid route选择，而当前局部planner仍用primitive长度/reverse倍数/固定换向代价；本轮候选暴露8段reverse的结构代价。下一阶段先追踪这两个实际消费入口/单位/速度来源与移植身份，利用同一个已知175输入作一个预封存、有界的路线时间预算核验；只在定义和来源明确时试一个已有时间代价消费环节，不能同时重新调curvature cap、tracker、map、PID和速度。拒绝候选本轮不安装，原参考和所有数值gate不追改；不拿旧未来B0轨迹当新路线真实标签、不把静态steady时间认证为倒车/交接实际延迟。其后物理候选仍须通过独立CPU资格、同初态原35秒进度/goal/横向/contact对照。全研究未完成，automation继续ACTIVE。

## 独立审查与作者收尾

ONE fresh-context只读独立审查：Critical0/Important0/Minor1。152sources/14187inputs/21extras/8definition来源/bodySHA精确；本人当前175/执行snapshot/过去地图timestamps、原ref175/2434nodes、两臂已保存primitive端点/raster.05采样/共同域/路径/换向/目标函数/goal误差与图逐项核验，fresh11fixturePASS.21s。未调用audit/grid/search/act/sim/fit/fullsuite，RED7fail4pass/作者227suite/两搜索进程证据是检查过的作者执行日志。后续行政归档和hashcheck由作者执行，不冒称第二次审查。

Minor M1 deferred：正文末段“具体阻塞…代价没有代表35秒执行预算”是过强因果表述。实际代码只证明当前objective没有时间项，本快照只证明候选路径和objective退化；**尚未隔离时间代价缺失作为实际35秒导航失败主因，也没证明时间代价候选有效**。下一阶段定位为预封存的来源/时间预算假设核验，不能自动升级为原因或修复。保留审查正文、sealedsource和全部失败gate；此追加限定是权威解释，重分级Minor因为没有据此部署控制或放宽准入。

审查未判断范围逐项作者裁决（不冒称新验证）：

1. Historical V1 producer identity：Current script is not archival identity; retain unproven measurement-producer status. 若错代价：Misattributing old measurements.

2. V1/BankMap physical equivalence：Numeric domain alignment is not a physical equivalence proof. 若错代价：Deploying an incorrect response model.

3. Candidate physical tracking：CPU eligibility rejected; no physical run or applied response admitted. 若错代价：False prediction of tracking success.

4. Complete raw/applied actions：Current geometry/map queries do not reconstruct scheduler/brake/PID/smoother/supervisor. 若错代价：Mistaking algebra for executed intervention.

5. Speed/steady-state admission：Common curvature domain does not reopen speed-domain or stability gates. 若错代价：Out-of-distribution control.

6. PID gains as failure cause：No gain intervention or single-cause isolation. 若错代价：Misdirected tuning.

7. Removal of action/supervisor limits：No such intervention authorized by data; limits retained. 若错代价：Unsafe or ineffective control.

8. 3D/blind-region/stopping safety：Original sampled observed-hazard raster pass is necessary only. 若错代价：False safety certification.

9. Generalization beyond current175：Single frozen case; do not claim broader feasibility. 若错代价：Overgeneralization.

10. Navigation success/rates：Candidate failed CPU gate; all old35s results untouched. 若错代价：Invented navigation baseline.

11. RGB/estimated-pose admission：Input layer privileged; previous RGB failures retained. 若错代价：Oracle leakage into visual claims.

12. Learning/world-model admission：No fitting or learned-memory validation occurred. 若错代价：Unsupported learned-model claims.

13. Time-cost effectiveness：Next bounded source/budget audit is a hypothesis test, not proven remedy or unique bottleneck. 若错代价：Wasted research effort if hypothesis wrong.

14. Alternative caps/parameters/seeds/rescue routes：Single candidate/two-search budget exhausted; no rescue search. 若错代价：Selection bias or budget inflation.

15. Independent historical execution-count attestation：Reviewer checked source/logs/hash/chronology but did not personally monitor original calls. 若错代价：Overstating independent runtime evidence.

16. Scheduler/later closure hashes：Author readonly ACTIVE snapshot and subsequent administrative check attributed separately. 若错代价：Closure mislabeled independently reviewed.

17. Whole-mission completion：RGB/pose/learned increment/formal35s still unfinished; continue ACTIVE. 若错代价：Premature termination.

用户留存/静默自主要求优先于skill删除workspace/无关Gitcommit/聊天逐项通报；所有原始版本与源码/data/test/plot/oldvideo/hash保存。作者readonly scheduler ACTIVE另记录，whole research未完成，fresh administrative checker另署。
