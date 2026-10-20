# 球型机器人双目记忆导航：自主研究任务

用户于2026-10-04授权：按照已经讨论的完整方案独立开展研究、实验和验证，完成后通知；无需重复审批日常可逆实验。任务尚未完成。

## 目标与边界

固定最新相机安装，使用真实双RGB匹配估计深度，在机体晃动和前方近场盲区下保持空间记忆，结合可执行局部运动与已有roll PID完成导航。随后隔离位姿估计误差与学习记忆/世界模型的增量，按原35秒规则验证E0/E1/E2。研究结论可以拒绝候选，但不能把未运行的模块写成成功或把只完成诊断写成全部完成。

工作区 `/home/an/Documents/ChatGPT/球型机器人导航`；原项目 `/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm` 只读。Python `/home/an/miniconda3/envs/ball_wm/bin/python`。已有GPU/Isaac环境，不修改硬件安装位置，不无限拟合网络、扫描阈值或重复相同矩阵。没有授权上传、发消息给他人或提交无关文件。

相机固定 `ball_stereo_front020_up005_v2`：X=.20m,Y=±.30m,Z=.05m；640×480；实际原生极线垂直，图像逆时针90°后保留测得左右顺序；焦距584.8889px，基线.60m，沿用自遮挡mask和照明。

## 当前证据与续做入口

先读 `docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md`、`experiments/stereo_memory_navigation_20261004_v1/progress.md` 与 `reports/benchmark_stereo_memory_navigation_20261004.md`（若已生成）。每次检查最新ledger/实际进程与文件，接续未完成的工作，避免重复运行。

H=`experiments/stereo_memory_navigation_20261004_v1`。

- 已完成原/重采集各66对SGBM及一个冻结官方Fast-FoundationStereo。所有数据是开发/历史回归，不是fresh held-out。四套预测真实存在，旧 gate不追改。当前深度覆盖/困难场景误差尚不够，不能准入纯RGB导航。
- 已完成四条12秒观察、两条35秒显式未知风险导航及相同两条fixed3render重跑。风险两条均超时；E0进展.8971m、E2 .8608m。原/修正动作与物理状态完全相同，没有新成功率。它们使用renderer depth和真位姿，不是纯视觉基线。
- 发现两项来源/接口问题：旧地图每帧seed_body把滚动.4m足迹未知写成自由，冻结protocol unknown_labels_unchanged字段被代码否定；新SpatialMemory是独立实现，尚未被这些导航调用。报告必须明确，不能暗改旧结果。
- 静态已知平面证明当前时间戳不能保证标注器内容当前，fixed3render可清除静态排队。但后续移动probe证明cached/direct getter相同，部分帧用当前位姿地面误差p90达.22m，前一个.1s动作周期位姿则约1e-7m。因此不得宣称fixed3render已修好所有移动同步。
- 正在运行/已运行 `probe_motion_kinematic_sync.py`，仅测试官方物理view运动学缓存刷新，固定12秒前进停止序列、物理/时钟不变；检查最新日志与assessment，先因果核验再决定修复。不得用未来真值或盲目固定偏移掩盖问题。
- 新24测试/历史40回归通过。独立review指出上述footprint先验和旧观察采集限制；报告生成后需完成表述复核。

## 接下来的工作顺序

1. 完成移动时采集/曝光位姿对齐的受控验证。成功修复必须有独立已知几何或完整序列证据；若采用延迟观测，明确观测曝光时间及其过去位姿，控制仍读取当前状态。保留旧采集和gate，另立版本。
2. 将有来源标签的空间记忆接入实际控制；不把脚下先验混成观测，明确cold-start可观测性/允许的探索风险。用同一起点完整参考比较current-only、memory和bounded-observation，独立真实进度/碰撞/盲区指标。固定动作、场景和测试预算后才运行。
3. 对齐后的双RGB深度做一个冻结方法的针对性验证；保持不可估计/遮挡/未知语义，不凭conditional误差隐藏无覆盖。几何校验失败先查原因，不立即无限换网络。
4. 完成测得运动/停止尾迹的局部候选评估，并真正接入规划消费者；目前primitive_planner只是离线支持检查原型，风险导航实际仍是grid A*+旧几何tracker，不能写成已经实现该规划器。
5. 明确分层基线：Oracle/full-map诊断、depth+true-pose上限、RGB+true-pose、RGB+estimated-pose。采用独立完整fresh验证序列，冻结前封存，不随机拆相邻帧。视觉输入禁用未来值、全图、renderer depth、GT位姿（对应层除外）与隐藏导航markers。
6. 显式记忆基线形成后，只验证一个因果action-conditioned学习增量/世界模型；记录输入、训练数据、误差和完整闭环增益，候选不通过就拒绝。不要强行生成“可用模型”。
7. 在必要前提通过后运行原35秒golden/hard及E0/E1/E2，保存视频、轨迹、传感器/执行链、真实成功/碰撞/超时、运行延迟和版本SHA。资料/代码/结果分别交付，最终综合报告区分已完成、失败和外部阻塞。

每次续做必须执行有界的实际工作并更新ledger、必要报告和下一步，不只输出建议。保留源代码身份、失败日志、测试证据和结果。发现前提失败时继续一项有证据的原因核验或受控候选，不无边界补测。任务完成才发最终通知；只有必须由用户提供的信息、权限或不可自行解决的外部条件才联系用户。不得把机器离线/额度不足当成研究完成。

## 最新更新：本轮可接续结论

完整报告现已生成。运动探针的FK单因果归因已撤回；消融表明单次base capture之后的三次额外普通render已足够，FK不必要。已实现run_transaction_navigation.py，只更新一次capture后的RGB/视频，不二次累积历史或物理。两条risk_runs_capture_transaction实际35s完成，E0/E2各290个非零动作块，进展约3.29/3.31m，仍超时但不再早停，接触0。它们是特权深度/位姿+滚动足迹先验+显式未知探索，不是RGB导航。下一轮以该采集事务为冻结版本，先做对齐后的双目/来源记忆接入；需要速度候选时，依据35秒与实测速度预先冻结一个参数，不无边界扫描。29新测试/40历史回归通过，审查继续进行。自动续做已实际启用，ID automation。

最终独立审查完成，report已补充：第三消融仅17非raw字段/相对时间完全相同，raw初始化原点差一物理步；第二探针才19全同。视频356帧含旧runner计分外5物理尾帧，不走新事务，评分35s仍351capture；原协议全视频同事务字段有范围限制，见protocol_scope_audit.json。所有350history窗口只更新一次。无需重跑已完成两条采集事务导航，只复核更多场景/对齐数据、真正接入来源memory与运动规划。下一次严格从此最新状态接续。

## 最新自动续做：06:20UTC轮次已完成

定点重采e1/hard0004两条完整旧请求序列，共20对，均退出0；同一冻结Fast模型全部实际推理完成。31CPU测试通过。报告新增 `reports/benchmark_transaction_stereo_targeted_20261004.md`。相机/标定/极线一致，全部17项非raw物理字段与旧fixed3重采集精确相同（e1raw初始化原点+.02s）。模型gate仍拒绝：e1body全体≤.10m42.46%，hard0004 .76%，地面覆盖差。困难样本在置信过滤前已错，不可删除mask制造准入；不是只修采集就得到可用RGB。两条开发重跑不是fresh。没有待续的实际进程。

下一次优先真正接入在线来源memory与规划消费者，先在特权深度层核验unknown/历史障碍/足迹先验与独立假自由指标，保留RGB拒绝。不要再重跑已做2/6数据矩阵。来源记忆与实测运动仍未进入此前nav；全文研究未完成，不通知完成，不停止automation。

原阶段报告保持独立审查时原字节/哈希；新20对证据写入单独targeted报告，下一次需完整阅读该新增报告，不追改原结论。

## 最新自动续做：07:20UTC轮次已完成

需完整阅读新增 `reports/benchmark_online_provenance_memory_20261004.md`。新ProvenanceGrid真正进入在线policy/A*消费者，禁用脚下自由先验，两条E1current/history真实运行完成。记忆保留最多30个被遮挡且占据图一致的已测代理格，current0；地图可从过去packet精确重建。36CPU测试通过，来源接口通过，但两条仍碰撞（28.8/31.4s），不能把延后失败写成成功。history减少XY摆动、沿独立参考终态1.8649→2.3140m，目标距离减少略差；可靠导航拒绝。两条使用特权depth/真pose，不是RGB。

history30.5s发零请求、31.1s smoothed/applied主轴才到零，31.36s实际接触，继续.10645m；尚未5次停稳。主轴.3请求上限不是实际XY速度上限。下次实施有界实际局部执行/制动/扫掠支持消费环节，先核验A*目标曲线可执行性；不要重复本轮两条、改多个模块或回去M2调参。旧采集/报告/gate不改，新证据版本分开。E1独立评估参考按原runtime冻结A*/.5膨胀复现、长度与manifest完全一致，运行后归档时间已披露。没有待续实际进程。完整RGB+估计位姿/学习增量/正式评测仍未完成，自动续做保持ACTIVE，不通知完成。

## 最新自动续做：08:20UTC轮次已完成

请完整阅读新增 `reports/benchmark_local_stopping_consumer_20261004.md`。唯一实测停止包络veto真正接入在线消费者，一条同起点E1候选35s退出0；没有重复旧两臂。28.4s前17非raw物理字段与原history完全相同、raw时钟同源、284policy记录一致、28共同来源SHA相同，真实350动作与记录/因果重建一致。41CPU测试通过，349个历史移位各只一次。全研究未完成。

停止veto使本次实际接触从有到无，但超时、独立参考进度2.3140→2.1489m、横向RMS.7532→.8503；拒绝可靠导航改善，不能把停在障碍前写成成功。原实测.140884m有限停止尾迹在新大bank状态失效：首次veto速度.11345m/s却bank24.207°（旧14.041°范围外），1.5s径向.183946m；主轴applied到29s才零，30.6s五次低速资格时径向.210444m。没有倒改旧结果或调大半径重跑。

先前245非零请求91.84%副轴协议边界；同目标PP诊断曲率中位1.2219m^-1，95.92%超历史约.8尺度。但真实信息tracker是heading→aux，这不是其实际曲率请求；历史稳态bank逆映射当前速度范围不适用，不能直接安装或称硬上限。

下一项应针对局部几何与真实执行脱节：审查可复用的成功Oracle曲率可行组件和旧物理数据，只对缺失的大bank转弯/制动状态补最多两条预先冻结完整序列，再选一个可执行短段消费环节。不要重跑停止veto或扫裕量，不无限标定/换网络；不能把当前低速当成已覆盖停止保证。RGB深度gate仍拒绝，估计位姿/学习增量/正式35秒E0E1E2仍未完成。无遗留实际运行进程，自动续做ACTIVE，保持安静直到整个研究完成或必要外部阻塞。

## 最新自动续做：09:21UTC轮次已完成

请完整阅读新增 `reports/benchmark_large_bank_motion_coverage_20261004.md`。完成两条预先冻结大bank转弯/停稳序列，各18s、180请求/181capture/901物理，均退出0；没有重复E1停止veto/原矩阵。44CPU测试通过，17非raw6s前缀一致，负向raw初始化原点+.02s独立披露。视频各181帧且无计分外尾帧。实际数据、执行历史、PID、双目和版本均已归档，无遗留进程。

关键新证据：倾侧开始1.5s前向约.103m、侧向约±.148m，root yaw只改变约±.062rad；短时XY不能被机体yaw稳态圆弧替代。低主轴大bank零请求后1.5s径向.17845/.17367，仍超旧.140884停止尾迹；12.6s主轴目标为0，14.2s才五次实际低速。负向bank趋势1.0603deg/s超过预冻结1deg/s，因此即便目标已达到，实测稳态gate仍拒绝；没有放宽。正向bank稳定也不认证恒曲率轨迹。

审查旧成功Oracle10条历史metrics不重跑：wrapper9份均原SHA，但15核心来源中当前runtime/pointgoalrunner两份变了，且旧environment无camera_rig；保留历史5/7、3/3而限制当前整套移植结论。当前独立InformationPolicy不同，不能把旧全图/参考/速度一起搬进来称局部修复。相同PID和函数可以明确身份后复用。

下一项：基于已归档完整实测响应，实现一个有初态适用性拒绝的短段消费者，分开真实XY扫掠/停止段与root航向，核验当前速度向量、bank、关节、PID与完整smoother状态；不能任意拼接异状态尾迹。响应bundle只是数据，部分执行链字段不是已经可克隆的runtime模型。验证完整序列必须事前封存，旧两条只能作开发；不要再次采同一bank矩阵/扫停止裕量/拟合M3。RGB、估计位姿、学习增量及正式E0/E1/E2仍未完成；自动续做ACTIVE，不通知研究完成。

## 最新自动续做：10:21UTC轮次已完成

请完整阅读新增 `reports/benchmark_guarded_measured_response_consumer_20261004.md`。实现完整只读PID/protocol/smoother状态及实测响应消费者，执行一条封存新朝向+20°、18s序列，退出0；49CPU测试通过。实际选择positive并发出其完整已知请求，180请求/181capture/901physics，视频无旧尾帧。历史1800tick与新600tick实测反馈下执行链精确重建；这不是无真值递归物理预测。

同开放平面近cold-state的朝向迁移gate通过，整个12s XY RMS0.000126m/peak0.000304m，停稳时刻同为14.2s。这主要核验确定性同初态协变，不代表任意动态状态/导航。选择扫掠211格有195未知，仍返回unknown_risk而不是安全。历史E1的255个6s后状态，仅6s初态符合；其余254及28.4s大bank状态明确拒绝。旧RGB、停止包络及可靠导航拒绝保持。没有待续实际进程，不通知项目完成。

消费者只能执行“6s转弯+自己的6s停车尾迹”，不能任意截短为1.5s就停车。下一项有界工作针对短请求前缀分叉的匹配停车响应，最多两条完整序列（开发覆盖+事前封存独立验证）；复用完整执行snapshot/来源map，禁止拼异状态尾迹、扫半径、删除初态门限或重跑本轮朝向协变。必须明确连续重规划/重复段尚未实现，而非把此表强行接入原E1。RGB+估计位姿、学习记忆与正式35秒E0/E1/E2仍未完成；automation保持ACTIVE。

## 最新自动续做：11:22UTC轮次已完成

请完整阅读新增 `reports/benchmark_short_matching_stop_and_repeat_gate_20261004.md`。两条预封存完整序列13.5s/21s均退出0；开发数据建立唯一1.5s转弯+对应6s停车响应，fresh真正执行第一段并完成第二次选择。53CPU测试通过，实际数据/视频/执行状态/来源packet/SHA均归档，无遗留进程。

单段实测响应gate通过，XY peak0.0000352m；但13.5s整段完成且已连续38次低速后，关节状态、PID积分和smoother仍不同，第二段明确拒绝。不能删除门限或reset来制造通过。拒绝后7.5s零请求仍漂移0.14188m，局部目标距离减少不是第二段成功，更不是原35s正式导航。原RGB、停止尾迹和可靠导航拒绝保持。

新停车证据：7.5s速度.18208/bank22.187°发双字段零请求，主轴smooth到8.1s零，XY反弹.21774，10.2s才五次低速。1.5/6s停车径向peak.20270/.24202。停车回正段(+.20179,+.13362)m抵消前段90.64%侧向净位移；main零与bank回零同时发生，单独因果尚未证明。第三记录DOF和PID积分均持续变化，USD静态顺序不能替代实际dof_names；未据此改物理模型。

下一项只选一个受控传统请求消费实验：相同7.5s前缀与完整实际执行状态后，主轴仍零，bank参考保留此前已知22.5°(编码+30)，而不同时回零；与已经归档development同6s窗口比较实际停稳、速度反弹、XY/冻结局部目标进展。先封存数值gate/唯一候选再执行，模型准入不作该传统接口因果核验前提；不改PID增益/相机/速度/地图，不每结束状态再无限补表，不复跑旧矩阵或M3。若不能停稳/风险增大即拒绝。RGB+估计位姿、学习增量、原35sE0E1E2仍未完成；automation保持ACTIVE，常规不通知。

## 最新自动续做：12:24UTC轮次已完成

请完整阅读新增 `reports/benchmark_stop_bank_reference_hold_20261004.md`。只执行一条13.5s预封存候选、退出0，复用旧short development B0，不重复基线。55CPU测试通过；分叉前17非raw物理字段、75请求/76完整执行状态与来源packet精确相同，raw初始化原点+.02s单独披露。主轴raw/smoothed/applied全程相同，停车后300副轴实际指令均不同，接口是真实行为干预。

保留此前bank请求22.5°而不同时回零，通过封存局部制动gate：五次真实低速资格10.2→8.8s，停车延迟2.7→1.3；6s径向peak.24202→.17139m减少29.2%，路径.38608→.18790减少51.3%，资格后无越阈反弹。停车侧向B0+.13362m回撤→候选-.03015m，本次支持bank参考影响回撤的因果判断；不把它称为PIDwindup修复。末态bank24.58°/速度.01576/第三关节仍运动；未恢复旧冷初态，且6s径向仍超原.140884旧包络，所以旧拒绝保持。局部目标距离改善不是新导航成功。

实际DOF顺序joint1/joint2/joint3已保存读回，驱动gain原样；wrapper目标slew仍30°/s，基础runtime配置中的null不代表真实wrapper无限速。运行135请求/136capture/676physics/28对双目，视频136帧10fps无legacy尾帧；来源/请求未来截断poison不变，完整测量反馈执行链精确重建。旧41来源/产物、原report和robot层SHA不变。无遗留进程。

下一项有界实际工作：把同一“停车时保持此前已知bank请求”的唯一来源改动，用于E1历史来源记忆+停止消费者的真实运动转停车事件，复用已有local_stopping_run B0；最多一条35秒诊断，事前冻结独立参考进度/横向/停稳反弹/碰撞判据。按请求事件而非episode ID或固定时刻触发，不改A*/mapper/PID/速度上限/五次资格/反弹清零；旧veto半径仍是已拒绝保证的启发式，不扫描半径或认证安全。首次分叉前完整状态/执行/来源同prefix核验，恢复运动由原policy接管，不强制reset/反转。只改善停车但不能完成路径就保留导航失败，并针对已定位的参考执行/视觉覆盖，不无限给末态补表。

相机、旧RGBgate及所有历史结论冻结；RGB+估计位姿/学习增量/原35s正式E0E1E2研究仍未完成。automation保持ACTIVE，常规不通知完成。

本轮接口范围补充：实际runner在运行前由冻结的已知75块请求前缀预构造bank保持计划，7.5s时这些真实请求已逐值一致；没有读取未来真反馈，受控因果对照成立。但尚未实现/验证在线停车事件锁存器；下一项E1消费者正是验证该接口，不能把离线前缀poison测试冒充在线消费通过。冻结runner/protocol未修改，报告已明确这一范围。

## 最新自动续做：13:25UTC轮次已完成

先完整阅读新增 `reports/benchmark_online_bank_hold_navigation_20261004.md`。真正在线的停车bank参考来源接口已实现并执行一条35s E1诊断，13:32:39—13:35:08UTC退出0；复用已有local_stopping_run B0，不重复基线。59CPU测试通过，39新产物SHA/旧27与41清单/原报告不变，无遗留进程。

第一请求事件28.4s前1421物理采样/285父policy当前记录与来源packet/公共policy状态精确同B0，raw原点也同；旧完整执行状态由实际冻结链+过去反馈重建，候选实时snapshot一致。主轴raw/smoothed/applied全程相同，330停车tick副轴实际执行改变。在线接口与制动gate通过：五次真实低速资格30.6→29.8s，延迟2.2→1.4；6s径向.23994→.14689m减少38.8%，路径.31300→.15069减少51.9%。仍漂移且超过旧.140884停止界，旧拒绝保持。

完整参考和导航gate拒绝：两者35s超时无接触；参考进度2.14889→2.13017，目标剩余4.08557→4.23038，横向虽改善也不能算成功。224非零主轴块不变，另66bank保持块不算前进。当前consumer不默认替换正式baseline。

实际因果核验：停车后66次原A*仍有unknown_exploration路径、原tracker仍提议运动，但旧observed-hazard停止veto全部否决；方向锁存low=0，新的bank保持器没有等待低速资格。34s当前格原A*clearance.600通过，最近已测hazard格中心.6073m、旧stopdisc内6格，表明路径/执行与停止消费不兼容，不能去缩半径删除veto。既有轨迹在停车前已偏离冻结参考，当前主轴.3显式风险策略实际active速度均值.11830m/s、15→28.4均值.09509；8.83675m参考在剩余29s需均值.30472，即使6.6m直线目标也需.22759。不是普遍物理速度硬上限，但停车单改无法修补此进度/预算缺口；35s规则不变。

下一项有界工作优先为**复用现有实际在线路径/来源/物理数据的因果审计**：分开跟不上自己当时A*路径、选的未知探索路径与冻结参考不同、目标/请求长期边界、实际XYcourse/bodyyaw、路径扫掠与stopdisc兼容性，以及速度范围/35s任务可行性。不要凭冻结参考横向误差就预认定tracker bug；它没有在线读取这条Oracle参考。使用已有成功Oracle/物理数据核验可复用速度/曲率范围，再依据证据选唯一局部消费环节。下一项不再补停车bank、扫半径/增益/速度、拟合M3或每个末态补表；只有证据支持才封存一次实际控制实验。也要回到独立失败的双RGB观测/覆盖层，不无限把整项目变成停车诊断。

保存350请求/351capture/1751计分物理、350来源packet/在线输入、351离线depth、35maps、350policy状态、351执行snapshot(initialNone)、两视频各356帧含旧runner5计分外尾帧；已披露。相机不变，来源/完整policy/执行链及未来截断poison核验通过。纯RGB+估计位姿、学习记忆增量和正式E0/E1/E2仍未完成，automation继续ACTIVE，常规安静。

## 最新自动续做：14:27UTC轮次已完成

请完整阅读 reports/benchmark_online_route_execution_causal_audit_20261004.md 和新增计划 docs/superpowers/plans/2026-10-04-route-stop-clearance-single-consumer.md。本轮不追加仿真，实际执行一次350帧/70窗口的离线因果核验，35来源map/350真实target/350原protocol请求逐值重建一致，policy.act调用0。63CPU测试通过；连续线段clearance测量修正了点采样漏中点的问题，失败日志保留。旧39产物/报告/gate保持。无遗留实际进程。

当前path误差RMS~.034m会被.5s重新起点规划重置；44完整运动窗口的旧path退出误差RMS.04668/peak.17499，不能认证持续路线。15s历史path到28.4s误差.61495m；当时局部path本就与独立Oracle参考不同，不能全部归trackerbug。晚期course/path切线中位差31.29deg，course/body仅3.80deg；实际bank中位25.10deg已实现，XY曲率中位-.59199(非上限)，不预认定PID/符号/滑动。

具体可提前定位的接口不一致：原A*点clearance.6与原stopdisc.71159475不相容；19s目标弦.70711、20s局部折线.69293已经不满足stop规则，当前中心仍1.5/1.44m。晚期84/134折线、94/134目标弦不相容，直到28.4中心.71030才veto，之后66全拦。不得缩radius删除veto。唯一离线候选把规划点规则对齐已有stopradius：19/20/25s同来源图找到更兼容路径(下一米/目标弦≥.8)，28s后当前格已不满足，不能停车后才修。只是开发离线可行性，不是新控制成功。

旧成功E1实际30.7s/path8.8358，XY均速.28757，主轴请求中位.5/p901.5，明确不同于当前.3探索域；历史runtime/rig身份限制保留，不能把旧全图/速度/路线合并移植为局部修复。速度预算问题仍独立存在，35s不改。

下一项唯一有界控制实验按新增计划：最多一条35s E1，复用本轮原online_bank_hold B0，不重跑baseline；从同起点执行完全同B0前缀，在实时规划的首个“前一米不兼容且当前disc仍无hazard”事件处分叉，仅对齐规划clearance消费(不扫描/改变停止radius，不改PID/cap/mapper/target/hold/latch)。先测试与封存gate，实际空间进度/目标距离/运动量/持续veto检验，停车或索引改善拒绝。与当前低速预算仍不能完成目标分开报告；不默认替换baseline。该一次实验后必须返回失败RGB层，复用已有对齐20pairs拆共同可见性/纹理真伙伴残差/过滤前错误，禁止再陷入停车/参数/矩阵循环。全研究未完成，automationACTIVE，常规安静。


## 最新自动续做：15:29UTC轮次已完成

先完整阅读 reports/benchmark_route_stop_clearance_consumer_20261004.md。唯一规划clearance事件消费者实际35s E1运行15:37:11—15:40:50UTC退出0；协议15:37:02先封存，baseline原online_bank_hold运行不重跑。69CPU测试，独立审查诊断时效问题RED→GREEN，37新产物SHA/旧15与39清单验证，无遗留进程。相机、地图、PID、cap、旧停止radius、hold、latch和35s冻结。

20秒真实旧局部路径不兼容事件锁存新A*净距规则，1001物理采样/201完整实时执行状态与来源packet同prefix，200旧policy/request公共状态一致，raw起点0差。实际350请求/35maps/完整policy/1750执行tick重建精确，未来输入截断poison事件一致，349history移位一次。规划时刻字段明确时间/有效性，不假冒当前反馈。

候选拒绝：20—27.9秒80目标均改变，但主轴原样、副轴大多同+30边界。首请求差异22.5秒，仅5移动块副轴最大.27664deg；停车前XY分离peak仅.000579m。真正主要后果28秒取整当前格.7不满足规划.71159，70次无路径/零主轴，比原28.4veto早.4。veto66→0是上游停车，不是绕障；方向锁存low0，没有等待五次停稳。整条35s仍timeout/0contact，参考进度2.13017→2.08066，目标剩余4.23038→4.25965，主轴运动224→220。局部.05m增益/完整参考/导航都拒绝，第一米事件参考实际未完成。

保留并明确本轮gate设计缺陷：事件20—26秒B0原有veto0，预封存“严格少于B0”无法满足；不追改原protocol，不把该项失败当控制退步。其余有意义增益/整条指标独立失败，拒绝不依赖缺陷。不允许补第二次clearance/radius/cap/PID试验挽救结果。

下一项必须回到失败RGB层：复用已有完整对齐20pairs和同一冻结Fast预测，做一个有界离线因果实验，分解共同可见性、真伙伴纹理/光度残差与极线匹配歧义、过滤前错误/过滤后无覆盖。不重新采完整矩阵/推理全部网络，不换模型/相机/门槛，不把GT伙伴标签喂在线。先读 reports/benchmark_transaction_stereo_targeted_20261004.md 及其实际文件清单，保存测量工具/测试/真实数据/版本与图。执行消费者这一方向已用完，不继续停车或速度调参循环。完整RGB+估计位姿/学习记忆增量/原35sE0E1E2研究仍未完成，automationACTIVE，常规安静。


## 最新自动续做：16:29UTC轮次已完成

先完整阅读 reports/benchmark_stereo_visibility_and_matching_audit_20261004.md。本轮真实执行一次20对已归档RGB/同冻结Fast预测的离线诊断，新增推理/采集/仿真/控制均0。预封存16:35:37UTC，初次16:38、审查修正版16:46完成。78CPU测试通过；68旧输入与52旧产物哈希不变、24新文件身份保存，3797有限warp块的整块支持逐一核验。无待续实际进程。相机、旧gate、历史结果和35s冻结。

新事实：左眼合格障碍标签中，两眼共同可见比例e1仅45.07%、hard0004仅13.74%；hard65.84%伙伴画面外、19.44%右机体mask。地面共同可见16.54%/15.03%。旧共同区域gate逐像素/字段复算一致，hard原视差准确1.54%、最终.76%，保留3141个错误标签；e1过滤丢14794个原准确标签但未过滤灾难尾仍不准入。不能删mask、把画面外/未知写自由或称RGB导航成功。

4399确定性分层块样本：body弱纹理88.12%/95.28%，竞争匹配93.58%/100%；真伙伴光度相近不意味着唯一可配。同e1亦歧义却较准，不是已证明弱纹理唯一原因。整数常位移/曲面跨度/每帧cap及重复帧/小子组限制明确，不把样本比例当人口加权或独立统计。真正photo/texture干预尚未执行。

独立审查发现左整块mask遗漏影响72/15个地面warp描述块，已一轮RED→GREEN修复，原pre_review输出保留；完整左右支持/有限GT块重新核验，同支持分母比较，旧逐像素结论全部不变。新增非有限GT块排除影响57/20障碍warp描述块，已披露。photo warp也不认证每个右伙伴实际可见；旧只在已知平面验证同步、不能扩展所有球面/地面证据的限制保持。

下一项有界实际工作：复用hard0004原场景/mesh源码身份与已归档20pairs，对已知障碍的解析射线或真实mesh交点/图像轮廓独立核验，确定球面深度标签的坐标与时间适用范围。不能只再量一次相同GT重投影称独立。标签通过后才依据证据封存ONE纹理单因素诊断或ONE只读过去帧多视角支持检验，不同时改光/相机/网络/控制。不得再回停车净距/速度/PID扫描，不重跑整矩阵。RGB+估计位姿、学习记忆与原35秒E0E1E2仍未完成，automation保持ACTIVE，常规安静。


## 最新自动续做：17:31UTC轮次已完成

先完整阅读新增 `reports/benchmark_static_scene_geometry_and_depth_labels_20261004.md`。仅复用hard0004四对/八眼帧，离线读取原场景与子网格，无新仿真/推理/控制。86CPU测试通过，独立只读审查无Critical/Important，矩阵说明已修正。46项旧SHA检查与28新产物身份核验完成；原解析helper逐字节版本另存。没有待续进程，automation保持ACTIVE，全研究未完成。

原理想球面1cm gate仍拒绝（000R/010R）；唯一原始子网格分解在同完整分母上当前≤1cm左99.9005%/右96.1886%、p90约1e-6m，旧pretransaction仅47.53%/64.82%。这给出静态已知球表面的独立当前位姿/采集事务证据，厘米差主要由理想球面与网格表示解释。保留左73/右893大误差及020R/023R无支持，未认证全场景/边缘/遮挡/动态曝光。RGB网络旧gate不变、纯视觉仍未准入；不能把这项源几何诊断改称视觉导航成功。

下一项仅一个因果过去多视角可见性诊断：先冻结协议，复用历史归档实际图像/位姿/离线标签，区分当前缺失是否过去真实可见与过去RGB深度是否可靠。禁用未来帧/源几何在线输入/unknown转free；源网格只作受限离线标签。不同时改纹理或换网络，不再跑执行消费者/速度/PID/固定时间偏移扫描。诊断完成后再据证据选择一个来源可追溯视觉记忆消费实验。RGB+估计位姿、唯一学习增量、原35秒E0/E1/E2仍待完成。


## 最新自动续做：18:31UTC轮次已完成

先完整阅读 `reports/benchmark_causal_past_multiview_support_20261004.md`。仅四对已归档hard0004，73383完整已知球面查询；无仿真/推理/训练/控制。92CPU测试，独立只读审查无Critical/Important、全部实际字段独立重建/更强未来payload抛错测试通过。51项旧SHA检查、20项新身份与初次全部数组/指标一致，图例说明补齐；最终报告正文是作者自核，不冒充独立正文审查。没有待续进程，automationACTIVE，全研究未完成。

因果过去视野确实补几何可见性：当前双目8379→当前或过去24298/73383(11.42→33.11%)；当前65004缺失中15919过去双目、40982过去任一眼。020/023当前无右眼支持，过去仍6635/7235；000无历史不凭空冷启动。严格2x2/.01m可见、冻结RGB完整足迹/.10m可靠深度来源却当前/过去均0，有效模型源仍误差~.11–.35m。拒绝“直接融合现有Fast深度称可靠视觉记忆”；保留旧.76%单像素gate，不扩大0到所有像素/场景/世界模型。查询不是去重地图面积或全盲区，已知源几何只作离线标签。

下一项唯一候选：因果多视角RGB度量匹配，先做RGB+已知位姿诊断层。仅当前/严格过去双RGB、标定与当时已知相机位姿输入；不读源网格/renderer depth/真伙伴或未来帧。先写手算几何/缺视差/完整足迹/未来边界测试，并封存一个传统低维结构、搜索范围、置信/拒绝和四对历史预算，再实际验证来源质量；不无限换网络/扫阈值，不改灯/相机/控制，不重采矩阵。通过后才选择一个来源记忆消费验证，失败则拒绝结构和明确层次。RGB+估计位姿、唯一学习增量、原35秒E0/E1/E2仍待完成。


## 最新自动续做：19:31UTC轮次已完成

先完整阅读新增 `reports/benchmark_causal_multiview_rgb_plane_sweep_20261004.md`。唯一传统因果RGB+已知相机位姿平面扫描V1已实现，并实际推理历史hard0004四帧000/010/020/023；20项输入预封存，净化包无深度/源几何/真伙伴，全部预测先保存才独立评价。无新仿真/控制/网络训练；98CPU测试、一次独立只读审查无Critical/Important、56独立SHA和最终40旧SHA/40新身份通过。四帧推理CPU7.1/19.2/30.7/41.2s，不是实时在线。没有遗留运行，automationACTIVE，全研究未完成。

V1明确拒绝：完整73383独立源查询，仅796保留、43中心≤10cm（.0586%全分母），条件p90=1.15745m；严格2x2实际贡献来源可靠仅1，不可凭>旧0称可用。Fast同中心2832/6、p90=.34134，旧gate未改。真点在搜索范围内，但分辨率/纹理歧义/可用来源随深度改变等未各自隔离。事后真深度成本低的真实来源仍存在；580保留点的全部贡献来源真点可见，却547错，000单来源也150/167错，所以不能默认来源切换独自解释。旧inverse/floor整数投影舍入敏感性公开，不改原数组/gate；全5x5表面可见性仍不认证。

下一项只做同冻结结构的小型成本/有效来源曲线诊断：先封存独立于GT成绩的确定性锚点选择，最多32（每帧≤8）、同128深度格，当前/严格过去实际RGB+已知位姿；逐深度成本与texture/parallax/mask/FOV/有效来源位码先保存，之后才附离线标签。检查错winner、真深度附近和来源切换，特别包含000单来源歧义；不重跑四dense预测，不改描述子/网络/阈值/灯光/相机，不改控制。没有找到具体可修原因就明确否定该解释、归结这一观测结构限制，不能无限继续审计或用真值选来源。真实RGB来源、估计位姿、在线来源记忆+可执行局部规划、唯一学习增量与原35秒E0/E1/E2依旧待分层完成；不通知全部完成。


## 最新自动续做：20:31UTC轮次已完成

先完整阅读新增 `reports/benchmark_multiview_cost_support_trace_20261004.md`。同冻结V1的32锚点/128深度成本与来源曲线实际完成，每帧8个，16个空间栅格与16个旧保留输出栅格位置；选择不读GT成绩，但后一半依赖旧confidence，不能称随机总体或fresh验证。协议20:36:19UTC先封存21身份，原始曲线20:38:43全部落盘后才附独立标签；不重跑四帧dense预测，无新仿真/控制/网络/视频。104CPU测试，唯一独立只读审查无Critical/Important；最终20:54:04UTC核验61旧SHA与26新身份、32输出/成本/竞争宽度逐值完全复现。无遗留实际运行，automation保持ACTIVE，全研究未完成。

32锚点中17保留、16错；错点8个相邻来源切换、8个没有切换；11个真深度最近格仍有效且全部成本比错误winner高出原竞争带+.05，其中6个真深度/winner来源位码相同，12个所有贡献来源的真中心实际可见。000单当前右眼也有无切换错误（2.1048→2.3519m）。拒绝“只修来源切换就够”的单因果解释；不因此确定曝光、纹理或曲面近似根因，不增加成本扫描/阈值/另一候选。V1及旧RGB/导航gate保持拒绝。

审查Minor为 `Final: minor (deferred)`：010/020/023无效顶部参考锚点辅助texture值因边界clamp不同于原boxFilter零填充；全部reference_valid=false、无有效aggregate/winner/confidence，32输出、17保留点texture和结论不变。代码/曲线冻结，报告明确这些无效点辅助量不是精确V1中间量；误读可能把无效边界纹理当原因。未来新仪器才零填充或显式无效并加边界测试，本轮不修后重跑。最终验证器由作者执行，不冒充审查者检查；原审查正文SHA和追加说明范围保留。

下一项仅一个移动高对比编码板采集：独立开放平面、一条12秒前进—停止完整序列，固定当前相机/灯光/PID/smoother/单次capture事务，开头重置、内部不重置。复用静态CalibrationOnlyBoard思想，先封存非重复板几何/颜色角点布局、请求、有限采样和数值gate；用真实双RGB角点/特征与已知板几何独立核验移动曝光/当前与过去位姿，几何证据通过后才用同冻结V1作有限来源质量测量。板几何仅离线标签，不输入预测；这是标定场景干预，不改正式障碍材料/目标marker、不称导航成功。原静态六render矩阵不重做，本项新增移动RGB见证。一次有界采集后直接定位失败层，不无限纹理/网络/同步阈值扫描，也不回停车净距/PID循环。RGB+估计位姿、可靠在线来源记忆+可执行局部规划、唯一学习/世界模型增量与原35秒E0/E1/E2仍未完成。


## 最新自动续做：21:33UTC轮次已完成

先完整阅读新增 `reports/benchmark_moving_coded_board_rgb_pose_20261004.md`。一条实际12秒forward-stop独立开放平面编码板序列已完成：120请求/121capture/601物理采样，双目及第三视角各121帧10fps，17非raw物理字段与旧同序列逐值相同；raw初始化原点本轮减旧-.02s/-1步，未混称完全同原点。无内部物理/PID/smoother重置，相机/灯光/执行/正式35秒规则不改。第一次沙箱GPU启动在数据前失败已保留并自行解决，唯一实际采集exit0；采集前作者镜像fixture修复和V1快照保留、V2重新封存均早于数据，不是看分数改gate。

真实RGB检测先落盘后载入板标签。全部12眼/时间组原gate均因覆盖拒绝：420期待角点/356独立找到/356全部≤2px，覆盖84.76%<90%；条件RMS .620—.809px、p90 .794—1.065px。7—9秒当前位姿p90~.79—.96px优于前一capture~2.02—3.29px，支持这些移动RGB特征当前位姿对齐，不认证全表面/所有曝光。16漏检tag用真四边形做离线诊断后仍全部拒绝，黑边5—13格误分白色>原max2；这是光度破坏证据，不证明specular唯一原因，不将GTwarp当独立识别。六帧完整左右tag交集0不等于所有像素交集0。几何gate不通过，0次V1匹配；预封存72锚点预算<min100缺陷另行保留，不算V1性能失败。

111CPU测试通过，唯一独立只读审查无Critical/Important。两Minor仅文字坐标轴和10Hz路径统计已核对澄清，不变代码/数组/gate；审查正文SHA和后加段落范围明确。作者最终2026-10-04T22:03:53.631110+00:00核验43旧SHA、190新产物身份，审查未全面审查作者验证器。三图实际检查、视频与全部数据/版本/失败日志保留；没有待续实际进程，automation保持ACTIVE，整体研究仍未完成。

下一项仅一次板材质单因素受控A/B：同初态、同完整12秒请求、同36码/板布局、同相机/灯光/decoder/PID/capture，复用本轮默认材质B0；唯一新增明确低反射/高粗糙度板材质。采集前独立封存完整几何gate和可达到min样本的有限V1预算，不追改本轮。比较完整漏检分母、黑边光度与current/past误差；失败即拒绝这一材料解释，不再换灯/颜色/阈值无限循环。几何通过才准入同冻结V1来源测量；不改正式障碍材料，不拿板真值喂预测、不称导航成功。RGB+估计位姿、可靠在线来源记忆+可执行局部规划、唯一学习/世界模型增量及原35秒E0/E1/E2仍待分层完成。


## 最新自动续做：22:34UTC轮次已完成

先完整阅读新增 `reports/benchmark_coded_board_material_and_sparse_depth_20261004.md`。只执行一条12秒板材质对照，复用上轮默认材质B0；27源/190旧产物先封存，实际22:40:07—22:40:58UTC退出0，120请求/121capture-history/601physics/两视频各121帧10fps。板自身绑定显式低反射高粗糙度材质、仍读原uniform面色；相机/光源/曝光事务/图案/解码门槛/控制/正式场景不变。整个12秒所有19物理数组、请求、121状态/history及板几何逐值/字节相同，raw原点也同，不是只验证前缀。

本轮解决本例板的光度漏检：完整420期待/420识别/420≤2px、全部12组原几何gate通过，RMS .595—.767px、p90 .765—1.092px。上轮同16漏检全由独立RGB找回；离线真四边形标签黑边误分白格5—13→0。运动时当前位姿明显优于前一capture，形成具体移动RGB几何见证；不认证全表面/所有曝光或已修好原hard障碍。六帧左右完整tag交集仍0，不能说全像素无重叠。材质这一整体因素充分改善本例，不分拆证明默认着色器哪项物理机制。

几何通过后按事前独立预算144≥min100实际执行6次同冻结V1稀疏预测，全部先保存再读板平面标签。144查询132原始有限/35最终保留/33≤10cm，条件p90 .06932m；完整准确22.92%<90%仍拒绝来源准入，不能拿条件94.29%准确称导航可用。34保留用到过去来源，真深度最近离散格当前右支持25/144，非current-only消融/非闭环记忆增益。旧72预算缺陷与所有历史V1/Fast/RGB/navigation拒绝不追改，未对几何失败B0另跑V1，不算深度A/B提升。

115CPU测试通过，唯一独立只读审查无Critical/Important；全部物理/检测/144输出/来源/误差/真实USD绑定/视频/三图独立核验。Minor保留：冻结summary副本的静态matching=false/geometry失败占位文字会误导单独读取者，报告明确披露、stage_summary与V1实际manifest/assessment是权威状态；不改旧数组/门槛。报告原审查SHA保持962b39f84008ec487b15bf32b2fb4c669071c3ff945cbff50bbc0165bcd36a47。作者最终2026-10-04T22:50:49.510274+00:00核验213旧SHA/203新身份，作者验证器不冒充审查者全面检查。全部代码/数据/视频/测试/失败日志/SHA保存，无待续实际进程；automationACTIVE，全研究未完成。

下一项只实现一个因果RGB特征轨迹—几何三角化—来源记忆候选，先known-pose诊断。当前/严格过去双RGB实际描述子匹配，禁用编码ID/板布局/renderer深度/源真几何/未来真伙伴提供对应；冻结唯一结构、有限完整序列归组的数据与完整分母/门槛，无descriptor/网络/ratio扫描。先复用本板序列做实现诊断，再对原场景归档RGB验证，不能拿高对比材质板替代原hard结果；历史数据不称fresh。低基线/缺失/漂移保留未知，来源/时间/眼别/轨迹与置信显式保留，不seed脚下自由。来源不通过即拒绝结构；通过才做记忆冲突/过期与可执行局部规划消费，再隔离估计位姿及唯一学习/世界模型增量，最后原35秒E0/E1/E2。结束光源/材质/同步排队/成本扫描分支，不返回PID/停车/阈值循环，不通知全任务完成。


## 最新自动续做：23:35UTC轮次已完成

先完整阅读 `reports/benchmark_causal_feature_tracks_and_sparse_memory_20261004.md`。唯一冻结ORB→极线描述子匹配→已知位姿三角化→稀疏来源记忆候选，26归档双RGB对/52 current-history预测在23:42:19UTC全部保存后才读标签；无新仿真/训练/控制/视频。三历史完整采集序列归组但有限选帧，非完整10Hz流/fresh held-out。124CPU测试通过；一次独立只读审查无Critical/Important，独立重提取全部特征/52预测/未来payload陷阱/记忆、独立射线标签与误差/拒绝链/三图核对一致。作者最终2026-10-04T23:57:51.252990+00:00核验248去重旧SHA（51源/203旧产物重叠）与79新身份；作者验证器未被审查者运行，不混称全面背书。没有待续本阶段进程，automationACTIVE，全研究未完成。

来源准入全部拒绝：板current59/46→history343/318（全1536查询，20.703%准确，条件p90.084725m）；原E1 38/19→88/47（全1014，4.635%、p90.179543m，15保留无有效标签）；hard仅87查询<min100，1保留/0正确，p9011.003498m。历史正确query加301减1，非实体去重地图/导航增益。hard错误点几何门槛全过（敏感性.016727m/视差14.2108°/重投影.500016px），离线真中心相隔7.692849m；不能把重投影一致当正确对应保证。E1也有真中心近而重建偏差大的证据，非单一错伙伴解释。

记忆终态板330记录308正确、E173/33且15无标签、hard1错误旧点仍存；从未生成free/认证扫掠或接入动作。known pose诊断不是估计位姿。独立body标签数据不足、mask旋转方形不认证完整ORB滤波依赖，报告保留此限制而不删除坏样本。Minor deferred：NaN reference/provenance时间比较可能绕过因果检查；实测全有限、当前拒绝结论不变，未来复用须以RED→GREEN验证独立坏时间guard，旧候选和结果冻结。原审查正文SHA72bb508ab3212a608d440148eb2203bca8e0dc5abcabbc8070cfc7b1fa11644b，最终追加复核段是作者，不冒充二次审查。

下一项仅一个原场景固定主动观察动作fresh完整采集见证，最多18秒/一条预封存完整请求/逐0.1秒真实双RGB和曝光位姿。先核验实测短段初态与扫掠适用性，保持rig/原场景材质/PID/unknown规则，不用未知地图宣称安全；不适用则给具体阻塞不启动。旧ORB结构作为冻结回归见证保持拒绝，不能事后补帧/换ratio/描述子/网络；必要新输入guard只拒绝坏时间，不改变正常结果。验证运动是否能增加真实来源供给，不能把主动观察或更长停留当导航成功。结束材料/光源/成本/描述子扫描；RGB可靠消费、估计位姿、唯一学习/世界模型增量、原35秒E0/E1/E2仍未完成。全部新代码、数据、测试、来源/失败/图/SHA保存，常规推进保持安静。


## 00:36UTC original-scene active observation — final stage closure

完整报告 `reports/benchmark_original_scene_active_observation_20261005.md`。一次原 golden E1 的18秒完整采集，00:45:55.968560—00:47:12.551316UTC、退出0；180请求、181双目对、901物理采样、1081可用接触记录均无实际接触，两视频各181帧/10fps。相机、照明、材质、roll PID与执行链冻结。已知完整地图只作受控采集的二维扫掠前提，未成为视觉观测/自由区；这不是视觉决定动作或E1导航成功。

实际整段转弯和自身停车响应通过原运动门槛：XY peak .000072081m、六时域yaw最大.000694rad/bank .000632°；五个实际10Hz XY速度低于.05m/s的资格14.2秒，与原实测响应相同。同六秒前缀17非原始时钟字段301物理采样、61执行状态及历史逐值一致，raw原点相同。readonly报表初稿通配排除raw_漏检raw_command_target已保留；独立精确版本补齐17字段，所有点/标签/误差/运动/gate及其他字段完全不变，没有重采或重推理。

冻结ORB的362 current/history预测在00:48:04.371701UTC全部保存后才读离线标签；模型/gate采集前冻结，推理runner/367输入身份推理前封存。完整5531查询（5087有限renderer标签、444无标签）current 226保留/206正确、3.724%完整准确/p90 .044577m；history 568/512、9.257%/p90 .051520m。历史正确查询净增306，不是地图面积或导航增益。两臂原90%完整门槛均拒绝，不用91.150%/90.141%的条件比例宣布可用。终态249记忆记录、203有标签/200正确、46无标签/3错误、247旧查询；从未认证free。旧ORB、hard错误、V1/Fast/M2及所有历史结论保持。

128CPU测试通过，四个坏时间/整段停车支持/初态测试RED→GREEN。作者最终验证 2026-10-05T01:14:05.342714+00:00：479去重旧SHA、1131新产物身份，181双眼特征重提取、362预测/有限wrapper等价/未来payload替换、181记忆快照、180历史移位与17字段完整前缀核验。一次独立只读whole-stage审查及其实际范围/限制见 `active_observation_review.json`；初次作者核验把当时仍写出的stdout log按空文件散列，独立审查确认为Minor；初次JSON保留并明确此局限，最终改用已预留排除的final日志输出，重新散列已完成的初次log。候选/数据/验证器未修改，最终完整身份逐项匹配；最终log在文档同步记录单独封存。不将作者检查冒充审查者执行。四张实际图作者已检查，六事件大图有缩放，另检查原尺寸两事件图。最终报告SHA `dda3ef8b69fa3d54e656b704f57b83dc2727b36dbe62a3584670383ed6254a65`。

这是预封存后的完整新采集，但环境/初态来自已使用E1，不是新的独立环境泛化。known-pose诊断不是估计位姿；nearest-pixel renderer深度/亚像素射线、边界混合、body-ID不足与ORB完整滤波支撑限制保留。未新增全robot资产或DOF/gain读回审计，不将此单一初态响应迁移作通用安全证书。没有新导航率、学习/世界模型或原35秒E0/E1/E2结果。阶段实际进程均结束；全研究未完成、automation ACTIVE，常规不发完成通知。

下一项只做同181帧的只读“稀疏来源地图→完整前进与停车扫掠支持”诊断shadow。固定同一实测参考及完整181帧；current/history同口径，unknown/当前/历史/predicted与来源/位姿层次明确，不重采动作、不换网络/描述子/阈值、不用真值过滤或seed脚下free。这些181帧的标签已被本阶段评价使用，消费者设计继承此知识，只能称历史开发诊断，不能称fresh held-out。先冻结消费者语义/预算及仅观测输入，消费者输出保存后才重新读取标签作分层核验；明确全查询比例不能替代任务空间面积。若本段支持仍不足，给出具体冷启动/观测任务条件阻塞并关闭这一分支，避免无限局部审计。拒绝候选shadow不获得控制准入；只有来源与消费者各自验证通过才改变动作，再验证估计位姿、唯一学习增量和原35秒评测。


## 01:38UTC endpoint support shadow — final stage closure

先完整阅读 `reports/benchmark_point_support_shadow_20261005.md`。一次真实历史回放181帧/362地图；190点/代码/参考身份在01:47:24.005734UTC封存，全部输出01:47:26.673320UTC保存后才重读离线标签。没有重采、重推理、训练、改动作或新视频；旧实际18秒双目视频作为来源。两地图都接收完全相同的causal当前query估计，current无旧query记录、history10秒原query时间；current不等于单帧双目，既往深度增益不混作本轮地图记忆效果。

实际568保留点全部world-Z .936991—1.015382m、nominal floor0；height/uncertainty筛选前，也没有任何点的nominal XY进入完整扫掠333格。起点187/转弯300/自身匹配停车225/whole333格在每臂每181时刻全部未知，所有phase0次通过，floor格全0。6秒开始运动前仍无起点支持；18秒事后249记忆也未补足。strict unknown-blocked语义下消费者拒绝，不能用已经走过/无接触回填观测free或调半径/放大稀疏点。

history在6秒252记录/248正确/4错误；末态249记录/203有标签200正确/46无标签3错误247旧query。hazard代理峰值20末态17，current峰值4末态1；history与旧occupancy自由格重叠峰值11末态10只是2D/高度/跨格代理，不自动等于真实错误物体。false-free0仅因floor空集合，不是安全通过。no ray-carving/interpolation/bodyseed/model-imputed-area；sigma只是扰动敏感性/knownpose_sigma0层；floor代理不是全体积认证，source旧拒绝保持。

六个literal消费fixture RED6→GREEN6；文档完整身份fixture另RED1→GREEN，最终135CPU测试通过（实际日志保存）。作者最终 2026-10-05T02:13:11.325621+00:00：1248去重旧/预封存身份，362地图重建、181packet/旧有效FeatureMemory点等价/未来payload与截断，4冻结raster几何+4独立continuousprojection格集合、1448直接map索引分类。原计划上轮append closing使全文SHA变，原采集前prefixSHA75204b3a…和recorded closure SHAda551acb…都逐字节验证并归档，不略过任意变更；所有其他旧源/数据/报告匹配。新增fixture曾并入封存测试文件，已精确恢复原6fixture、另文件保存新1fixture/合并中间版；首次split缺pytest的失败日志保存，最终135 green。作者一次重复1448geometry核验被主动中断后改为4geometry+1448map-index，仅核验进程、非实验中断，不少查数据。所有失败与版本保存，模型/mapper/runner/190协议/362地图/数据/gate未改，没有实验重跑。

一次fresh-context只读审查范围和发现见 `point_support_review.json`，作者复核不冒充其执行。两图实际检查；所有输出日志另封存，避免循环自散列。报告最终SHA `14bd954cff7172e9f0cde720f6fcd715528e803248d0ff054289dcb30e9e6b42`；最终579新产物身份和文档同步单独保存。任务阶段完成、全研究未完成，automation ACTIVE，常规不发完成通知。

关闭“冻结ORB+这条主动观察+保守endpoint支持解决严格冷启动”分支；没有近场floor/扫掠支持是具体阻塞，不能推出所有像素/所有方法/固定rig永久不可导航。停止追加动作、sigma/半径/TTL/网络扫描和同一局部诊断循环。

下一项回到完整控制/在线系统基线：先检查是否已有相同当前rig与控制身份的完整Oracle结果，存在则复用，不重做矩阵；核验 `H/reusable_oracle_audit.json` 及 `experiments/oracle_feasible_route_20260929_v9/hybrid/` 的实际class/继承/act/PID/协议/环境身份。旧wrapper9同SHA、旧core两份变化/旧environment缺rig，不能由名称继承旧3/3、5/7。必要时最多一条原35秒完整golden E1的已知地图Oracle诊断，同时真正在线运行仅当前/过去RGB+本层knownpose的只读来源shadow；参考、增益、速度配置保留旧成功控制器，不将其当原InformationPolicy的单层修复或同时调参。shadow不改B0动作，全图/全局参考不得写进RGB观测地图，旧source/consumer gate不解除。保存完整路径/真实到达/接触/时延/图像/执行/身份，不报告纯RGB或正式E0/E1/E2矩阵成功率。Oracle成功只说明控制上限，不能替代视觉基线；若要safe-start/已知地面先验必须单列prior任务条件，禁止冒充观测。估计位姿/唯一学习增量/原35秒正式分层结果仍按总计划，未完成不宣称完成。


## 02:39UTC current-rig Oracle success / live RGB shadow — final stage closure

先完整读 `reports/benchmark_oracle_online_rgb_shadow_20261005.md` 及本轮冻结计划 `docs/superpowers/plans/2026-10-05-oracle-online-rgb-shadow.md`。ONE真实原35s goldenE1 Oracle完整导航在02:49:22.233268—02:52:07.139341UTC退出0，30.7s目标entry、距离.1954856m、实际1843接触样本0；307动作/308双目capture/1536physics，两视频各308帧10fps无legacy5尾步。全研究未完成、automation ACTIVE。

205旧environment没有录入当前rig的Oracle；实际旧ConfidentSpeed→MappedBankOracle→OraclePathPolicy(MRO/act源码身份明确)，BankMap/P040D070 PID(.4,.1,.7)、auxsupervisor.3、lookahead.5原样。预sealed58源/配置/精确缓存旧hybrid已选30waypoint(8.7m/0reverse/483展开节点)参考，不重跑hybrid完整搜索/修改planner，不以未来实际轨迹作输入。当前rig固定ball_stereo_front020_up005_v2/原照明/采集事务/hidden markers。307actions/308XY以及11旧共有状态/执行/roll/velocity/clock遥测数组EXACT历史已归档成功，raw起点也同；旧rig未记录/不宣称同RGB/不是控制候选独立A/B。独立参考进度8.657741m、横向RMS.089963/peak.233021，真实路径8.835802；arc声明8.7、折线8.688761、runnerA*评价8.836753分开。Goalentry成功不是末端停稳保持。

Genuinely LIVE RGB shadow每次capture之后/下一act前都真实执行冻结ORB+last3pastpairs、10s原query endpoint map，共308；只允许frame/time/双方RGB/knowncamera poses/K，拒绝map/reference/depth/goal/actions，复制隔离。不读futuretruth/实际futureapplied/supervisor。Oracle完整地图/参考ONLY控制基线，不写RGB观测，knownpose不冒充estimatedpose。无floorprior/raycarving/interpolation/GTfilter/predictedfree/动作准入。

全28120ORBquery/24742有限label/3378nolabel，current756保留426正确1.515%p90.167196；causal2044/1096正确3.898%p90.170100/1419past。BOTH旧90%/.10m必要gate仍REJECT；旧source/consumer拒绝不倒改。末态917记录688labelled352correct336wrong229nolabel912old；记录≠实体/面积。Floor格all308为0，hazproxy峰98/end81，整冻结1521格sweepminunknown1505/end1517unknown4hazard，0supportpass；只是固定空间诊断/不要求提前看见全路准入，仍无floorendpoint。有限nearestpixel/subpixelray/bodyID/fullpyramid/2.5D限制保留。Featurep50.04936/p95.16041/capturepartialp50.3188/p95.44927s，因果在线≠wall10Hz/完整端到端。

五fixtureRED5→GREEN，140CPUtests2.76s(独立审查实际测试另见reviewJSON)。作者full2026-10-05T02:58:19.550166+00:0058源+2478初始pre-close输出SHA（并非全部关闭后输出）、308feature/pred/map真实重建、307实际同输入Oracle on/off/request/diag/mapping EXACT、3真实cold/mid/finalRGBprefix futurepayload-trap/truncate；实际时钟0age/markershidden/无尾步。历史579旧产物身份最终匹配，所有科学源/协议/数据/gate未经审查后改变；最终轻量identity closure仅封存报告/审查/日志，避免无理由重复科学重算。初始manifest在runtime.close前保存，继承close之后才写bank_feedback，故它不含该文件；独立审查Minor记录此范围缺口，冻结runner/manifest保留，post-close supplement保存其SHA c3c56ea6ad00be94c6d4abf597b2eebdbccb30532c0b0c2757665987dde89ea2，作者2508-entry核验已包含它，最终identity再封存。不得把初始2478称为关闭后全部文件。审查范围/发现/declined rulings见 `oracle_rgb_shadow_review.json`；审查正文SHA `2093868fe8fd5ef3b114007207a17b95d6d9e7faf50500b7ab9f009d22c10cbf`，之后作者段落分开，最终报告SHA `a06e2c76256860e756904fc79257c5d2f1be5042c218df1ad9e39df7b53e5d44`。封存工具最初只碰到新stage ledger追加、原prefix精确归档；生成nestedclass缩进parse错在数据前修复/初稿保存，唯一真实仿真不重跑。新stage计划被protocol封存保持原字节，不追加closing破坏旧输入身份。

控制基线已复现成功；冻结ORB/endpoint冷启动分支关闭，视觉与估计位姿/世界模型未准入。下一项独立特权depth+knownpose局部来源记忆控制基线：复用本次成功曲率/roll跟踪组件，局部参考来自当前/过去来源map，全图/全局Oracle路线不得进入局部planner；全球goal允许。先CPU核验真实class/局部参考/曲率/请求链，事前封存一个完整package与原35s、同初态，最多一条E1。保留明确原unknown-risk任务条件/unknown标签，不seed脚下free、不冒称safe；不称InformationPolicy单层修复或纯RGB，不调PID/速度阈值/旧stopradius，旧异状态有限响应不强拼。该上限通过后再按任务条件回到RGB可靠障碍消费/estimatedpose/唯一学习增量，仍不启动正式E0/E1/E2矩阵或宣称总研究完成。若采用flat-ground/safe-start prior必须单列prior任务而非观测。常规安静，automation ACTIVE。


## 03:39UTC privileged depth local curve — final stage closure

先完整读 `reports/benchmark_depth_local_curve_navigation_20261005.md` 和本轮冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-depth-local-curve*.md`。本阶段ONE真实原35s goldenE1已结束03:52:31.736585—03:55:56.221204UTC、退出0；没有待续仿真或第二条运行。接口通过、导航REJECT(timeout35/4.486646m/路程5.890657m/实际2101可用接触样本0)。350动作/351双方capture/1751physics、两视频各351帧10fps/无5s视频尾步。全研究未完成，automationACTIVE，不能作为完整结果通知。

独立privileged rendererdepth+knownpose来源地图→局部target2.4m/crop±4m/原HybridA*node3000→ConfidentSpeed/MappedBankOracle/OraclePathPolicy→原rollPID(.4,.1,.7)、lookahead.5、auxsupervisor.3。外层privilegedFalse只拒绝runner全局Oracle输入，传感仍privileged；全图/全局参考不得入外层，全球goal允许。Unknown labels独立，规划副本风险探索不认证安全，无footprintseed/raycarving/GTfilter/futureinputs。初次局部reference初始化，后续仅替换几何，五低速/反弹/方向/历史/smoother状态保持。冻结Oracle .5/1/1.5主轴域与旧InformationPolicy .3 unknowncap不同，且6s零请求前缀计35s，不能单因素A/B归因或冒称RGB/完整3D认证。72source于03:52:11.003572UTC preseal后保持；post-close2762outputs覆盖bankfeedback，初始manifest只pre-close。

58plansALLsuccess；localtarget world-y符号7.5s(-1.4→+1.4)、16s(+2.2→-2.0)两次变动，与来源hazard变化关联，未隔离单个格因果或形式拓扑。仅一次请求forward→reverse16s，actual18.5s，原合理制动2.5s；不称方向反复/resetbug。265有效movingtracking，protocolclip186(70.19%)、bankcurvatureclamp183、V1outside89(33.58%)。独立旧评价参考finalprogress4.167298/RMS.497267/peak1.114722m，不作为当前planner输入或纯trackingerror。0.5s重新起始参考exitRMS.027668/peak.080303不能证长期成功；保存冻结5s描述窗口，无另一计划实际label冒用。当前/历史floor31/1715、hazard63/163、unknown158028；depth层有来源floor，不改变旧ORBfloor0/RGBsourceconsumer REJECT结论。planp50.797596/p951.243388s，policy p95.940341/capturepartialp95.297808；causalonline≠wall10Hz。

148CPUtests4.39s；作者full2026-10-05T03:57:09.067008+00:0072source+2762postcloseSHA、351双方depthpacket、350真实请求/地图/局部reference/historyshift精确重建，3finitepaststate clones futurepayload/truncation一致；不是3完整freshphysicsruns。全部1750raw physics请求对齐，实测feedback执行链1745ticks重建一致，首5ticks不克隆/不是递归物理预测。初次assessment pending保留，随后verification interfacePASS/navREJECT为当前状态。预数据fixtureV1aux0非理想直行、boundaryhazard首次未RED、seal terminalnewline/initialcapture-reset调用序问题均如实归档，全部封存前解决；没有数据后调参/重复候选。ONE独立只读review范围/发现/declined rulings见depth_local_curve_review.json；原审查正文SHA 94d7cdbfdfba68f8913f5a6606720503ef975624240730c2a362dfff24557e3d，最终作者报告SHA f2cb7daf73773957a910b2736f76cafc7bea365c2a2dbf08359e37fa82495596，后续作者append分开。最终轻量identity只封存/复查现有完整科学输出，旧2513SHA与新72源一致，不重做矩阵。

接续尚未执行：从本次完整58map/reference数据先核验合法参考连续性候选，已有走廊仍受观测支持时稳定reference消费；真实新hazard失效允许明确重建，不锁死绕行/强制方向。仅一个参考消费修改，经CPU几何证据/预sealed独立进度、空间检查点、目标、接触gate后，最多一条同起点/同depth-risk-domain/同6s前缀/同原35s候选对本次B0。保持planner算法、PID、speed、mapping、rig/oldgates，候选不合理则拒绝；本heartbeat不再仿真。局部层未通过，RGB+估计位姿/唯一学习增量/正式35sE0E1E2未触发。常规安静，ACTIVE。


## 04:41UTC reference continuity causal audit — final stage closure

先完整读 `reports/benchmark_reference_continuity_causal_audit_20261005.md`、`docs/superpowers/plans/2026-10-05-reference-continuity-audit.md` 及对应spec。ONE归档几何实验完成04:50:01.698168—04:50:02.974850UTC退出0，协议04:50:01.580440封存77源/2798旧输入；58replan/首N_A+57旧局部段/current+installation114几何；仅8gridA*queries两事件75/160(prior/currentmap×XY)，无额外search/Hybrid/sim/推理/训练/控制/新视频。原35s timeout4.486646m/所有RGB/M2/停止/导航gate保持，全研究NOTcomplete/automationACTIVE。

新观测地图足以使目标y两次切侧，保留旧图只改当前XY不切侧。新hazard3/2格使旧WHOLEgridroute各10顶点原.6栅格失效/min.4，故不能判定无理由planner抖动。但75时旧INSTALLEDprefix剩2.313414/误差.045706/continuous1.027154/raster1m仍有效；160prefix剩2.316381/误差.003219/raster.6通过，continuouspolylineproxy.567948不通过且155安装时已如此，不是新增hazard破坏这一prefix。连续polyline/rint格点/nominalprimitive/实际动态层次分开，不宣称源实现bug/真扫掠危险。57交接44necessaryscreen通过，7continuous不足/6raster不足不重叠，无lookahead/projection拒绝。预declared两个switch均可保留的提议REJECTED；无无条件固定侧/无删除检查/调整.6后跑物理。几何输入干预仅隔离目标侧选择，不隔离35s超时贡献/另一计划真实label。

六literalfixturemissingmodule→RED6callables→GREEN，154全Hsuitetests4.39s。初作者cwd+relativepath写入全部失败、file-not-foundlog保留，之后正确路径创建；旧源/数据无改。作者verify2026-10-05T04:52:36.920731+00:0077源2798旧输入/114scalargeometry/5809brutenearestcentre+paddedraster/4实际原route-target对角精确，无额外search。报告/源码/协议/58rows/8完整NPZ路线/图/旧实际视频引用/所有log保留；初始assessment不倒改。ONE freshreadonlyreview与每条declined scope裁决见reference_continuity_review.json；原bodySHAcc7a132a8e672b003456dc16b9a6e40d468de232a1c2db5373e843757b60a0b6，作者最终报告SHA52e64cc9788e4c72630d86eb4f4cffab0c2d8d52cca72bbaffe0fdda7caf14c2；作者append不冒称二次审查，最终identity/docsync单独核验。

接续单一控制工作尚未实现：只在旧局部suffix+currentbridge两检查通过、remaining>=.5/projection<=.5时保留reference；失效或耗尽则交回原planner重建，允许合法换向/原五低速反弹，不强制选侧或等无法启动条件。此为有有效性guard的新候选，不是把本次拒绝的两个switch一律保留筛选改为通过。先新版本TDD验证真实内层索引/方向历史/smoother状态和hazard/耗尽重建；预封存唯一reference替换时机改动/同depth-knownpose风险域/同6s前缀/同35s，最多一条同起点goldenE1对归档B0，其他planner算法/PID/速度/映射/rig/gates不改。真目标到达/独立进度/检查点/接触验收，不能停慢或跳index称成功。动态安全仍未认证，不继续新几何矩阵/网络/PID/阈值扫描。RGB可靠消费/估计位姿/唯一学习增量/正式35sE0E1E2仍待验证；常规安静/ACTIVE。


## 05:42UTC guarded reference retention — final stage closure

先完整读 `reports/benchmark_guarded_reference_retention_20261005.md` 和冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-guarded-reference-retention*.md`。ONE真实原35s goldenE1已结束05:51:21.407818—05:54:11.093362UTC、退出0，无待续仿真或第二条运行。仅参考替换时机改变；保持原planner/goal/map/rollPID/speed/rig/光照/五低速确认/反弹清零。接口PASS、事前限定改善PASS、导航REJECT(timeout35s/目标剩4.156980m/0实际接触)。全研究NOTcomplete/automationACTIVE，常规安静。

86源/2834旧输入05:51:05.605849封存；350请求/351双方capture/1751physics/2101接触样本，两视频351帧10fps/无尾步。58边界50保留8重建(6success/2failure)，旧suffix/current bridge .6连续和.6栅格、remaining>=.5/projection<=.5，只在原.5s边界检验，不认证动态/3D/unknown安全。首次动作分叉69/6.9s，此前69请求346physics17非raw字段/70完整execution+packet-history EXACT；raw起点-.02s/-1tick分开。249请求block/1393appliedticks实际改变，绝非raw规范化等价。独立冻结评价reference进度4.167298→4.855576m，目标距离4.486646→4.156980，RMS.497267→.174067/peak1.114722→.475266m；2/4m检查点24.9→13.9和34.2→25.3s，6/8未通过，不把索引/短local误差当导航成功。

原真实方向状态17.5—23s经历3.8s制动及1.7s反向跟踪，随后26—35s沿当前local曲线增1.397859m/RMS.045075/peak.102368，但main+.5/83.333%请求裁剪，独立参考偏差仍上升。moving全224帧clip54.464%，V1outside45.089%比旧33.585%更高；不称全链恢复/不把剩余失败全归PID。nominal planner根同时forward/reverse成本0且输入未含实际direction，是初态与执行接口待隔离假设，不是已证明bug或修复。knownpose+真实rendererdepth仍特权诊断层；所有RGBsource/consumer/M2/旧拒绝保持，estimatedpose/学习增量/正式E0E1E2未准入。

162全HCPUtests6.40s，7feature RED+1旧cadence回归；初GREEN fixture误把no-route rebuild当已安装时间，数据前仅修fixture，完整失败log保留。作者full 2026-10-05T05:55:08.554643+00:00 86source/2752post-close SHA，351bilateral packet、350causal地图/reference/request/diag/history重建，1745实测feedback执行链一致(首5ticks未克隆，非递归物理预测)，3finite-past checkpoints futurepayload-trap/truncate(非3完整freshphysicsruns)。独立review仅一次，范围/发现与逐条declined裁决见guarded_reference_review.json；原bodySHA a5f21c0d1f11dad7b9ec56bd20ad2c556bd7abf7208ac7267f9857dfc4540202，最终作者报告SHA 4dba7eecdaf6ca77a468f3552171f86c55c06197cf62bc471534272d2108d0a3，追加作者段不冒称再审查。最终closure不重跑科学矩阵；旧源/数据/旧gate不改，归档B0对照不是独立fresh环境泛化证据。

下一项尚未执行的有界任务：对本次17.5s冻结输入做一个根状态成本反事实，比较把原有.5m direction-switch成本应用于当时真实accepted executed direction的首次primitive是否改变选择与可行性。先独立版本TDD/同map/同goal/同原primitive代价和nodebudget，允许必要reverse，不强制forward/不扫描新惩罚/不改PID速度阈值/不再次重做矩阵；归档几何不冒用另一计划真实运动标签。有合理且真实改变的证据才另外封存唯一35s运行；本heartbeat不再新增search/物理序列。来源可靠性、估计位姿和唯一学习记忆增量仍待分层研究，未宣称整个任务完成。


## 06:44UTC current-direction root cost — final stage closure

先完整读 `reports/benchmark_direction_root_cost_audit_20261005.md` 与独立spec/plan `docs/superpowers/{specs,plans}/2026-10-05-direction-root-cost*.md`。一次归档frame175/17.5s名义几何对照完成06:55:35.435283—06:55:41.269461UTC退出0，preseal06:55:21.321515UTC94源/5630输入。仅1原grid+2Hybrid，同当前地图/pointgoal/XY/yaw、原node3000/.3primitive/.05sampling/.2goal、原reverse1.25/switch.5，不改实际动作/物理/rig/光照/mapper/PID/速度/guard/five-low/rebound。没有新仿真、推理、训练或视频，旧实际视频仅context，不能作候选真实标签。全研究NOTcomplete/automationACTIVE。

原planner完整1821节点/route/target/pose/direction/command EXACT归档。当前acceptedforward/pendingNone，与当时appliedmain+.7、mainjoint实测+.876695/XY纵向+.339162一致；当前afterplanningdesiredreverse和零请求direction_change_braking单独report-only，预测snapshot禁止它们/未来输入。独立copiedplanner仅机械import、initialenum参数/验证、rootg+priority改变，两根仍可选。候选2434节点success首R→F，path3.3→3.0，统一current-directionobjective4.375→4.075，终点误差.174093→.018737。原内部换向1→2，但包括初始交接两者均2；拒绝消除换向/已降低制动耗时解释，不把名义曲线称真实轨迹或导航改善。

作者2026-10-05T06:59:34.658791+00:00 无search验证94源5630输入/字段defaults/当前来源/严格未来payload，21primitive147采样端点<1e-15/原inflatedraster/终点.2均过。Primitive栅格min .632456→.6，当前整polyline连续min .642272→.614018，原guard栅格min .583095FAIL→.6PASS，层次不同/未知风险和动态3D未认证。接口PASS与预declarednominaleligibilityPASS仅给未来独立单次控制对照的必要证据；旧原35s超时距goal4.156980/0contact保持，全部RGB/sourceconsumer/M2拒绝不改，estimatedpose/学习增量/正式E0E1E2未准入。

16真实fixturecallableRED1.18→GREEN1.36s，178全Htests7.01s。作者两次cwd+relativepath写入全失败且无文件创建、缺文件logs保留，随后真正一次科学对照，不隐藏failure/不重搜；机械import用于独立副本/旧defaults原样。ONE只读独立review/每条declined范围裁决见direction_root_cost_review.json；审查bodySHA 71400a7d86431fd5e4212b0528d31ed8cad44f4053f990b7af4cd53d5a8fcd92，最终作者报告SHA dc7cfcdc7c0684bbef9e452b51e5b15da52426db175039fbee91f5ed1f68b017；作者append/封存另核验，不冒称再审查。全部代码、两完整参考、当前快照、协议、数据/测试/图/日志/来源与旧身份保存，不做无关Gitcommit/delete。

接续一项尚未执行的传统控制候选：仅在原guarded-reference policy每次正常rebuild向新planner传当前accepted-direction并应用原.5rootcost，retain不搜索/reset，不按episode/time触发、允许必要reverse。先CPU核验真实class/request类型转换/状态/五次与反弹及保持原guard来源；新协议事前封存最多一条同起点/同完整任务/原35s goldenE1对归档guarded B0，真实目标/独立进度/横向/空间检查点/接触/动作链验收，失败直接拒绝，无penalty/nodebudget/goal距离/速度扫描。该候选仍有2次换向，真实效果不能由当前几何gate代替。本heartbeat不新增控制run/更多search；可靠RGB、估计位姿和唯一学习记忆/世界模型仍待分层研究，全研究继续ACTIVE。


## 07:44UTC current-direction root cost online — final stage closure

先完整读 `reports/benchmark_root_direction_online_navigation_20261005.md` 和冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-root-direction-online*.md`。ONE同起点goldenE1实际原35s于07:54:40.229131—07:57:32.904552UTC结束exit0，无遗留仿真/第二候选，归档guarded B0不重跑。接口PASS，事前限定改善REJECT，导航REJECT(timeout35s/goalremain3.671378/0contact)。全研究NOTcomplete/automationACTIVE，常规安静，不替换已归档B0。

只有原guard rebuild向独立planner传PRE-planning accepted direction并应用旧.5rootcost；初次None，后11次forward；retention不搜索/reset。原reverse1.25/node3000/primitive.3/.05sampling/goal.2/localtarget2.4/crop±4/clearance.6、PID/speed/rig/light/协议/smoother/五低速反弹均冻结。真实depth+knownpose显式unknownrisk，不是RGB/safe/estimatedpose，外层privilegedFalse只拒绝全图/全局Oracle在线输入。无footprintfree/raycarving/GTfilter/futuretruth/新模型或参数扫描。

106源/5670旧输入07:54:23.300306 presealed，350请求351双方capture1751physics2101可用接触0，两视频351帧10fps无尾步/markershidden/0age。关闭后2762manifest含bankfeedback，原manifest pre-close范围单列。首分叉175/17.5s，175request876physics17nonraw/176execution+packet history EXACT，raworigin+.02s/+1tick初始化差异独立；116rawblocks738appliedticks改变。实际175 reference EXACT先前名义candidate，58boundaries46retain12rebuildALLsuccess，source类型/安装/继承act/reset和retention状态明确。独立评价reference进度4.855576→5.389775(+.534199)，goalremain4.156980→3.671378(-.485602)，但RMS.174067→.572390/peak.475266→1.105993，故预gate拒绝；4m空间点25.3→19.7s，6/8未实际通过，不把索引或停车当成功。

只读完整日志分段定位继承projection接口：frame181idx0F→idx1R pendingR/low0，182在accepted仍F/未资格时窗口放行下一forward段→idx3F、pending取消；version3未替换。17.5→23 raw/applied/mainjoint均无negative，最低applied+.45、mainjoint+.418757/root纵向速度+.161221，短reverse未实际执行。候选brake.1s不能解释为比B0合法3.8s换向更好。同期自己冻结version3 RMS.260636/peak.510890偏离；后来稳定local23→27/30.5→35 RMS.035565/.022107，但独立全参考末态偏离1.086392，不能把全部失败归首次交接/PID，也不能以local小误差认证导航。

六feature RED1.81→GREEN2.53/184全Htests8.45s。作者2026-10-05T07:58:39.955951+00:00 106源/5670旧输入/2762产物SHA、350causal policy/map/reference/request/diag、351双方depthpacket、350单次history shift、1745实测feedback执行链精确，首5ticks不克隆/非递归物理预测；3finite-past checkpoints futuretrap/truncate一致不是3完整freshphysicsruns。初assessment pending保留，后verification接口PASS/导航REJECT，旧gate不改。ONE独立只读review及每条declined裁决见root_direction_review.json；审查bodySHA 9da9130cd1ff5cc20d48512f8763cd086cb1cde82f1e1dad0b8d22d5dc0b37a3，作者最终报告SHA 5cce76e114febecf087c74deeebcb22506119fc8ba3a1e2d27843cde918d8b36，作者append/最终hashcheck不冒称二次审查。全部实际代码/数据/日志/视频/版本保留，无无关Gitcommit/delete。

接续尚未实施的一项工作：CPU精确复现未资格换向时投影越过短reverse段，核验waypoint出段/accepted/pending/窗口调用顺序；只设计一个最小投影方向边界修复，保持原五次低速/反弹/mapper/PID/speed/planner/rig/gates，不强制提前反转或循环等待。真实fixture验证/身份与预sealed独立进度/横向/目标/contact gate后，才在后续独立阶段考虑最多一条同初态35s对照，本heartbeat不新增仿真/搜索/调参。所有RGB/sourceconsumer/M2/旧停止包络拒绝保留；估计位姿/唯一学习记忆世界模型/正式35s E0E1E2未准入，全研究继续ACTIVE。


## 08:44UTC unaccepted cusp projection regression — final stage closure

先完整读 `reports/benchmark_cusp_projection_regression_20261005.md` 和H/cusp_projection_protocol.json/assessment/verification/review。上一轮全部来源/产物/文档身份保持，无遗留实际仿真，本阶段只有CPU最小修复+一次归档两时刻对照，无新sim/grid/Hybrid/训练/视频/成功率。Projection regression PASS；实际闭环/反向段完成/导航尚未验证，原35s timeout3.671378/RMS.572390/peak1.105993/0contact以及rootcandidate拒绝保留，较强guarded B0未替换。全研究NOTcomplete/automationACTIVE，常规安静。

唯一修复：已知当前reference方向!=accepted时保持当前idx并返回真实距离，否则调用原projection。原act/handler/five-low .05m/s五10Hz与反弹、速度/PID/BankMap/协议/smoother/planner/rootcost/guard/相机照明保持。正常初始化无pending仍能进入反向交接，资格后恢复原投影；合理新reference可取消过时pending，不永久锁旧路线/不强制提前反转。独立with_cusp_boundary基类仅覆写project，make_cusp_oracle保持actualwrapper组成，CuspProjectionPolicy继承RootDirectionPolicy仅episode reset装配，不action时reset。非法accepted投影拒绝；旧rootplanner入口None Minor仍未修，不把这一层检查冒称全部入口修复。

11真实fixture初callablebaseline7failed4passed RED1.25→11GREEN1.23/fullH195PASS8.72s；双向短F/R/F复现、实际act五低速/反弹/首次无pending/资格release、正常同向完整act/raw/protocol/diag精确、合法geometry replacement/实际MRO/非法None均覆盖，无planner或sim。109源8487旧/case输入于08:53:56.762833 presealed，一次2arms×2currentcalls08:53:58.904760→08:53:58.951319exit0。初态frame180过去9policy资格字段+121pastmapping、175已知参考、181/182当前观测；不是fullphysics/PID/smootherclone。原臂两请求/diag EXACT，181共同状态/动作同，182实际速度.231750候选保持idx1/pendingR/low0/zero，原idx3/pendingNone/main+.5/auxraw-.582204 protocol-.523599。分叉182即结束，不使用183+旧实际轨迹当候选labels；只是raw/protocol改变，不claimapplied/停稳/实际reverse完成。

作者2026-10-05T08:55:18.621803+00:00 hash/scalar/chronology/source检查109/8487、4calls/两原请求/共同前缀/新索引层语义与assessmentSHA；没有再act/仿真。决策图是已知参考和共同actualXY，视觉已检查，无候选后续运动。ONEfreshreadonlyreview范围/发现/每条declined裁决见cusp_projection_review.json；审查bodySHA 371ca02bcca7f03dfb6e82b6e4f6b0d85538df0b9470bb1a2123102e846d6122、作者最终报告SHA 91c88f066c63ff2a7146837c071c997d95a19b83ac11e98d25aedc285f419522；作者append/finalchecker单独归属未二次review。全部源码/case/日志/图/协议保留，无无关Gitcommit/delete；旧sourcegate/M2/RGB/stop拒绝与rig不变。

接续尚未实施的一项实际工作：预封存唯一CuspProjectionPolicy同初态/同6s前缀/原35s goldenE1，对归档root_direction_run仅projection改动因果比较，并同时报告较强guarded B0；胜过更差比较器不是baseline准入。原rootcost/geometry/mapper/PID/speed/guard/five-low/rig保持，首分叉前完整物理/关节/执行/history/SHA核验，之后测真实applied/XY及主轴方向/是否实际通过交接段、独立参考进度/RMS/检查点/goal/contact。CPU修复不保证实际primitive完成；资格后仍用原nearest投影，未证所有短段闭环完成。导航必须goal<=.2原35s0contact，旧限定改善阈值不放宽；失败如实拒绝，不扫参数/网络/重做矩阵。RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2尚未准入，本CPUheartbeat不再actualrun，全研究继续ACTIVE。


## 09:45UTC cusp projection actual35s — final stage closure

先完整读 `reports/benchmark_cusp_projection_navigation_20261005.md` 与独立冻结spec/plan `docs/superpowers/{specs,plans}/2026-10-05-cusp-navigation*.md`。ONE同起点goldenE1原35s于09:49:35.045858→09:52:28.764418UTC结束exit0，无遗留仿真/第二候选，旧root/guarded不重跑。接口PASS，限定改善对两臂均REJECT，导航REJECT(timeout35s/goalremain4.756419/0contact)。全研究NOTcomplete/automationACTIVE，常规安静，不替换较强归档B0。

只安装冻结CuspProjectionPolicy投影边界修复：未accepted当前方向期间保持索引，资格后原投影；legitimate replacement仍取消过时pending，不强制提前反转。外层继承RootDirectionPolicy act/plan_boundary，只episode reset换tracker；inner真实ConfidentSpeed/MappedBankOracle/CuspBoundary/Oracle原act/handler。runtime5处机械diff；原planner/rootcost.5/reverse1.25/node3000/geometry/guard/mapper/PID/speed/five-low/rebound/rig/light保持。仍是双rendererdepth+knownpose/未知风险，不是RGB估计pose/认证安全，outerprivilegedFalse不洗掉sensor privilege；旧RGB/sourceconsumer/M2/停止包络拒绝保留。

119源/8509输入09:49:05.318957presealed，实际350请求351双方capture1751physics2101contactsamples0actual，两视频351帧无尾步/hiddenmarkers/0age，2752关闭后manifest含feedback。首分叉182/18.2s相对root：911physics17nonraw/183execution+来源historyEXACT，raworigin-.02s/-1tick另报；141rawblocks840appliedticks改变。不能称与guarded到182全共同前缀，它与root此前175已经分叉。

四次19.7/23.7/29.7/33.3 accepted F/R交接，均真实五个当前10HzXY<.05，原反弹语义保留。180负rawticks278负applied428实测main负各自分母；197acceptedR时main还+.040957/previousapplied0，215main-.460650/previousapplied-.5/XY倒车。version3/6各约.299mR段，自己reference寿命内实际最接近终点.044042/.013268m、有向投影max.359736/.390492m，不能仅用index或accepted认证primitive。离线描述auditpostseal不改control/gate，lifespan负ticks不专属单primitive。

独立全参考终态progress3.707562、RMS.135045/peak.260893；相对rootprogress-1.682213/goalremain+1.085041，相对较强guardedprogress-1.148014/goalremain+.599439。虽然误差/0contact三项过，进度/goal两项失败，两臂gate均拒绝；4m检查点27.4s，6/8未实际到达。74direction-change零块=7.4s，合理停稳不当bug；25/25.5s observed-obstacle route失败、26s新F/R/F导致第二循环，不把首交接当全部失败。19.7→21.5实际yaw-1.178943→-1.224909；23.7前向接受时reference-.737970/actual-1.198351误差.460381，24.9仍.433406。是真实欠对准证据，未证PID/符号/映射单因果。

作者2026-10-05T09:54:10.961008+00:00核验119源/2752actualSHA、350causalpolicy/map/reference/request精确、351双方packet/350单次history、1745实测feedback执行tick（首5不clone）、3finitepastfuturetrap/truncatePASS；非递归物理预测/非3freshphysics。195/8.72与11RED/GREEN是前阶段冻结证据未冒称新跑；wall10Hz不成立，planp95 3.444s/capturepartialp95.311s另披露。继承SDKhiddenmarkerprimvarwarnings留rawlog，actualexit0不称blocked。原机械plot继承错label文件保留，独立postruncomparison_v2三实际臂更正且视觉检查。ONEfreshreadonlyreview范围/发现/每条declined裁决见cusp_navigation_review.json；审查bodySHA ef9f8d28596758816edd89ce202ad76cdf21c019cdeb1f3c4b0fc070af81c33a、作者最终reportSHA a1f061300c41dd480eeee28bb8b13787454ed4ce5304fc37df6d36a883d5979a；作者append与最终hashcheck不冒称二次review。保存全部来源/数据/失败/测试/视频/版本，无无关Gitcommit/delete。

下一项尚未实施的有界CPU工作：针对version3/6实测R tracking和资格后F tracking，核验reference body/profile与PP/feedback实际曲率合成、V1请求方向、roll反馈/PID/applied、实际yaw/XYcourse，尤其23.7/33.3欠对准和短R名义yaw与实际变化；字面直线/圆弧/正反向/连续性fixture，不二次act/不拿braking陈旧值/不默认符号bug。明确错误才选择唯一传统tracking最小修复，无错则拒绝该假设并隔离动态欠对准/来源route阻塞证据，不扫PID阈值/换M2M3网络。实际候选需另事前封存，同初态35s与原进度/goal/横向/contact gate；本heartbeat不追加search/sim。RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2仍待分层推进，全任务未完成，保持ACTIVE。


## 10:45UTC outgoing-edge lookahead CPU audit — final stage closure

先完整读 `reports/benchmark_outgoing_lookahead_and_heading_chain_20261005.md` 与独立spec/plan `docs/superpowers/{specs,plans}/2026-10-05-outgoing-lookahead*.md`。只有CPU最小query修复+一完整归档审计，无新sim/search/fit/inference/视频/正式successrate。CPU query regression PASS，实际新候选导航未测；旧35s cusp timeout4.756419/进度3.707562/RMS.135045/peak.260893/0contact与两对照限定改善REJECT不改。全研究NOTcomplete/automationACTIVE，常规安静。

确定outgoing convention bug：planner _reconstruct在i点赋actions[i]，direction是i→i+1出段；Oracle lookahead先看nextdirection故丢掉当前最后edgeendpoint。version3/6单R段旧targetidx1本身/累积0，实际197PP+17.088480而原名义Rbodyk-.956781/yaw+.287034；还有175首F0→1R也旧targetcurrent导致PP0。独立with_outgoing_endpoint仅先消费CURRENT edge再到nextendpoint；不会进对向下一edge/不改pathidx或latch。make_endpoint_oracle包冻结actualclass，EndpointLookaheadPolicy继承CuspProjectionPolicy只episode reset装配，act/rootplan/原projection/five-low/rebound/PID/BankMap/速度/协议/smoother/planner/mapper/rig/light原样，未bundle入口None Minor。

15字面F/R直线圆弧±k/小误差/边界与终点/完整同向act/五低速反弹真实fixtures，callable RED13fail2pass1.39→GREEN15pass1.36；fullH210pass9.18。初写cwd失败missingfileexit4不当RED，初fullsuite test_board_material缺libtf.so collectFAIL后加已有SDKLD库全pass，fail logs保留无testdisabled。v1审计10:56:13.960111封存125/11310，frame74用了float64diagnostic速度代替原actfloat32proprio norm导致5e-9yaw差/1e-10assertFAIL，无输出data。旧源/protocol/fail日志不改；独立v2只恢复原CURRENT dtype层语义、不放宽gate/不改control，10:57:51.417566封存126/11312。共2尝试(1不完整fail+1complete)明确披露；完整归档审计10:57:52.429785→10:57:56.377453exit0，不冒称singlelaunch。

350records188有效tracking/162无效或早return，188原target/PP/V1inverse/BankMap/raw/protocol全一致，1750旧actualbank/PIDtarget/smoothed/applied与physics逐tick一致；非递归物理预测/不重放policy.act。61zero query改正52raw/32protocol/5main当前代数改变，既有speed策略因新k触发不是新speedpolicy。首次代数分叉175，其他188行各自条件旧实测状态what-if，不是candidate真实trajectory/future标签。R3 |PP|median17.4087→2.3645仍19/19clamp；R6 26.4472→.5216clamp15→6；后F13/17queries仍same/clamp。不能把raw不同且protocol相同当执行改善，不能以counterfactual夹边少称导航成功。

真实旧R3Euler yaw-.053950/world-z integral-.057851与名义+.287034不同，但rootEuler/worldz非同量；bank是asin2(wx+yz) bodyY worldZ，非完整Eulerroll，原PID实际量归属明确。旧R3bankerrRMS.452392/PIDclip100%/supervisor3.16%，F3 .353504/78.46%/4.62%；低supervisor比例不能证限制无影响或删除，PIDclip不能独断gain不足。已知上游target错误可以改变bank请求，欠对准/routeobservedblocked仍有其他因素。已检查图只CURRENTknown refs/实际positions/查询目标，无candidate未来运动。

作者2026-10-05T10:59:27.095315+00:00无science重算hash/唯一行/计数/时序检查126源11312inputs/188rows1750feedback；ONE fresh readonlyreview独立范围/发现/每条declined裁决见outgoing_lookahead_review.json。审查bodySHA 476a36f4fda57a81861e0ef4d2299c3c2a40ba24821045ce98c71d3f25142055，作者最终reportSHA 5bf025a06663ed9d8a5ff7c1e9ac70729832ac3eeb2e5c61e85a6e23a936aea5；作者append/finalcheck不冒称二审，全部源码/REDGREEN/suite/failure/v1v2data/图/旧视频/身份保留，无无关Gitcommit/delete。自动续做仍ACTIVE。

下一阶段唯一未执行物理工作：预封存EndpointLookaheadPolicy同起点/同6s前缀/原35s goldenE1，相对归档cusp solequery因果对照，同时报告较强guarded与已拒绝root。相机/地图/rootgeometry/gains/PID/speed/handler五低速反弹/原acceptance均保持。共同完整physical/joint/exec/smoother/history/source到真实firstdivergence(不能假定197，应可能175)，之后只测candidate自己的applied/XY/bank/yaw/出段/独立进度/RMS/peak/检查点/goal/contact，原改善5项/35sgoal<=.2 0contact不放宽，失败直接reject不扫PID/阈值/网络。CPU修复不等于navigation，RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2未准入，全任务继续ACTIVE。本CPUheartbeat不再phys/search/fit。


## 11:46UTC outgoing endpoint actual35s — final stage closure

先完整读 reports/benchmark_outgoing_endpoint_navigation_20261005.md 与独立spec/plan docs/superpowers/{specs,plans}/2026-10-05-endpoint-navigation*.md。冻结EndpointLookaheadPolicy查询已真正运行。仅query改动 vs cusp causal comparator；guarded强对照和root拒绝臂同时报告。全部旧源/rig/阈值/gate不改。一次预采集sandbox GPU/CUDA/Vulkan startupSIGSEGV失败无physics/data/metrics，原空namespace/status/log保留；事前GPU环境-only addendum/获权限后原runtime重试，2launch attempts/ONEcompletedphysical sequence，非两条导航/非调参。

137source/11342inputs于11:49:58.157440presealed；fresh15fixturePASS1.40s，旧RED13fail2pass/GREEN15/fullH210原证据保留未重做matrix。唯一实际goldenE1 2026-10-05T11:51:16.855484+00:00→2026-10-05T11:54:05.194954+00:00exit0，6s观察包含原35s/350requests351captures1751physics2101contactsamples0actual，两视频351frame10fps无tail，hiddenmarkers/0age。2762postclosemanifest含PID。runtime机械diff只policy导入/装配及artifactnames，实际MRO Endpoint/ConfidentSpeed/MappedBankOracle/CuspBoundary/Oracle，原act/handler/five-low/rebound/rootcost/geometry/planner/mapper/PID/speed/auxlimit/rig/light全部保持。仍特权双depth+knownpose/source memory/unknownrisk，不是RGB估计pose或3D动态safe。原RGB/M2/停止拒绝不倒改，全研究NOTcomplete/automationACTIVE。

first REAL rawdivergence175/17.5s：与cusp此前876physical×17nonraw/176fullPID/smootherexec+bilateralsource/history精确，rawclockorigin0/0tick。102rawblocks875appliedticks改变，行为非±35→30等价。之后只候选自己的actualfuture为标签，非旧188what-if或旧cusp未来轨迹。作者2026-10-05T11:55:27.055915+00:00核源137/实际2762，350因果policy/map/reference/request、351双方packet/350history移位、1745实测feedback执行（首5未clone）、3finitepastfuturepayloadtrapPASS；非递归physicalprediction/非3freshphysical。

真实进度5.028426/距goal3.961658/RMS.171278/peak.384271/0contact/timeout35s。相对cusp进度+1.320864/goal距离-.794760，但RMS/peak更大，原限定5gate拒绝；相对较强guarded进度+.172851/goal-.195321/RMS-.002789/peak-.090995，5gate均PASS；相对root进度/goal失败。联合三臂限定改善REJECT；实际goal<=.2原35sgateREJECT，不能装成成功baseline或新E0E1E2成功率。4mcheckpoint18.6 vs cusp27.4，6/8m未过，6mmin1.147126。全参考评估器独立候选索引，停止不当成功。

仅20.1F→R与23.7R→F两交接，各原五当前10Hz XY<.05，实际90negative raw/139applied/230mainjointtick(1750tick各层不同)。方向brake38blocks3.8s vs旧74，合理制动保留，不强制反转。version3单R段.298971m实际lifespan到335，minendpoint.033647/signedmax.342228连续描述，不由accepted/index另造成功gate。R20.1→21.9Euler yaw+.048570/worldZintegral+.060030，bankerrRMS.415646/peak.610090/PIDclip68.89%。相对旧负yaw只是不同完整轨迹，不单因果标定PID/sign。F23.7→35实际沿独立ref+1.565348m，横向.155513→.384271，currentbodyheading误差entry.292202/peak.528624/last.056954；bankRMS.126795/PIDclip73.805%。234validmoving protocolclip127(54.2735%)/V1outside103(44.0171%)，kmedian.812188。仍有正常同方向饱和欠转弯与完整35s预算问题，不默认首交接解释全部失败。

描述性headingv1误读legacy数组恒0，原script/sidecar保留并拒绝；独立v2只读取actual tracking.reference_body_heading_rad/验证每validrow同reference/projection及实际rootheading，无效earlyreturn None，原控制数据不改。PIDraw/protocol中间输出在原feedbackbanklog，完整v2/phase表单独保留。Euler/worldZ不同量。计划wallp952.814430s/capturepartialp950.303882s，因果在线不是real10Hz。4计划ALLsuccess，33.5originalrasterclearance重建，没有旧25/25.5routefailure那种再次循环；旧数据不能给新动作未来标签。

全部源码/preseal/diff/失败/环境addendum/fresh测试/新actual数据/PID/视频/图/hash保存，像素已检查。ONEfreshreadonlyreview与每条declined裁决在endpoint_navigation_review.json；作者append/finalhashcheck另归属，不冒称二审。旧报告/结论/验收未倒改，无无关Gitcommit/delete/upload。阶段是否已完成以endpoint_navigation_final_check freshcheck为准，整个项目未完成，不停止自动续做。

下一项未做的有界CPU工作：复用本次237→335正常同方向实际data，核验planned/profile k与实际PP、BankMap当前speed支持夹边、PID动态bank、真实XY几何曲率/航向响应及33.5rasterguard失败的真实位置/已测map来源。只读当前/过去knownreference，不重复标定/换M3/扫增益/减速/裕量，不先验认定PIDbug；明确一个可隔离消费约束后另预sealed最多一条物理对照。本heartbeat不再physics/search/fit。可靠RGB+estimatedpose、唯一学习记忆/世界模型、原35s正式E0/E1/E2仍待分层推进，全研究继续ACTIVE。



## 12:46UTC tracking capacity and guard-source audit — final stage closure

先完整读 reports/benchmark_same_direction_capacity_and_guard_source_20261005.md 与 docs/superpowers/specs/2026-10-05-tracking-capacity-audit-design.md、plans/2026-10-05-tracking-capacity-audit.md。本轮只完成一个预封存CPU测量/来源审计，不新增物理导航、planner search、policy act、模型拟合或控制候选。144sources/14148旧与实际inputs于12:56:02.888173UTC冻结，一次完整审计12:56:18.580804→12:56:19.612774退出0；RED4fail2pass.16s→GREEN6pass.12s。首次cwd写入失败无文件/无science，保留日志。原35秒结果、rig、mapper/PID/speed/guard及旧RGB/M2/停止拒绝不改。

237..334的98个有效同方向tracking：名义main+.5/aux-30°的V1机体曲率+.880052，而当前速度插值BankMap正端点.796201..796603；98/98名义超过加权端点、98/98正式map夹边、98/98协议夹边，80/98速度超出表域被夹边。加权端点不是各层逆映射顺序，也不认证动态物理上限。490真实PIDtick377clip/5supervisoractive；早期倾侧滞后明显，334实际bank-.393705已接近目标-.392699，仍请求k1.660542/离参考.425525m。不能单凭PID饱和认定增益不足，不能说限制无作用或删supervisor。

10个不重叠测量窗全部可估计。真实XY三点几何曲率与root Euler yaw/arc、world-z积分/arc分别报告；早期XY曲率负并不自动证明打滑/符号bug，后期bank达到不证明恒曲率稳态。窗口后续实测只作离线描述，不输入在线预测。数据、唯一行/分母、图、PID与当前reference字段源均保存；禁止使用legacy全零heading数组。

33.5s原guard的4个past/current位置330/335×map330/335组合，位置330均保留，335均因原rasterclearance拒绝；只说明这个局部screen，不证明map全程无影响。原连续suffix/bridge距离.640678893>=.6，原rounding EDT.583095189<.6，规则如实触发重规划而非碰撞终止。两失败sample是同一投影点，round位移.058137568m；最近hazard为历史code4、timestamp13.099999707/frame131，来自3个原实测depth点，高度.528327..643834。hazard不受floor10秒TTL删除，不是335当前视觉/脚下先验。本层是特权depth+knownpose，不能改写为纯双RGB性能。

作者12:59:10.556788UTC核144/14148及全部保存计数/来源时序，无science重跑；两PNG视觉核验。本轮无新视频，旧endpoint两条实际视频作为上下文。ONE readonly第二人审查见tracking_capacity_review.json。新fresh-context dispatch因agent thread limit被拒绝，实际复用上一阶段审查者并公开保留上下文；不称fresh或二次独立重跑。正文与审查/作者追加分开，作者append/hashcheck未二次review；最终审查发现与逐项scope裁决以附录为准。保存所有代码/失败/数据/图/SHA，无无关Gitcommit/delete/upload。阶段完成以tracking_capacity_final_check为准；整个研究仍未完成、automation ACTIVE。

下一项尚未实施：ONE有界CPU规划名义响应/控制consumer一致性实验。先审查原V1/BankMap曲率定义、命令域与steady nominal适用性；在一个完整已知当前快照（可用175首次重建前，不取未来335结果设计输入）上只改变一个名义曲率响应来源或必要screen，原搜索/mapper/PID/gains/speed/rig/guard不动。比较同起态原计划与唯一候选的几何/goal/clearance/长度/换向数/当前请求域支持，冻结接受门槛后最多一次搜索对照；不把当前速度预测全部未来、不用旧实际轨迹当候选label、不强制禁reverse或循环等待。CPU资格通过才另阶段最多一条原35秒同初态实际对照。当前不再science/search/sim，RGB+estimatedpose、唯一学习记忆/世界模型与正式E0/E1/E2仍未准入，不宣称导航成功率。

