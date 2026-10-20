# 短换向段前瞻与真实航向执行链核验

2026-10-05 10:45UTC有界CPU阶段。确认一个参考消费错误并完成唯一最小查询修正：规划器的方向/指令描述当前点的出段，旧前瞻却在读取出段终点前检查下一点方向，导致单个短F或R段的目标退化成自身起点。已通过字面几何及执行规则回归；归档当前状态下有真实protocol请求变化。新候选尚未仿真，不宣称导航改善。上一条35秒cusp候选失败与全部旧gate保留；整个双目/估计位姿/学习记忆研究未完成，automation ACTIVE。

## 身份与约定核验

工作区 `/home/an/Documents/ChatGPT/球型机器人导航`，H=`experiments/stereo_memory_navigation_20261004_v1`，原项目只读。使命/主计划/ledger已完整读的冻结前缀，经上轮document_sync SHA精确复核后复用；最新append及上轮完整报告重新读，不把宽输出截断称作本轮全新完整读。实际cusp status finished/exit0，finalcheck通过，未续跑已完成的物理序列。spec/plan为 `docs/superpowers/{specs,plans}/2026-10-05-outgoing-lookahead*.md`，用户已授权自主实施，既有隔离H/unbornGit保留，不做无关commit/delete。

原 `benchmark/kinodynamic_planning.py::_reconstruct` 明确在index<len(actions)时赋 `main,aux=actions[index]`，direction由main符号决定。因此i点属于i→i+1 **出段**，i+1的标签用于下一条出段；端点坐标必须包含在当前路段中。原OraclePathPolicy `_lookahead_target` 在消耗i→i+1之前检查 `reference[i+1].drive_direction != current_direction`，两个约定不一致。

实际version3点1R→点2F，反向段长.298971m；旧前瞻idx1、累积0。197位置距点1仅.046965m，PP body曲率+17.088480，方向sign=-1导致yaw-rate请求-3.417696，原名义该R出段body曲率-.956781、签名-.3m弧长带来名义yaw增加+.287034rad。旧反馈追向段起点、近零距离将曲率放大；这项证据不要求假定PID或物理转向符号错误。version6同样idx1R→idx2F、短段.297952m，名义body曲率-1.350482/yaw增加+.405145rad。

更早frame175新参考初F出段0→1R也受影响：原target等于当前位置、PP=0，修正target为点1、PP=.622023，与该出段模型曲率.622023精确一致。不能说修复只影响倒车或从197才分叉。

## 唯一候选及测试

独立 `H/outgoing_lookahead.py`：`with_outgoing_endpoint`只覆写 `_lookahead_target`，以**当前出段**方向决定是否继续，再积累到下一坐标。到达下一方向起点（当前方向终点）可返回该坐标，但不进入其相反出段。投影索引/active/pending不在查询内改变，五次低速/反弹由原handler保留；终态零指令marker与同方向普通查询行为不变。

make_endpoint_oracle在冻结make_cusp_oracle实际class外包这一层，外部EndpointLookaheadPolicy继承CuspProjectionPolicy，仅episode reset装配该tracker，act/plan_boundary继承不变。真实链 Endpoint→ConfidentSpeed→MappedBankOracle→CuspBoundary→Oracle，原BankMap/V1/profile/PID/协议/smoother/supervisor/五低速/rootplanner/速度逻辑不改。没有把M1/M2/M3放进planner，也不改相机X=.20/Y±.30/Z=.05、照明/来源地图/未知标签。边界目标变化可能触发**已有**调速条件，不能称同时换了速度策略。

15真实字面fixture：短F/R/F及R/F/R终点、夹到边界不读相反边、四种F/R×±圆弧的理想body曲率、小位置/heading误差及正反符号、同向完整act+请求+全部diag精确、terminal marker、实际act五低速及反弹/外层身份。期望由手算独立确定，非planner生成expected，未mock生产方法。原可调用scaffold RED **13失败/2通过，1.39s**；唯一方法实现后GREEN **15通过，1.36s**。完整H **210通过，9.18s**。

失败保留：首次写文件cwd与相对路径错配，文件未生成、pytest missingfile退出4（prewrite_failure.log），这不是行为RED。完整CPU suite最初test_board_material收集因libtf.so缺既有SDK LD_LIBRARY_PATH而失败；加回已有库路径后全210通过，未屏蔽测试，environment_failure.log保留。没有仿真失败/外部阻塞或新科学矩阵。

## 归档核验、两种速度精度与界限

v1于10:56:13.960111UTC封存125源/11310输入，首审计在frame74停止：act用float32 proprio XY范数.203180164099，tracking/BankMap诊断用float64速度范数.203180155373。重建若混用，yaw请求差5.17e-9、超过原1e-10一致性检查。正确float32当前输入恢复原-.12036346043047019；这不是控制缺陷，不能放宽gate遮住。

v1源/protocol/失败log保持；独立audit_v2只按原代码分开两层当前速度，BankMap仍使用原float64诊断速度，yaw-rate用原float32当前proprio。v2封存10:57:51.417566UTC，126源/11312输入。共**两次尝试：第一不完整失败、第二唯一完整审计**；不是宣称只启动一次。v2实际10:57:52.429785→10:57:56.377453UTC退出0，无archive act/planner/search/sim/fit。

350旧记录中，仅tracking.valid且真实source=tracking的 **188** 条作曲率/映射/请求重建，162braking/settling/no-route等无效或不适用行剔出分母，不读取陈旧诊断。188原lookahead/body PP/V1逆映射/current world-z feedback/BankMap/raw/protocol逐条与实际一致；V1 forward(inverse(k))在原支持范围内自一致，超范围明确夹边，未将其当物理充分模型。BankMap相对V1是独立稳态映射，命令用它加原yaw correction；profile曲率仅诊断，不在此真实class合成入PP，没有重复前馈/heading feedback在实际曲率合成中。PID稳定bank是后续真实执行环节，不能混称几何计算。

候选每一行只在**同一当时实测状态+当时已知参考**上改query并计算当前请求；未读取未来applied/位移，没有二次policy.act。首次当前代数分叉frame175/17.5s。175之后其他行是分别条件在旧真实状态的独立what-if，**不是候选递归rollout或真实标签**，不能把197之后旧轨迹当新候选运动。52/188 raw请求改变，32/188 protocol请求改变，5条主轴请求改变；剩下raw变化但protocol相同不能称执行改善。61原零前瞻边界行变为正长度；其中也包含末F出段。全188原协议裁剪98，候选当前代数裁剪84；此计数下降不是导航gate。

作者后续verify只重核源126/旧及实际输入11312/唯一行及计数/时序/SHA，未再算全部策略或运行仿真，结果 `H/outgoing_lookahead_verification.json`。actual反馈1750ticks仅作实测对应关系检查，不重新运行smoother/物理预测。

## 具体当前查询与未解决的饱和

|实际时刻|旧/新目标idx|旧PP body 1/m|新当前PP body 1/m|旧/新protocol aux rad|
|---|---|---:|---:|---|
|17.5s F|0/1|0|+.622023|-.010311/-.403104|
|19.7s R|1/2|+17.088480|-1.218632|-.523599/+.523599|
|21.5s R|1/2|-1.470696|-7.718638|+.523599/+.523599|
|23.7s F|4/4|+1.588542|+1.588542|-.523599/-.523599|
|29.7s R|1/2|+25.765510|+.540486|-.523599/-.371511|
|31.3s R|1/2|-1.065886|-5.153290|+.523599/+.523599|
|33.3s F|4/4|+1.881336|+1.881336|-.523599/-.523599|

19.7s修正指向反向终点，使瞬时曲率符号与该出段目标更合理；但29.7s因旧实测位置/朝向偏差仍得到+.540486，不能强行替换为名义-1.350482或声称所有符号都纠正。整个version3 R 19行：|PP|中位17.4087→2.3645，但BankMap/协议仍19/19夹边；version6 R 17行：|PP|中位26.4472→.5216，夹边15→6。version3后F 13行/version6后F17行查询不变、全部夹边。新终点仍是稀疏单点目标，动态是否可执行必须实际测，不能由离线幅度小替代。

## 实际roll/PID/航向层

本次使用旧cusp实际日志、不是新候选。1750feedback行bank取**base/root body-Y轴的世界Z倾侧分量**：bank=asin(2(wx+yz))；它是正式BankPID实际读取的物理侧倾，通常不等于完整Euler roll=atan2(R21,R22)，球壳/body身份沿上轮源码冻结。本报告不拿球壳滚动Euler累积当侧倾。

逐tick核验bank与root quaternion、desired bank=.75×夹过协议边界的aux、smoothed/applied与physics原值对应。BankPID的target/PID输出、正式smoother、aux速度supervisor、实际aux角度分别保存 `H/outgoing_lookahead_actual_feedback.json`。所有都是真实旧反馈，不作为新candidate预测输入/成绩。

|旧实际区间|真实Euler yaw变化rad|world-z角速度积分rad|bank误差RMS rad|PID自身夹边比例|速度supervisor比例|
|---|---:|---:|---:|---:|---:|
|19.7→21.6 R|-.053950|-.057851|.452392|100%|3.16%|
|23.7→25.0 F|+.035303|+.004835|.353504|78.46%|4.62%|
|29.7→31.4 R|-.004303|-.023994|.357725|78.82%|3.53%|
|33.3→35.0 F|+.033119|-.008620|.334580|60%|4.71%|

Euler yaw变化与世界Z角速度积分在倾斜机体上并非同一量，不能直接相减说测量错或yaw符号反了。真实R执行main applied已经负，但bank请求在旧起点目标附近反复改变且自身夹边，实际响应不够稳态；F的欠对准和bank误差也存在。低supervisor激活比例只能否定“这一窗口几乎每tick被速度supervisor挡住”的说法，不能证明限制无影响/因此可删。不能用PID自身夹边比例单独认定PID增益不足；上游不合理/饱和目标是已有确定问题。

原35秒导航保持timeout/goalremain4.756419/独立进度3.707562/RMS.135045/peak.260893/contact0，限定改善相对root与guarded均拒绝；25/25.5秒真实观测route阻塞也不能被query修复直接抹掉。此CPUstage无新导航成功率、无正式E0/E1/E2、无RGB/source gate准入。

## 可视证据与下一实际阶段

![当前已知路段、真实当前位置与查询目标](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/outgoing_lookahead_current_geometry.png)

图只显示197/297当时已知参考/真实位置/两查询目标，右图为旧条件下逐时刻代数值；没有候选未来轨迹。保存全部源/test/RED/GREEN/suite/失败log、v1/v2协议、188完整行、1750实际反馈、verification/图，原实际双目/外视视频在cusp_navigation_run，不新造视频。

本CPU候选可进入**另一个阶段唯一原35秒实际比较**的预封存准备，尚未准入导航。仅把同EndpointLookaheadPolicy查询安装到上轮runtime，其余冻结。对归档cusp作sole-query因果比较，同时报告較强guarded和已拒绝root。第一分叉可能175、不能假定到197共同前缀；核验完整物理/关节/PID/smoother/latch/history/source前缀、之后真实applied/转弯/进度/横向/检查点/goal/contact。原限定改善与35秒导航gate不放宽，失败拒绝，不同时调PID/速度/handler/地图/目标。CPU纠错可与最终导航失败共存；后续RGB+估计位姿与唯一学习增量仍需分层推进。

## 独立审查与作者收尾

ONE fresh readonly reviewer `/root/outgoing_lookahead_review`（gpt-6-astra）：0 Critical/0 Important/1新增Minor+1继承Minor。裁决为有界CPU查询修正证据可归档，并可进入另预封存单次实际比较准备；不能准入导航baseline。审查bodySHA `476a36f4fda57a81861e0ef4d2299c3c2a40ba24821045ce98c71d3f25142055` 原字节保留，以下补充和最终document/hashcheck是作者收尾，不是第二次review。

新增Minor经作者亲自核验并澄清：`H/outgoing_lookahead_actual_feedback.json` 保存 bank target/error、`pid_clipped`、实际关节、smoothed/applied 等摘要；**PID的 raw/protocol 中间输出** `pid_raw_aux_rad`、`pid_protocol_aux_rad` 保存在冻结原始 `H/cusp_navigation_run/feedback/bank_feedback_golden_e1_0001.json`，不是摘要sidecar。两文件共同对应执行链，原摘要/原始log/正文不改。继承initializedNone root入口Minor保持另维护，不混入本query候选；本次原实际初始化F/R有效。

审查者亲自重核126源/11312inputs与前轮5docsSHA；限定15fixture新跑15PASS/1.35秒（禁bytecode/cache），重算350/188/162分母、175firstbranch、52/32/5raw/protocol/main改变、61zeroquery和98→84当前代数clip；用当前已知几何重算175/197/297 PP=.6220227158511691/-1.2186324011896603/.540486463245111；重核frame74两种速度精度，全部1750旧actualbank/target/smoothed/applied与四窗口量，视觉检查CURRENTgeometry图。审阅者自己的第一次vectorized quaternion校验保留float32失败，按生产函数先castfloat64后最大差1.11e-16；仅只读审阅计算修正，未改数据/重跑科学审计。

作者保存的RED/GREEN/fullH210及环境失败、两次archive尝试/v2完整188原请求重建断言、原35s失败由reviewer检查source/log/hash，未重跑fullsuite/科学audit/physics。后补MRO/closure/check是作者来源与行政证据，reviewer未执行行政收尾，不冒称独立复制全部验证。源/数据/候选/gate保持冻结。两次尝试偏离原ONE/no-scientific-retry字面要求，作者裁决：当前dtype来源复核可确定、v2重新封存/v1保留、gate不放宽/candidate不变，报告显式2launch1complete；接受这一次精度修复偏离，不开启第三scienceaudit或更多candidate，错误成本是原输入语义被误解释，已经对应事实修正。

15条declined范围逐条保存reviewer理由/作者裁决/误判成本于outgoing_lookahead_review.json：实际导航、实际gain、后续参考/index、物理前缀、primitive完成、其他tracking质量、PID单因果、supervisor删除、动态model、yaw量等价、route修复、RGB/pose/learning/formal准入、real10Hz、None入口、未来行政/scheduler。各自留在对应证据层，不合并丢弃。最后一项由作者另只读确认automation文件ACTIVE并freshclose/check，不改变scheduler，也不宣称整个研究完成。

最终作者关闭/检查保存于H/outgoing_lookahead_final_identity.json、document_sync.json、final_check.json；只有fresh检查通过才称本CPU阶段归档完成。原导航拒绝保持，新实际候选未运行，下一项原35s物理对照保持未执行；本heartbeat不追加任何search/sim/fit/网络或参数扫描，整个任务继续ACTIVE。
