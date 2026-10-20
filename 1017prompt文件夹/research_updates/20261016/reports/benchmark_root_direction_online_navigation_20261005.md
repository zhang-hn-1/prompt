# 当前方向根成本：在线35秒导航对照

2026-10-05，07:44UTC续做阶段。**接口通过；事前限定改善拒绝；导航拒绝。** 唯一候选真实多前进约0.534m、目标距离减少约0.486m，但相同冻结评价参考的横向RMS从0.174升至0.572m，峰值从0.475升至1.106m，35秒内没有到达。候选不替代已归档guarded B0。进一步定位：候选参考里的短倒车段在尚未停稳时被投影索引越过，实际没有执行倒车；减少制动时间不能算成功换向。整个双目记忆导航研究仍未完成，automation保持ACTIVE。

## 1. 唯一改动、身份和冻结范围

工作区 `/home/an/Documents/ChatGPT/球型机器人导航`，H=`experiments/stereo_memory_navigation_20261004_v1`；原项目只读。新文件 `root_direction_consumer.py` 的 `RootDirectionPolicy → GuardedReferencePolicy → DepthLocalCurvePolicy` 只覆写 `plan_boundary`，继承实际 `act/reset`；内部仍为 `ConfidentSpeedPolicy → MappedBankOracle → OraclePathPolicy`。源码/class检查见 `root_direction_identity_audit.json` 和两个独立diff。Git为未提交的既有证据工作区，不建立无关commit；以源码SHA、冻结输入和spec/plan识别版本。

仅在原0.5s重建边界，将**重建前已经接受的方向锁存**传给独立planner，沿用原0.5m换向代价惩罚相反根。首次参考尚未安装时传None，构造器默认forward不是实测接受方向；后续输入不读取pending、desired、raw/protocol/applied、关节或未来反馈。保留参考时不搜索、不重置物理/smoother/PID/方向历史。copied planner的waypoint/result按字段转换回原runtime dataclass，不改单位。两根始终可选。

保持原reverse1.25、node3000、primitive0.3m/采样0.05m/goal0.2m、local target2.4m/crop±4m、observed hazard clearance0.6m、旧连续及栅格guard/remaining≥0.5m/projection≤0.5m。roll PID(.4,.1,.7)、主轴调度(.5,1,1.5)、副轴supervisor0.3rad/s、lookahead0.5、协议、smoother、XY连续五次10Hz低于0.05m/s及反弹清零均保持。零请求6s前缀计入原35s规则。相机X=.20,Y=±.30,Z=.05m、640×480/focal584.8889、照明/自遮挡mask/隐藏markers/采集事务不变。

外层privilegedFalse只拒绝runner全图/全局Oracle参考输入；本实验仍用renderer实测深度和known pose，是**特权深度+真位姿、显式未知风险开发对照**，不是RGB基线或安全认证。全局goal允许，完整参考/全图仅离线评价。无脚下free先验、ray carving、GT障碍过滤、未来输入或新网络。旧RGB/source consumer/M2/停止包络及历史拒绝原样保存；估计位姿、学习增量、正式E0/E1/E2未触发。

## 2. 封存、运行和真实动作变化

`root_direction_protocol.json` 于07:54:23.300306UTC封存106来源/5670旧输入；旧guarded验收字典逐字段相同。仅一条候选 `golden_e1_0001`，07:54:40.229131—07:57:32.904552UTC退出0。旧 `guarded_reference_run` 不重跑，候选保存于 `root_direction_run`。未重复完整矩阵、扫描惩罚/速度/node/目标或启动第二候选。事前封存完整序列不等于fresh环境泛化。

350请求、351双方capture、1751physics、2101可用接触样本，实际接触0/runtime error0。两个视频各351帧10fps，无legacy计分外尾步；camera age0/markers隐藏。关闭后独立manifest有2762产物并包含bank反馈；原初始manifest在runtime.close前写入，不冒称完整关闭后manifest。

第一raw请求分叉frame175/17.5s，此前175请求、876physics的17非raw字段、176完整执行快照与176双方来源/history packet精确相同。raw初始化原点候选比B0多0.01999999955s/1tick，相对物理/动作时钟一致，单独披露。分叉后116请求块和738真实applied ticks改变，所以这是真实控制干预，不是把raw±35°规范化为±30°的行为等价修改。

候选实际frame175的全部pose/direction/command与上一阶段冻结名义候选逐数组精确相同。58计划边界中46保留、12重建均success；B0为50保留/8重建(6成功/2失败)。首次6s输入None，其后11次重建输入forward；保留时新输入None，plan metadata保留安装时的输入，不当成当前重新调用。各retain前后完整边界状态不变。原guard允许真实失效/耗尽重建，没有强制锁侧。名义planner success不意味着实际段执行成功。

## 3. 同一独立评价器的实际结果

冻结评价参考长度8.836753m；两臂同一几何投影器，独立于候选path_index和局部参考更新。验收原样：进度增≥0.05m、目标距离减≥0.05m、横向RMS/peak不超过B0+1e-5、零实际接触，全部同时成立才限定改善通过；真实goal≤0.2m且35秒内零接触才导航通过。

| 实测量 | 归档B0 | 唯一候选 |
|---|---:|---:|
| 终态沿冻结参考进度m | 4.855576 | 5.389775 |
| 目标剩余距离m | 4.156980 | 3.671378 |
| 横向RMS m | 0.174067 | 0.572390 |
| 横向peak m | 0.475266 | 1.105993 |
| 真实XY路程m | 6.411888 | 6.881080 |
| 真实接触 | 0 | 0 |
| 终止 | timeout35s | timeout35s |
| 事前限定改善 | 历史对照 | **拒绝** |
| 导航goal entry | 未达到 | **未达到，拒绝** |

描述性空间检查点使用冻结世界XY坐标、半径0.3m：2m处两臂均13.9s；4m处B0 25.3s、候选19.7s；6/8m处两臂均未实际通过。候选到6m检查点最近仍1.252m，到8m最近2.890m，不能用索引提前跳转称完成。

有效moving tracking分母271，协议clip141/271=52.0295%、合法角目标到边界相同；V1超旧support117/271=43.1734%。B0分母224、对应54.464%/45.089%，不同分母不代表全链恢复。60 settling、271 tracking、18 goal_braking、1 direction_change_braking；goal_braking是局部参考消费来源，不是到达全局goal。

plan耗时p50 .355316s/p95 1.864794s，policy p50 .002583/p95 .008239s，capture部分p95 .310839s。模拟时间因果online成立，不宣称实际墙钟10Hz。

![实际轨迹、独立参考进度和横向误差](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/root_direction_comparison.png)

## 4. 分段核验：倒车段被越过，并非换向已改善

使用关闭后的完整实际记录做只读诊断，`audit_root_direction_phases.py`、`audit_root_direction_handoff.py` 不调用policy.act、不搜索、不产生另一计划标签或改控制。frame175参考前三个primitive方向F→R→F；waypoint方向表示其后输出段。

| 实际事件 | frame181/18.1s | frame182/18.2s |
|---|---|---|
| 投影前idx/参考方向 | 0/forward | 1/reverse |
| accepted/pending/低速计数 | F/None/0 | F/R/0 |
| 实际选中idx | 1 | 3 |
| 选中参考方向 | reverse | forward |
| 请求来源 | direction_change_braking | tracking |
| 结果 | 开始等待换向 | 取消pending，仍forward |

冻结原 `benchmark/policies.py` `_project_to_reference` 从下一个waypoint开始检查与accepted的方向差异；当前idx已经进入reverse但accepted仍forward时，下一个点又是forward，窗口因此放行后续forward段。当前XY各候选点距离精确复算，181选1、182选3；期间reference version3没有替换，未reset。`act`先投影再调用方向处理，pending尚未连续五次低速资格，就因desired回F而取消。此为已定位的继承参考消费接口现象，本轮未修复它。

17.5—23s期间negative raw/applied/main measured joint均0，最低applied主轴+.45、最低实际主轴+.418757、root航向上的XY速度最低+.161221m/s。倒车primitive没有实际执行。B0 accepted F→R19.5s→F23s，制动总3.8s；候选accepted始终F、只一块0.1s制动。不能据此声称合法换向开销3.8→0.1s或规划已更可执行。分段表中的制动区间不包含其后第一tracking资格帧，B0区间内five-low标null不代表B0未达到资格。

冻结候选当时**自己已安装**的version3参考，17.5—23s实际进度1.364537m、RMS.260636/peak.510890m，支持这次交接段确有实际参考执行偏离，不能只用全局参考绕路差异解释它。但后续23—27s稳定version4，RMS.035565/peak.067112；30.5—35s稳定version12，RMS.022107/peak.038075，仍能跟当前局部参考。27—30.5s有多次替换，冻结旧version5窗口只能描述，不能称同一参考纯tracking对照。

候选18.2—35s沿独立全参考增加1.707655m，误差从.164601升至1.086392m，peak1.105993m。不同阶段窗口/地图/后续路线均改变，因此尚不能把全部终态超时/全局偏离因果归为第一次跳段或PID；局部跟踪小误差也不意味着全局目标完成。

## 5. 实际验证、测试和证据范围

六真实feature fixtures：缺callable RED1.81s→GREEN2.53s；全H 184测试通过8.45s，原日志保留。覆盖初次None的原toy plan精确相同/原runtime类型、retain禁搜索/状态不变、acceptedR与desired/pendingF分离、真实反弹.06清零/第五.04资格、未安装非默认latch仍None、非法方向在搜索前拒绝、terminal-only无可用参考语义。终态objective_summary旧Minor冻结，新consumer不调用它。科学源/配置预数据封存后未改。

作者完整核验07:58:39.955951UTC：106源/5670旧输入/2762关闭后产物SHA；350实际policy请求、地图、局部参考、诊断和mapping重建精确；351双方实测深度经known pose重建packet；350 history移位仅一次；3个有限过去状态克隆future-payload trap/截断不变。后者不是3条fresh物理验证。1745 ticks实测反馈下protocol/PID/smoother/supervisor执行和快照一致；首5ticks未克隆，不是无真值递归物理预测。

初始assessment的interface_pending保留原字节；随后verification为接口PASS/导航REJECT，不倒改旧gate。后运行只读分段诊断不影响预封存控制/验收。原35秒、rig、所有旧来源/历史报告/先前gate保持。独立最终审查及逐条范围裁决在追加作者闭合段中单独标注；不把作者后追加内容冒称再审查。

关键文件均在H：`root_direction_protocol/status/assessment/verification/handoff_audit/phase_audit.json`；`root_direction_run/{physics,episode_arrays}.npz`、`policy_records.json`、`execution_states.json`、maps/online_inputs/sensor_packets/history/local_plans和post-close manifest；source/runtime diff、RED/GREEN/suite/seal/run/assess/verify日志。运行时全部实际证据保留，不删除失败结果。

[实际双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/root_direction_run/stereo_view.mp4) · [实际第三视角视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/root_direction_run/third_person.mp4)

![实际双目17.5s和35s截图，自遮挡仍明显](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/root_direction_stereo_view_frames.png)

## 6. 接续一项有界工作

本候选关闭，保持原B0与各项拒绝。下一阶段先CPU精确复现**未完成换向时投影越过短反向段**：参考来源/waypoint出段语义、accepted/pending/五次与反弹、投影窗口调用顺序分别核验；再设计一个最小投影方向边界修复，不能跳过真实反向段，也不能形成永远无法开始换向的等待。重规划合理替换要保留明确定义，不能把全局lateral大就认定局部tracking坏。

只有真实fixture复现并验证一个局部修复、身份明确和事前封存独立进度/横向/目标/接触gate后，才在**后续独立阶段**考虑唯一同起点35秒控制对照。本heartbeat不再仿真、不改B0换向资格、不扫root成本/速度/目标/node或拟合模型。该consumer问题解决也不自动认证RGB深度、estimated pose、学习记忆/世界模型或正式E0/E1/E2；研究任务继续，常规不通知完成。

## 独立审查与作者收尾

唯一fresh-context只读审查 `/root/root_direction_online_review`（gpt-6-astra/high），0 Critical、0 Important、2 Minor，允许关闭本次**失败候选阶段**，不允许生产/导航准入。审查正文SHA `9da9130cd1ff5cc20d48512f8763cd086cb1cde82f1e1dad0b8d22d5dc0b37a3`；作者以下追加内容未进行第二次独立审查。完整返回要点/亲自复核范围见 `root_direction_review_notes.md`，逐条declined作者裁决见 `root_direction_review.json`。

Minor分别处置：①已安装context的accepted=None在新adapter仍被允许；原tracker正常路径不会产生、本次后11次均F，不影响已存结果，作者按Minor保留。后续独立可编辑版本须新增无搜索None fixture并RED→GREEN收紧已安装F/R校验，本轮冻结源不改。②正文ConfidentSpeedPolicy仅是简称，准确qualname `with_confident_speed.<locals>.ConfidentSpeed`，来源 `/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_feasible_route_20260929_v9/hybrid/gated_speed.py`，SHA `152f25764480542f32ad2f3f28119e1ab4b20bc4f9ca794bb04a57115901c55b`；MappedBankOracle的准确qualname为 `make_policy.<locals>.MappedBankOracle`。不改已审查正文，不把简写称不同控制器。

审查亲自运行4个无搜索fixture（4 passed/2 deselected/1.13s），独立复算两臂XY投影、真实goal距离/空间检查点、首分叉及物理/执行/来源前缀、116raw/738applied变化、46retain/12rebuild、175名义参考匹配、181→182索引与实际无倒车、两臂2101零接触。106source/旧40产物/5docs/handoff5input SHA及两mechanical diff、冻结验收字典亲自核验。184全suite/完整350policy重建/351depth/350history/1745反馈/3未来检查/完整5670旧input2762output检查只读作者已有证据，不冒称审查重跑。审查没有sim/search/fit/policy replay/文件或Git操作。

作者逐条接受10项范围限制并记录若误判的代价：修复效果留下一阶段；首次跳段不当成全程唯一主因；.5成本不当最佳值/真实制动模型；planner最优/泛化未证；未知/3D动态安全不认证；RGB/estimatedpose/learning/formal门槛保持；墙钟10Hz未证；旧summary不使用不修改；无部署/merge；最后行政append/hashchecker为作者独立完成、未审查范围。这些限制不丢弃当前已证实的继承投影缺陷，也不免除下一阶段修复责任。

106科学源/5670旧输入/2762实际输出与reviewed body先核验原SHA，然后仅追加mission/mainplan/ledger。最后fresh check核验源/输出/阶段产物/5文档、旧文档前缀/唯一review/时间顺序和拒绝gate。无需重复科学计算/实际仿真。整个研究未完成，automation ACTIVE；下一项CPU投影方向边界复现及唯一最小修复，旧B0/rig/gates保存。
