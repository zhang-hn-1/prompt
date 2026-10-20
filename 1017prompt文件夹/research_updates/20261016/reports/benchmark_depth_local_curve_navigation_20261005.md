# 特权双目深度＋来源记忆＋局部曲线：一次原35秒 E1 闭环诊断

2026-10-05，03:39UTC heartbeat。本阶段已完成一个有界实现及一次真实导航实验；完整研究未完成，automation ACTIVE。最新数据位于 `experiments/stereo_memory_navigation_20261004_v1/depth_local_curve_run/`（下文 H 指其父目录）。

## 结果与范围

接口与执行重建通过；导航候选拒绝。golden_e1_0001 在原35秒内未到达，终距 **4.486646m**，实际路程 **5.890657m**，真实接触 **0**。58次局部搜索全部成功，不能把失败解释为没有路径或搜索超预算。本次把“当前和历史深度形成来源地图，再生成可执行局部曲线，由成功 Oracle 的 roll 跟踪器执行”的完整链真正接入了仿真，但尚未获得合格局部导航基线。

这是一个独立的 **renderer depth＋known camera pose 特权控制包**，不是纯RGB/估计位姿导航，不是上次 InformationPolicy 的单因素修复。传感、局部参考、控制任务域与6秒零请求前缀都是预先声明的包条件；不能把与上次完整地图 Oracle 的30.7秒成功差距归因给唯一一个模块。旧 RGB、M2、停车包络与安全认证拒绝结论保留；不启动正式 E0/E1/E2，不报告新导航成功率。

## 冻结接口和真实控制身份

设计与计划分别为 `docs/superpowers/specs/2026-10-05-depth-local-curve-design.md` 和 `docs/superpowers/plans/2026-10-05-depth-local-curve.md`。72项源、配置与输入 SHA 在 **03:52:11.003572UTC** 封存，早于运行；见 H/`depth_local_curve_protocol.json`、`depth_local_curve_identity_audit.json`。封存后的设计、计划和源不追加改写。

外层 `DepthLocalCurvePolicy` 的 `privileged=False` 仅表示 runner 不向它提供全局 Oracle context；不表示 renderer depth/knownpose 变成非特权输入。外层明确拒绝完整 occupancy/global_reference/full Oracle path，只接受当前双目投影点、当前实测状态和全球目标。严格 packet 字段只有 frame/time_s/points_world/root_xyz；非有限值、额外真值字段被拒绝，输入复制隔离。离线 occupancy 和旧30点 Oracle 参考仅在运行结束后用于评价，不回写地图。

地图复用旧 ProvenanceGrid/SpatialMemory：真实 floor/hazard endpoint、来源与时间分别保留，floor寿命10秒，hazard保留历史；unknown始终独立，无脚下free种子、ray carving、插值、完整地图过滤或预测free。允许在 **规划副本** 对 unknown 承担风险，不能宣称已观测可通行或完整3D安全。

每0.5秒由当前来源地图 A* 获取前2.4m目标，以机器人周围±4m地图裁剪、全部已观测hazard的原0.6m clearance及原 V1 nominal primitives，调用已有 Hybrid A*，node budget3000。没有改搜索算法/重做标定/拟合物理模型。nominal几何可行不等于从任意实际速度、roll、副轴状态能及时实现。

内层实际继承链 **ConfidentSpeedOracle → MappedBankOracle → OraclePathPolicy**，复用冻结 BankMap、roll PID **(.4,.1,.7)**、lookahead .5、aux supervisor .3、协议/smoother。其成功 Oracle 原有 .5/1/1.5主轴计划/调度域保留，旧 InformationPolicy 的 .3 unknown 主轴上限未继承，必须按独立任务域理解，不能说速度条件相同。初次安装初始化内层；后续只替换局部几何参考及索引，保留 active/pending direction、五次低速确认/反弹清零、policy历史与执行状态。没有重置 smoother/PID，没有强制提前反转。实际 act 每帧只调用一次，不二次调用有状态策略获取诊断。

## 实际运行与原始证据

唯一真实运行 **03:52:31.736585—03:55:56.221204UTC**，退出0。冻结相机 `ball_stereo_front020_up005_v2`、安装/照明/材质/掩膜、hidden markers 和同步采集事务均保留。前6秒零请求也计入原35秒，之后29秒执行；没有追加 legacy 视频尾部5秒物理步。

| 项目 | 实测 |
|---|---:|
| 终止 | timeout / 35秒 / 350 action blocks |
| 真目标余距 / 余距减少 | 4.486646m / 2.113355m |
| 真实路程 | 5.890657m |
| capture / physics samples | 351 / 1751 |
| 可用接触样本 / 实际接触 | 2101 / 0 |
| 两个视频 | 各351帧，10fps |
| 同步相机最大年龄 | 0s |
| 规划 | 58次 / 58 success |
| 独立冻结参考终态进度 | 4.167298m |
| 同一独立参考横向 RMS / peak | .497267m / 1.114722m |

独立参考仅是完整地图旧成功路径上的统一评价，未用于本次控制；它不同于在线局部路径，所以横向误差不是纯跟踪器误差。runtime minimum clearance .208276m 是二维已知 occupancy 的描述性评价，不替代实际接触或完整3D证书。6秒零请求期间仍有初始物理松弛位移 .106468m，“零请求”不是“冻结物理”。35秒名义协议时间与浮点累计 simulation_time 34.9999992s 区分。

原始 `states.json`、`episode_arrays.npz`、`physics.npz`、`physics_contacts.json`、`policy_records.json`、`camera_clocks.json`、`execution_states.json`、每帧 `sensor_packets/`、`maps/`、`local_plans/` 均保存。`stereo_view.mp4`、`third_person.mp4` 为实际当前相机与第三视角。初始 `artifact_manifest.json` 范围是 child close之前；parent在 child退出后保存 `artifact_manifest_post_close.json`，**2762项**涵盖关闭后bank_feedback，避免把初始manifest冒充全部关闭后输出。

## 失败分段与有限归因

| 阶段 | 时间 | 路程m | 独立参考进度增量m | 进入→退出横向误差m | 有效tracking裁剪 |
|---|---|---:|---:|---|---:|
| 零请求采集 | 0–6 | .106468 | .033762 | .000001→.000747 | 不适用 |
| 正向tracking | 6–16 | 1.769036 | 1.176324 | .000747→.986745 | 71/100 |
| 原换向制动 | 16–18.5 | .292064 | .192569 | .986745→1.069330 | 提前返回，无有效tracking |
| 倒向tracking | 18.5–35 | 3.723089 | 2.764643 | 1.069330→.025602 | 115/165 |

观测到局部目标的世界y符号两次切换：**7.5秒 -1.4→+1.4m**，**16秒 +2.2→-2.0m**。对应当前/历史hazard格为39/2→40/4、30/29→30/31；这是来源地图更新伴随路线改变的实测关联，不证明某个新增格单独导致了失败，也不把世界y符号直接认定为拓扑类的形式证明。后一次切换时请求方向变为倒向，实际锁存方向在18.5秒才切换；全程仅一次实际正→倒转换，**没有反复换向证据**。2.5秒制动包含真实运动/五低速确认，不能预设为实现bug。

265个有效移动tracking样本中，协议裁剪 **186/265=70.1887%**；BankMap曲率夹紧183/265；V1支持外 **89/265=33.5849%**，|请求曲率|中位数 .899523/m。三者层级不同，不混成一个“转弯失败率”。旧成功Oracle的完整路线、无6秒前缀和任务域不同，不据其裁剪数量做因果A/B。

每次重建参考都从当前位姿起始，58个0.5秒局部窗口退出误差RMS .027668m、峰 .080303m，不能据此声称长期跟踪好：索引/投影重置会让新路径入口误差变小。`depth_local_curve_failure_audit.json` 另外保存6/10/15/16/20秒安装路径的固定5秒描述窗口，不用候选索引评价成功；这些参考在线已被替换，不能冒充另一未执行计划的真标签。

目前明确的是 **局部参考连续性与执行匹配层未通过导航验收**，并非数据同步或搜索无解。绕行侧选择变动、长期曲率裁剪、6秒前缀及短时重新起始参考均有实际记录；各自对超时的贡献未通过受控干预隔离，不能直接断言是PID、换向或单个规划bug。

![实测轨迹与指令链](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/depth_local_curve_failure.png)

## 地图与时延边界

frame349/time34.9：当前floor31、历史floor1715、当前hazard63、历史hazard163、unknown158028；历史hazard与离线二维occupied重叠proxy102，floor-overlap proxy0。格数不是独立实体/通行面积；二维proxy不认证完整机体扫掠。与上一ORB来源shadow的floor始终0相比，特权深度层确实产生了可追溯的floor记忆，但这不等于RGB恢复这些观测，source gate仍拒绝。

policy wall p50 .002770s/p95 .940341s；58次规划 p50 .797596s/p95 1.243388s；capture partial p50 .259763s/p95 .297808s。因果在线、物理10Hz动作块与 **wall实时10Hz** 是不同声明，本次不满足后者。capture partial也不冒称完整端到端时延。

## 测试与作者验证

测试先RED后GREEN；新增8个行为fixture，总套件 **148 passed / 4.39s**（`depth_local_curve_suite.log`）。覆盖输入权限/复制、未知不seed、真实hazard阻塞、真实tracker pending/五低速状态跨参考替换、局部非零动作、未来payload不可读和失败规划0.5秒有界节奏。

保留作者过程错误：首个几何fixture误以为冻结V1 aux0严格直行，实际 k(.5/1/1.5,0)=-.04342/-.06038/-.05369，仅纠正测试为原预定.2m终点容差，没有改V1/gate；首个重试fixture hazard .6是合法边界，未产生RED，随后用原clearance内部.4位置才复现失败，去掉 planNone触发逐帧重试。初始seal按双换行切ledger少了原terminal newline，归档初稿后用精确旧prefix匹配；不跳过身份差异。初始capture发生在policy.reset之前，源码核验发现packet会被reset擦除，数据前改为act边界绑定current cache。以上均发生在72源封存/唯一运行之前，失败日志保留，未据真实结果调参。

作者完整验证 **03:57:09.067008UTC**（`depth_local_curve_verification.json` / `depth_local_curve_verify.log`）：72源与2762 post-close输出SHA；351双方深度packet从同capture逐条重建；350实际请求、诊断、来源地图、安装局部参考和历史shift精确重建；cold/mid/final三个有限 **past-state clone** 对future payload替换/截断动作与地图不变。不是三个完整物理fresh-reset实验。

所有1750实际physics raw请求对齐；从首个capture（已推进5 ticks）的执行状态克隆，后续 **1745 ticks** 的实测反馈条件下 smoothed/applied及capture状态一致。最初5 ticks不在克隆覆盖内，不能把它写成1750全程执行克隆；这也不是无未来真值递归物理预测。时间0age、hiddenmarkers、双方完整capture与无尾步核验通过。初次assessment保留 `interface_gate_pending_full_replay=true` 的当时状态；随后独立作者verification是最终 `interface_gate_pass=true` 的证据，不倒改旧assessment。

## 接续任务（尚未执行）

先在本次58个完整map/reference序列上做有界的 **参考连续性干预设计与CPU几何核验**：已有局部绕行方向/走廊是否仍被当前观测支持，若仍有效则避免仅由局部重投影切换；若出现真实新hazard使旧段失效，允许明确失效后重建，不锁死路线或强制方向。将单一参考消费修改与本次B0在同起点、同传感/未知风险域、同6秒前缀和原35秒比较；先冻结验收为独立真实进度、空间检查点/目标到达、同参考横向误差和接触，不奖励停车/索引跳转。保持PID、速度、映射、planner算法、相机和gate；不重做矩阵或扫阈值。

这只是下一项待实施的有界工作，当前不声称其通过，也不额外启动仿真。若数据不能支持合法连续性候选，应拒绝并明确阻塞层，不能强行强制选一侧。特权层通过后才重新评估可靠双目RGB障碍消费、估计位姿及唯一学习增量；不得恢复已拒绝ORB模型为合格观测。

## 独立审查与作者收尾

一次 fresh-context 只读审查完成，Critical/Important/需修Minor均0。原审查正文SHA `94d7cdbfdfba68f8913f5a6606720503ef975624240730c2a362dfff24557e3d`；本段是随后作者追加，不冒称二次审查。审查者实际核验72源/2762关闭后输出SHA、350来源地图/1750raw指令/2101无接触记录，7帧双目packet、4次局部reference重建、30实测feedback执行ticks；独立8个focused测试通过(2.66s)。148全suite、完整351/350/1745重建及3past-state未来payload测试是作者证据，审查者只检查程序与输出，未重新完整重算/重跑物理/逐视频帧检查。密集源码仅可读性观察，不为此改冻结源。

作者逐条接受10项declined范围：纯RGB、估计位姿、学习增量、完整3D/停车认证、任意状态物理预测、泛化/正式矩阵成功率、历史单因素归因、失败因素独立因果、尚未实施连续性候选、首5tick执行克隆均未实现或未认证；不能由本轮接口通过外推。完整审查与每项ruling保存 `H/depth_local_curve_review.json`。作者最终封存将复核旧2513项、新72源/2762post-close输出、报告原正文与各日志身份，并append mission/主计划/ledger；不重复科学重算、不追加仿真、不改gate。最终identity/documentsync为实际完成证据，全部研究仍未完成/ACTIVE。
