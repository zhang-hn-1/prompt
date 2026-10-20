# 未资格换向投影修复：原35秒实际闭环

2026-10-05 09:45UTC自主阶段。唯一CuspProjectionPolicy候选实际运行结束；接口核验通过，但导航和两条对照的限定改善均拒绝。机器人开始真实倒车，避免了前轮未停稳就跳过倒车请求的接口错误；全任务的进展没有改善，不能把这项修复准入成成功导航基线。整个研究仍未完成，自动续做ACTIVE。

## 范围、身份与预先冻结

工作区 `/home/an/Documents/ChatGPT/球型机器人导航`，H=`experiments/stereo_memory_navigation_20261004_v1`，原项目只读。独立spec/plan为 `docs/superpowers/specs/2026-10-05-cusp-navigation-design.md`、`docs/superpowers/plans/2026-10-05-cusp-navigation.md`。旧Round结果、source/RGB/M2/停止包络拒绝保留。无新模型拟合、标定矩阵、参数扫描或正式E0/E1/E2。

CuspProjectionPolicy继承RootDirectionPolicy，只在episode reset安装冻结make_cusp_oracle；outer act/plan_boundary与RootDirectionPolicy同方法，inner act/direction handler与原Oracle同方法。实际继承链由 `H/cusp_navigation_identity_audit.json` 的源码SHA/MRO保存：ConfidentSpeed → MappedBankOracle → CuspBoundary → OraclePathPolicy。不能用B0名称代替身份。候选相对前轮root_direction_run唯一行为改动：当前reference方向与accepted方向不同期间保持当前投影索引，资格后继续原投影。合法新参考仍可取消旧pending，不永久锁存旧几何、不强制反转、不修改五次规则。

`run_cusp_navigation.py` 相对前轮只机械替换policy构造/import、protocol/output/completion名称，五处diff保留于 `H/cusp_navigation_runtime_diff.patch`；父runner对应机械名称替换。冻结cusp_projection.py保持前轮原字节。旧root planner入口允许initialized None的Minor未修，正常实际运行已初始化F/R；投影层非法None拒绝不能冒称所有入口已修。

相机固定X=.20/Y=±.30/Z=.05m、640×480/f=584.8889px/原光照自mask、hidden markers、capture事务；mapper真实双深度+knownpose与过去来源记忆、显式unknown risk，无脚下free先验/ray carving/GT滤图/在线全图或全局评估参考。outer privilegedFalse只拒绝Oracle全图上下文，不代表RGB估计位姿导航。原root switch=.5/reverse1.25/node3000/primitive.3/sample.05/goal.2/localtarget2.4/crop±4/clearance.6、guard连续+raster.6/剩余≥.5/投影≤.5、roll PID(.4,.1,.7)/速度(.5,1,1.5)/副轴supervisor.3、protocol/smoother/五个10Hz当前XY<.05与反弹清零均不变。

`H/cusp_navigation_protocol.json`于09:49:05.318957UTC封存119源与8509旧/病例输入。每条对照均沿用原限定改善门槛：独立参考进度增加≥.05m、最终goal距离降低≥.05m、横向RMS/peak≤对照+1e-5、实际contact0，五项同时通过。真实导航goal≤.2m、原35秒、实际contact0。初始6秒零请求也计入35秒。root臂已经被拒绝，只是同控制器因果比较器；较强guarded臂也必须报告，不能赢过更差比较器就准入baseline。

## 唯一实际运行与共同前缀

`H/cusp_navigation_status.json`：09:49:35.045858→09:52:28.764418UTC，exit0；golden_e1_0001，35秒timeout，距goal4.756418677m，实际contact0。350请求、351capture、1751physics、2101contact采样；两视频均351帧/10fps，无计分外物理尾步。post-close manifest于09:52:29.168468生成，2752产物，包含SDK关闭后写出的反馈；pre-close manifest范围另保留，不能替代post-close检查。

相对归档root的第一raw请求分叉是frame182/18.2秒，此前911物理样本17非raw字段、183完整执行快照、183双边packet及history共同前缀精确一致，含关节、root、smoother/PID/执行状态。raw初始化时间/step原点差-.02秒/-1tick单独报告，不混为零。分叉后141raw动作块、840applied ticks真实改变。较强guarded是另一历史控制对照，未宣称它也与候选共享到182的全状态前缀（root干预此前在175已分叉）。没有再次跑旧baseline。

CPU前阶段冻结11真实fixture RED7失败4通过→GREEN11通过，fullH195通过/8.72秒；本阶段复用原测试证据并核验源/输入SHA，没有冒称重新跑195测试。新实际运行和完整实际重建是本阶段新增的主要验证。

## 真实停稳、倒车与短段几何

离线描述脚本 `H/audit_cusp_navigation_motion.py` 于封存后编写，只读取结束的本次实际日志；不属于控制改动，不新增验收门槛。`H/cusp_navigation_motion_audit.json` 保存完整阶段表。方向接受、负raw/applied、关节负转与机体运动分开核验。

|接受时刻|方向|当时XY速度m/s|此前五个当前10Hz采样均<.05|随后主轴raw|
|---|---|---:|---|---:|
|19.7s|F→R|.022827|是|-.5|
|23.7s|R→F|.022757|是|+.5|
|29.7s|F→R|.036898|是|-.5|
|33.3s|R→F|.012646|是|+.5|

首段19.7s的五个速度为 .036090/.017073/.004963/.014084/.022827；不是关节过零代替停稳资格。181保持R pending/low0，182继续R pending/low0/零请求，196为low4，197实际接受R、负请求。197当时实际main关节仍+.040957、previous applied0；215已实测main-.460650、previous applied-.5/XY朝倒车段运动，因此接受并非瞬间实际换向。本次全程负raw180ticks、负applied278ticks、实测main负428ticks，各自不同分母。

首已安装reference version3（17.5→25秒）反向名义段长.298971m，实际轨迹距起点最小.003763、距终点最小.044042m，沿该段有向投影范围[-.037339,.359736]m、结束.277372m，实际沿段速度最大+.241445/最小-.319773m/s。version6（26→35秒）反向段长.297952m，距起点.010935/距终点.013268m，投影范围[.000316,.390492]、结束.245934m，沿段速度最大+.207071/最小-.216441。此为自己已知参考寿命内连续几何，证明有实测倒车方向位移并靠近相应端点；不是索引跳转冒充通过，也不是另一计划实际标签。未在看完数据后另定“primitive成功”二值gate。寿命内负tick统计属于整个安装窗口，不专属于某一个primitive。

18.1→19.7首次制动确认1.6秒；19.7→21.6倒车tracking1.9秒；21.6→23.7回前向制动2.1秒。第二轮27.9→29.7制动1.8秒、29.7→31.4倒车1.7秒、31.4→33.3回前向1.9秒。总74direction-change零请求块=7.4秒，保留合理制动耗时，不称bug或解除安全资格。

## 独立导航评价与阶段定位

统一冻结完整几何评估参考，独立于候选path_index；无投影口径切换。全部是实际同起点完整35秒日志，旧对照未重跑。

|实际臂|终态沿参考进度m|距goal m|横向RMS m|横向peak m|contact|结果|
|---|---:|---:|---:|---:|---:|---|
|较强归档guarded B0|4.855576|4.156980|.174067|.475266|0|原timeout/拒绝保留|
|归档root-cost因果比较器|5.389775|3.671378|.572390|1.105993|0|原限定改善拒绝保留|
|本次cusp投影|3.707562|4.756419|.135045|.260893|0|timeout/拒绝|

相对root进度减少1.682213、goal距离增1.085041；相对guarded进度减少1.148014、goal距离增.599439。两条的误差/接触三项通过，但进度/goal两项失败，所以两条限定改善均拒绝。RMS下降不能覆盖进度损失，也不是可靠导航证据。

冻结2/4/6/8m空间点、.3m半径：候选2m于13.9秒到达、4m于27.4秒到达、6/8m均未到达，最小距离1.990377/3.858112m。root4m19.7秒、guarded4m25.3秒，候选更迟。候选最高进度3.896759、终态3.707562m，不以最高值代替终态。

|本次阶段|持续s|独立沿参考进度变化m|进入/退出误差m|
|---|---:|---:|---|
|6.0→13.5 F tracking|7.5|+1.659951|.000747/.227996|
|18.1→19.7 F制动|1.6|+.140999|.157975/.257715|
|19.7→21.6 R tracking|1.9|-.176143|.257715/.126987|
|21.6→23.7 R制动|2.1|-.168326|.126987/.059426|
|23.7→25.0 F tracking|1.3|+.074880|.059426/.020562|
|25.0→26.0 local-no-route|1.0|+.136725|.020562/.079406|
|26.0→27.9 F tracking|1.9|+.171792|.079406/.072813|
|27.9→29.7 F制动|1.8|+.055116|.072813/.251111|
|29.7→31.4 R tracking|1.7|-.103418|.251111/.119121|
|31.4→33.3 R制动|1.9|-.201025|.119121/.166654|
|33.3→35.0 F tracking|1.7|+.117879|.166654/.107225|

本表由实际请求来源+accepted方向分段，末点为下一阶段动作前状态；全表含初始观察、短goal-braking等保存在motion audit。独立完整参考的倒退不等于自己的R参考未执行。初次换向不能单独解释全程失败。

真实边界58次：52retain/6rebuild，其中4成功、25/25.5秒两次observed_obstacle_blocks_route；guard不保留原因remaining_arc_under_lookahead2次/original_raster_clearance1次，retain状态保持。26秒重新成功安装version6，再有F/R/F；没有持续沿完整路径走出这一区域。不能为了少交接强制forward，也不能删历史观测让route成功。

`H/cusp_navigation_selected_chain.json`保留当前诊断有效性及实际量。例如version3 R tracking19.7→21.5，root heading -1.178943→-1.224909rad，idx1参考body heading -1.025005；21.6进入下个F参考heading-.737970，实际-1.232894，23.7接受F时仍-1.198351，heading误差+.460381；24.9误差仍+.433406、aux raw-.568909/protocol-.523599。这里是重新前进时航向欠对准的实测证据，尚不足归因于符号、PID或稳态映射某一层。正常tracking188块，协议裁剪98/188=52.1277%，合法角目标到边界同分数，V1支持外88/188=46.8085%，不拿陈旧braking字段混进分母。

## 因果与执行链验证

作者 `H/verify_cusp_navigation.py`于09:54:10.961008UTC退出0、结果 `H/cusp_navigation_verification.json`：119源/2752关闭后产物SHA；350实际policy请求、全部map/localref/diag精确重建；351双深度packet、350history各仅一次；1745实测反馈条件下protocol/smoother/supervisor/applied精确，初始5ticks不克隆；3个finite-past checkpoint的future trap/truncation一致。旧8509输入由封存/最终作者hash检查核验。重建只用当前/过去及当时已知参考，未向候选喂旧臂分叉后真值。不是递归物理预测，不是3条fresh物理验证。

初assessment的interface_pending保留，后full-verification接口PASS另立结果；导航False/source-RGB拒绝/full3D未认证保持。实际低速/倒车量是本次自己实测日志，不是监督预测成绩。

policy p50/p95=.002628/.008070s，planning p50/p95=.504918/3.444218s；capture partial p50/p95=.285339/.310600s，尚非完整端到端。因果在线接口成立，wall10Hz不成立。SDK存在继承hidden marker primvar widths/displayColor警告，原日志完整保留；实际exit0/markers hidden，不将其写成仿真失败或用户阻塞。

## 图、视频与证据入口

以下均用本次实际视频/状态，已视觉检查；17.5s为干预前共同状态，35s为本次终态，非合成候选未来轨迹。

![三条实际轨迹与统一评价](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/cusp_navigation_comparison_v2.png)

![实际双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/cusp_navigation_stereo_view_frames.png)

![实际外视角](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/cusp_navigation_third_person_frames.png)

原机械plot保留错误的继承root title/候选标签，不能作身份依据；封存源/旧PNG不改，独立post-run `plot_cusp_navigation_labels_v2.py`只纠正名称并加入真实root对照，报告引用v2。图不是单独导航gate。

实际视频： [双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/cusp_navigation_run/stereo_view.mp4)、[外视角](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/cusp_navigation_run/third_person.mp4)。完整physics/执行/camera/history/depth/RGB/memory/localplan/contact存于同run；身份、raw SDK stdout、seal/assess/verify日志和独立gate保留H。复现入口spec/plan+presealed protocol，不以旧全局参考作在线输入。

## 下一项有界任务

本heartbeat不再search/物理/fit/换candidate。下一阶段优先对version3和version6的实测R tracking与换向后F tracking做一次CPU执行链几何核验：已知reference body heading/profile、PP/feedback曲率实际合成、V1请求方向与请求/roll反馈/applied、实际yaw与XY course分别列出；尤其23.7s后约.46rad和33.3s后欠对准为何只能小幅恢复，以及短R名义yaw变化是否在现有执行链中自一致。用字面CPU直线/圆弧/正反向与同向连续性fixture复现可证错误，不能默认符号bug；不二次act，不沿用braking陈旧诊断，不再改五低速/PID速度阈值。

有确定错误才形成一个最小传统tracking修复；无错误则在冻结范围内拒绝该假设并明确实测动态欠对准/来源route阻塞各自证据，不靠无限扫PID或M2/M3补拟合。下一物理候选必须另行事前封存、同初态原35秒、相同独立进度/goal/横向/contact gate，不能靠放宽导航时限或停车得到通过。完成诊断不代表RGB、估计位姿、学习记忆/世界模型及正式E0/E1/E2已完成；它们仍待分层推进。

## 独立审查与作者收尾

ONE fresh-context只读审查 `/root/cusp_navigation_review`（gpt-6-astra），0 Critical/0 Important/2 Minor，裁决可归档为REJECTED诊断证据，不能准入导航baseline。五条Review Focus均满足。审查body SHA `ef9f8d28596758816edd89ce202ad76cdf21c019cdeb1f3c4b0fc070af81c33a` 保持原字节；当前作者append、最终identity与continuity文档另作作者证据，不冒称第二次审查。

审查者亲自重新核验119源/8509旧输入/2752实际产物及前轮27/44阶段产物SHA、机械runtime/parent差异与三版同gate、911×17/183前缀、141raw/840applied变化、三臂几何/检查点、四次五低速、180/278/428负量、两段连续实际几何、188tracking中98裁剪88支持外、58边界52retain6rebuild、zero-age/markershidden；允许的11无搜索真实fixture新跑11PASS/1.34秒，禁bytecode/pytestcache。作者350policy/351depth/350history/1745实测feedback/3finitepast结果只读检查，未由审查者重跑；195/8.72与RED/GREEN保留历史。审查者只看纠正比较图和哈希视频，未逐视频完整decode；作者decode帧数与选取frame175/350视觉检查另属作者证据。

两项Minor单独裁决：初始化None入口校验是继承问题，本次实际context有效，保留另维护不修改封存控制。`assess_cusp_navigation.py:61` / assessment checkpoints中的旧 `arm=B0` 实指 **root_direction_run 因果比较器**（4m19.7s），不是正文/图中的较强 **guarded_reference_run B0**（4m25.3s）。原assessment/source/旧label图不改；新增 `H/cusp_navigation_checkpoint_alias.json` 仅显式别名，不重算分数或gate。所有原分数与正文表格保持。

17条declined-to-judge范围在 `H/cusp_navigation_review.json`逐项保存原理由、作者裁决及误判成本：fresh物理、350fullpolicy、双depth/map、1745执行复算、195旧矩阵、穷尽future、递归物理预测、RGB估计pose、学习增量、正式generalization、3D动态安全、二值primitive通过、航向亏损单因果、参数最优、视频全视觉、wall10Hz、最终closure/scheduler。各自保留证据层次限制，不被合并丢弃。最后一项由作者随后执行行政关闭/hashcheck、另只读核验scheduler snapshot ACTIVE，不改变scheduler；首次元数据读取因Python3.10缺tomllib失败无写入，随后仅读取文本id/status/name成功，不作为科研运行失败。

Task3作者保存review/别名/全部报告与数据，并通过close_cusp_navigation_stage.py和check_cusp_navigation_stage.py作最终fresh源/旧输入/实际产物/文档append-only/SHA/时序检查；最终实际输出见H/cusp_navigation_final_check.json，只有其成功才称本有界阶段归档完成。整个研究未完成，automation ACTIVE，下一项CPU航向/执行链原因核验保持未执行，本heartbeat不追加任何候选/search/physical/fit。
