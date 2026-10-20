# 当前相机配置的完整Oracle导航与在线双RGB只读记忆：2026-10-05 02:39UTC

**完整控制基线本次成功：30.7秒到达目标、剩余0.195486m、实际接触0。双RGB记忆真实在线运行，但仍拒绝视觉准入。** 已获得当前固定相机下的一条完整导航结果；不是纯视觉导航成功，也不是正式E0/E1/E2矩阵或整个研究完成。

## 本轮解决了什么
此前连续冷启动/停止局部诊断没有证明当前配置能走完整路线。本轮结束这条冻结ORB/18秒endpoint-support分支，回到独立的完整控制基线：当前rig/照明/采集事务下，已冻结Oracle可以完整走过E1绕障参考。将“机器人和roll控制能否完成路线”与“当前视觉是否能提供可靠信息”分开：前者本次通过，后者仍失败。没有再次调PID、速度、半径、ratio、网络或重做标定矩阵。

## 控制身份、参考与预算
实际MRO：`gated_speed.with_confident_speed.<locals>.ConfidentSpeed` → `bank_mapping.make_policy.<locals>.MappedBankOracle` → `benchmark.policies.OraclePathPolicy` → `_PolicyBase`；act仍为原OraclePathPolicy.act。保留旧BankMap、P040D070 roll PID kp=.4/ki=.1/kd=.7、lookahead .5m、auxiliary velocity supervisor .3rad/s、正式±30°协议与原smoother。MRO、act源码文件/SHA、配置和58源码/输入身份在2026-10-05T02:49:07.174097+00:00封存。源rig固定ball_stereo_front020_up005_v2，X=.20/Y=±.30/Z=.05、640×480/f=584.8889；固定照明fixed_camera_fill_v1和隐藏诊断markers。
205个旧environment目录中没有记录当前rig的Oracle结果；旧wrapper文件相同，但历史runtime/pointgoal源码两份不同、旧environment不含rig。因此不凭旧名称继承当前成功。
使用**精确缓存的旧成功30个已选参考waypoint**，对应旧hybrid决策frozen_measured_time_with_reserve/8.7m/0反转/483展开节点；不再运行hybrid完整搜索，不重写planner，不用本轮未来真实轨迹。Oracle单独允许完整地图/A*路径/全局参考；shadow完全没有这些输入。8.7m是旧arc规划声明长度，30点折线8.688761m，运行器A*评价参考8.836753m，三者职责不同，不能混成同一长度。
沿用原RunnerConfig：预算35s/350块、10Hz决策×每块5个50Hz控制、goal tolerance .2m、physical_contact_v2、无诊断settling扩展。一次reset，内部不reset PID/smoother/物理；原计分termination即结束，不执行旧video-terminal的5个额外物理步。缓存参考这是单条控制基线复现诊断，不是新planner候选或全套搜索端到端复现。

## 唯一实际运行与完整路径
本轮唯一仿真2026-10-05T02:49:22.233268+00:00—2026-10-05T02:52:07.139341+00:00、退出0。307 policy act/307实际请求、308双目capture、1536物理采样；1843可用物理及capture接触记录，全部0接触。两视频各308帧/10fps，无计分外尾帧。

|真实指标|结果|
|---|---:|
|原35秒计分终止|success / 30.7s|
|剩余目标距离|0.195486m|
|真实XY路径长|8.835802m|
|沿冻结30点参考终态/最大进度|8.657741m|
|横向RMS / peak（同几何评估器）|0.089963 / 0.233021m|
|实际接触样本|0 / 1843|

沿参考评价只用固定参考与真实XY，不依赖policy.path_index，不把停车或提前跳索引计成功。原goal entry成功不等于停稳；本轮没有运行末端settling，也不称到达后保持成功。
与旧已归档成功E1直接比较：307个动作及308个XY位置全部逐值完全相同，11个共有yaw/速度/角速度/关节/执行/roll/相对与raw时钟遥测数组也精确相同。于是当前真实控制行为复现旧控制结果，rig/照明/采集记录差异本次没有改变这些数组；**不等于两次图像相同，也不能补认旧rig身份**。这是历史确定性复现，不是新控制候选的独立A/B或环境泛化。

![完整Oracle轨迹与只读RGB记忆](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/oracle_online_rgb_shadow.png)
[本次双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/oracle_rgb_shadow_run/stereo_view.mp4)；[本次第三视角视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/oracle_rgb_shadow_run/third_person.mp4)。灰色障碍仅离线已知地图绘图，不是RGB记忆输入。

## 真正在线、只读RGB接口
每次真实capture完成一次既有采集事务后，立刻用当前左右RGB、K、当前已知左右相机位姿，以及最多3对过去RGB特征做同冻结ORB匹配与三角化；每帧prediction/feature/map在下一次policy act前实际落盘。全部308次来自此次实际运行，不是旧日志回放冒充在线。
新shadow接口严格只接收frame/time/left_rgb/right_rgb/left_camera_usd_world/right_camera_usd_world；拒绝额外depth、reference、map或control字段，数组复制隔离。有限时间、严格连续frame/单调时间，过去源才进入history；future包只读时间、不给payload。known camera pose明确是特权诊断层，**不是RGB+estimated pose**。
PointSupportMap仍是独立10秒原query时间的provisional endpoint代理，fixed[-20,-20]、400×400/.1m坐标metadata，no interpolation/ray-carving/body seed/truthfilter/imputed area。Floor/hazard定义与上一轮相同，sigma只是像素扰动敏感性；全图/Oracle参考/目标/请求从未进入shadow，无动作授权。保存offline_truth仅作随后有限标签，在线shadow不读；shadow与offline标签分文件命名空间。
CPU实际Oracle同输入shadow on/off动作和policy状态已验证。随后作者全308帧重建/307次真实请求回放和三个真实RGB prefix未来替换/截断范围单独记录；这些是作者核验，不预先冒称独立审查完成，也不是第二条on/off物理仿真。

## 独立视觉结果：完整运动也没有解除来源问题
所有live prediction及输出manifest完成后（2026-10-05T02:52:05.089309+00:00），才由独立assessment于2026-10-05T02:54:21.166748+00:00读取renderer nearest-pixel depth标签。全分母28120个左ORB query，24742有限标签、3378没有有限标签；不是所有像素/可碰撞体积面积，也不是fresh环境泛化或随机训练测试拆分。保持原90%/.10m/p90≤.10/min100必要门槛。

|来源|保留query|≤10cm正确query|全query正确比例|条件p90|
|---|---:|---:|---:|---:|
|当前双目|756|426|1.515%|0.167196m|
|当前+严格过去|2044|1096|3.898%|0.170100m|

过去源参与1419个保留query、正确query净增加670，不是去重实体/地图面积/闭环成功。两臂门槛均拒绝；旧ORB/Fast/V1/M2和前一stage结论不追改。
末态917 memory记录，688有标签，其中352正确/336错误，229无标签、912过去query。错误会进入并保留记忆，不能用真值在线删掉它们，也不能把保留记录多叫可靠记忆。
308地图floor格始终0、hazard代理峰值98末态81；完整冻结扫掠1521格最少1505 unknown，末态1517 unknown/4 hazard/0 floor，所有308时刻支持0次通过。对全参考的固定空间检查不意味着每一当前状态都适用该完整请求计划；也不要求提前看到未来整条路作闭环准入。**这次endpoint表示连floor都没有提供**，具体来源不足保持，不用实际已走过/无接触反写free。这些都是2.5D代理，不能把错误记录计数与真实误障碍实体/碰撞一一等同。
nearestpixel renderer/亚像素ray、混合边界、bodyID不足、ORB完整金字塔滤波支持未认证等旧限制仍在；这不是所有像素、所有固定rig视觉方案永久失败的证明。

## 耗时与实际验证证据
在线feature/匹配/地图计算p50 .049362s / p95 .160412s；包括既有render事务及capture保存的采集时段p50 .318806s / p95 .449269s。本例保证在线因果处理，**不保证现实10Hz实时**；原35s是物理计分预算，并非墙钟完成耗时。实际运行墙钟约165s，包含初始化/视频关闭等；capture时段数值不包括随后offline-depth存储与policy-input记录全部成本，不能称整系统end-to-end延迟。
五个手算/真实Oracle接口fixture RED5→GREEN，全部H CPU140测试2.76秒通过；拒绝truth字段/复制隔离、未知不seed、坏时钟不改状态、未来payload、缓存参考反向/终态命令与真实Oracle on/off动作/锁存一致。实际运行的零龄时钟、308预测与地图/特征/来源、原动作、控制状态与原参考身份随后完整核验。
保留两项**数据采集前作者工具失败**：复制嵌套capture类时缩进parse错误，按原缩进规范修复并保留初稿；封存检查发现ledger新stage追加使上轮全文SHA变，原字节prefix精确归档/旧579产物身份仍匹配。二者均在唯一仿真前解决，未改模型/数值gate/旧结果或重跑物理。
实际shadow不访问未来pose、真实applied或supervisor输出来提高成绩；没有学习模型拟合/训练或新的导航成功率矩阵。完整数据、源码、RED/GREEN日志、protocol、实际status、视频、图和SHA保存在 `/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1`。独立审查/作者最终核验按各自实际范围另记。

## 结论与下一项有界工作
当前控制上限已经有完整成功证据，后续不再退回停车/roll PID/速度阈值扫描。冻结ORB与endpoint冷启动分支明确关闭；视觉从完整运动中保留更多点，可靠性与地面支持仍未通过，不强行融合为可用世界模型。
下一项应把此次已复现的传统曲率/roll跟踪组件接入**独立的特权depth+known-pose局部来源记忆导航基线**：仅来源map规划局部路，全球goal允许、全图/全局Oracle参考不进入局部感知消费者；清楚列出原显式unknown-risk任务条件，unknown绝不写成观测free。先CPU核验实际局部参考/曲率/请求及接口身份，冻结一个完整控制package、初态与35s预算，最多一条E1验证；不能称它是原InformationPolicy的一个单层修复或纯RGB。旧停止包络不重新认证，不能在不适用的异状态下强拼短响应。
该上限若通过，才依据已分层的可靠障碍/未知任务条件返回RGB消费者与estimated pose；已有已知地面/safe-start若采用必须明示prior任务，不能偷偷冒充视觉。本轮没有正式E0/E1/E2矩阵、estimated pose或学习/世界模型增量；总研究仍未完成，automation ACTIVE，常规不通知全部完成。


## 独立审查与作者收尾

一次fresh-context只读整阶段审查已经结束：Critical 0、Important 0、Minor 1；范围与实际检查保存在 `oracle_rgb_shadow_review.json`。独立运行140项CPU测试（2.89秒），检查58份封存源码/输入与2478项初始manifest身份；独立复算实际路径、全部有限query标签、记忆计数、1521格扫掠、地图拒绝与延迟，核验全部308帧来源时序、实际MRO、接触、时钟及两视频，并重提取首/中/末6张RGB特征。独立审查没有重新运行仿真/训练，没有执行作者全量重建器，也没有独立重做307次policy回放或全部308次特征预测；这些明确属于作者验证，不能合并冒称独立执行。

作者完整重建于2026-10-05T02:58:19.550166+00:00退出0：58个源身份、2478个初始产物身份、全部308帧实际RGB特征/预测/地图、307次实际同输入Oracle shadow on/off动作与诊断/映射状态、3个真实cold/mid/final prefix未来payload替换及截断均通过。后续核验清单包含2508项实际源/产物身份；本轮没有第二条物理on/off对照，没有额外实验或修改科学输出/gate。

**Minor范围说明：** 最初2478项manifest在 `runtime.close()` 之前保存；继承的close随后才写1535条 `feedback/bank_feedback_golden_e1_0001.json`。因此初始清单不能称关闭后全部文件。保留冻结runner与原清单；该反馈已经纳入作者2508项核验，SHA为 `c3c56ea6ad00be94c6d4abf597b2eebdbccb30532c0b0c2757665987dde89ea2`，另保存 `oracle_rgb_shadow_post_close_supplement.json` 和最终身份封存。此补充是关闭后的来源记录，不补称采集前/标签前封存，也不改变本次成功或视觉拒绝结论。

独立审查正文SHA为 `2093868fe8fd5ef3b114007207a17b95d6d9e7faf50500b7ab9f009d22c10cbf`；本节是审查后作者追加，并非已独立审查正文。最终报告/产物身份与mission、总计划、progress同步另存 `oracle_rgb_shadow_final_identity.json` / `oracle_rgb_shadow_document_sync.json`，冻结本轮新计划不追加改写。该有界阶段完成，全研究仍未完成，automation ACTIVE；下一项按上节开展独立特权深度局部基线，不解除RGB旧拒绝，不新增本轮正式成功率。
