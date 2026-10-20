# 原场景主动观察与完整双目来源验证：2026-10-05 00:36UTC

**实际18秒观察动作及有限响应迁移验证通过；冻结ORB的完整观测来源准入仍拒绝。整个导航研究未完成。** 此次转弯和停车可执行，历史图像带来更多正确稀疏点，但无法把大部分缺失空间补成可靠地图。没有新导航成功率。

## 预封存、身份与任务条件
采集协议 2026-10-05T00:45:39.180713+00:00 先于实际运行 2026-10-05T00:45:55.968560+00:00—2026-10-05T00:47:12.551316+00:00，退出0；37源码/历史测量身份、79上一阶段产物封存。推理输入/runner的367身份在2026-10-05T00:47:57.821019+00:00、全部推理之前封存；collector/ORB结构/gate在采集前封存，离线报表仪器不混称采集前预注册。
只运行golden_e1_0001原场景、原起点/姿态/种子，一次完整reset；6秒零请求→6秒主轴+.3rad/s、encoded副轴+30°→6秒原双零请求。保留完整配套停车响应，不拼接异初态尾迹，不追加另一动作/网络/参数。未改障碍材质或添加标定板。
相机ball_stereo_front020_up005_v2固定X=.20m,Y=±.30m,Z=.05m，640×480/f=584.888889px，标定/眼别/旧mask/照明fixed_camera_fill_v1不变。旧roll PID、protocol/smoother/supervisor、增益、动作、地图与35秒正式规则不变。实际类为动态安装roll_pid的CaptureRuntime；没有Oracle路径跟踪、视觉planner或图像决定本轮动作。
这是预注册后采集的新完整181对10Hz序列，无拟合/调参，但场景/初态来自已使用的E1，**不是fresh独立环境或新的泛化证明**。旧26选帧数据保持历史；不把两组不同请求/分母当作控制A/B。
冻结feature_tracks.py SHA `26432b42101ca47550af9efb99b1a02b813f6d934bf5fb2ec0a05d3b2c76e3b1` 原样；新finite-time caller active_observation.py `0e9ed2ec4832542fdaf5c1d021aa9f1f6efa80bd897efb3886dca2870f971705`。原场景SHA `d6236dea995f04e3a81fb90dcde4bde6b63841c283cc5b2470aee8cda34c8634`、occupancy `e61a4c4a6f30598606ae8628ce0abf95a9e92fcd9916d8ecbe91243031927bd7`。

## 开始运动前的适用性与风险
既有E1六秒cold-state的关节、局部速度/角速度、倾侧、PID积分/rate、完整smoother/protocol/上一supervisor均通过冻结适用性；最大关节速度差.0001373rad/s。实际六秒边界再次只读检查通过、完整执行状态不改；实测高度.400000542m与旧.399999976m满足预定1cm同平面约束。
完整实测12秒XY扫掠在原已知occupancy中333格、未知0、占据0；连续折线到hazard格中心净距1.23326195m，要求.77071068m（body.5+旧margin.1+原响应误差门槛.1+grid半对角）。这是**特权已知地图下采集动作的二维前提**，不是RGB地图、完整三维/任意状态安全证书，也没有把先验写成观测自由。无适用动作就会在六秒边界终止，不降门限或换另一段。

## 实际运行、同步和运动证据
180请求/181capture/901物理采样；每个capture都保存双RGB、known camera pose、离线depth、动作/姿态历史和完整执行快照（初始PID安装前None明确保留）。current RGB文件只有left_rgb/right_rgb/time_s，known-pose独立文件只有两个USD pose/time_s，depth另存offline_truth，不能误把真值包作为视觉输入。
所有181左右render时钟与physics raw clock差0，render批次保持物理状态/时钟，marker每次确认隐藏；一次base capture后固定三render图片事务，history只更新一次。两段实际视频各完整解码181帧10fps，没有旧runner的计分外尾帧。时间戳一致与过往板几何证据不能认证此次所有表面曝光；标签限制继续保留。
相同六秒前缀：17非raw-clock物理字段301采样逐值一致，61完整执行状态及历史完全一致，raw时钟/step原点差0。最初报表通配排除raw_误漏raw_command_target，因此初稿检查16字段；已保存初稿代码/assessment/log，另版精确只排除两个raw clock字段并重算。所有来源点/标签/误差/运动/gate和其余字段逐值不变，只补齐原始请求字段检查，未重采/重推理。
原开放平面整段响应在原E1中的实际XY RMS=0.000051550m、peak=0.000072081m；原六个时域的yaw误差最大.000694rad、bank误差最大.000632°，原.10m/.15rad/5°门限均通过。实际五次10Hz XY速度<.05m/s资格为14.2s，与原响应相同；1081实际contact样本可用且接触0。不是任意末态可重复primitive证明。
独立冻结实测折线长0.998972m，实际沿此参考进度0.998907m，横向RMS 0.000038740m、peak 0.000067787m；真实XY路径0.998977m、净位移(.805595,-.165701)m。这里完成的是一个观察段，**不是完成E1目标或正式导航参考**；观察18秒若用于正式任务仍计入35秒。
保存源码/完整执行配置及关节状态，未新增DOF/gain物理读回或全robot资产依赖审计；身份结论限于已记录源码与本次前缀/完整响应，不额外认证全部历史runtime。

## 冻结视觉结构、完整分母与结果
362个current/history预测全部保存2026-10-05T00:48:04.371701+00:00后才评价标签2026-10-05T00:54:30.170356+00:00。两臂同一ORB/256query/极线-Hamming-ratio/正深度-视差-重投影-像素扰动敏感性/来源一致结构；history只有最近三对严格过去双眼，内部不清空记忆。没有读取未来camera/depth/反馈、编码ID或场景几何提供真对应。
NaN reference/provenance旧Minor由新caller guard拒绝，真实有限输入的预测与原冻结函数逐值相同，旧feature_tracks.py/旧结论不改。该guard不是全面认证所有损坏pose/frame/数据结构。
所有181对的左眼query都纳入；不挑图、不去除失败角点来满足min100。原min100、正确/全query≥90%、有标签保留点三维欧氏p90≤.10m，缺失/无标签保持完整分母。

|输入|全部左query|保留|有标签保留|≤10cm|全query准确|所有保留条件准确|有标签p90(m)|准入|
|---|---:|---:|---:|---:|---:|---:|---:|---|
|current|5531|226|207|206|3.724%|91.150%|0.044577|拒绝|
|causal|5531|568|520|512|9.257%|90.141%|0.051520|拒绝|

共5531 query，其中5087有受限renderer标签、444没有，完整分母不变。历史正确点加307/丢1，净增306；402最终保留来源来自过去（568总保留），来源冲突拒绝2query。真实history输入带来更多被验证正确点，同时保留的3D定位误差p90较小；**9.257%完整准确仍远低于90%，不能用90.141%条件比例宣布可用RGB导航**。
原场景评价仍是nearest rounded pixel的当前renderer轴向depth+原subpixel ray/knownpose，包含边界、混合像素、无效深度/面几何限制；独立body-ID分区不可获得，world-Z<.08m floor/elevated_or_other分开保留，不能认为mask就是真body标签。旋转方形支撑不证明ORB所有金字塔滤波依赖均无机体污染。sigma_m仍是单像素扰动敏感性，不能保证真实对应或位姿误差界。未拿真值选源/滤坏点。
这个原E1完整序列有更多小误差点，不能推翻旧hard87query/唯一11m错误、板/V1/Fast/M2或其他拒绝，也不能和旧16选帧数据直接比较总体提升。唯一ORB结构整体旧拒绝保持；本序列来源gate亦拒绝，不触发控制消费者、学习增量或正式E0/E1/E2。

## 记忆、测试与失败层次
终态249稀疏记录，其中203有标签/200≤10cm、46无标签、3已测错误，247来自旧query。这不是去重实体点/地图面积或全盲区覆盖；未知没有变成free，certified_free始终false。少量错误和无标签点仍进入并存活在结构中，不能把容器实现通过当作可靠记忆准入。
四个手算坏reference/provenance拒绝、完整匹配停车扫掠/unknown拒绝、真实关节与当前yaw迁移测试RED4→GREEN；完整128CPU测试2.40s通过。作者最终验证全部181特征对/362预测、有限wrapper与原函数等价、未来payload替换、记忆/历史移位/前缀/实际请求/版本，核验文件另存。最终一次独立审查范围另见记录，不把作者验证器当自动获得审查背书。
Isaac日志保留原Hydra displayColor/不可识别widths与静态collision geometry警告，实际capture/guard/请求/物理/两视频全部完成且退出0，不因此称采集启动失败。没有新网络训练，纯视觉闭环、estimated pose、学习记忆、正式35秒E0/E1/E2均未运行。

## 图和实际视频
![实际执行与停车](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/active_observation_motion.png)
![实际双目观察变化](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/active_observation_motion_rgb.png)
![完整来源与记忆记录](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/active_observation_sources.png)
六个预定事件双目图：[active_observation_stereo.png](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/active_observation_stereo.png)；[双目完整视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/active_observation_run/stereo_view.mp4)；[第三视角完整视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/active_observation_run/third_person.mp4)。左眼离线颜色仅表示本次query标签：灰未保留/绿≤10cm/红>10cm/橙无标签，右眼原图。

## 结论与下一项有界消费接口
执行/同步事务接口和本cold初态完整响应通过；来源整体覆盖拒绝；空间记忆只保存有限点且有污染；没有新导航结果。失败不能归结为“机器人这段控制跟不上”或“只再换一个网络”：本次机器人确实执行了预定观察动作，仍缺大量有验证标签的可靠观测。
结束追加观察动作与描述子/阈值分支。下一项只做**本181对数据的只读稀疏来源地图→完整扫掠支持消费验证**：将真实预测点、双眼/时间来源、像素扰动与pose层次显式送到独立地图，unknown/历史/当前/predicted区分；当前-only与history在同一冻结实测路径上检查真实缺支持位置、错误free/障碍持续和过期。固定预算181帧，不再采第二运动、不重推理另一方法、不用真值在线过滤或seed脚下自由。它是拒绝候选的诊断shadow，不改变动作、不继承可靠视觉导航准入。
先回答缺失点是否正好位于本机器人下一步及停车扫掠、是否只是无关壁面角点，不能把all-query分数直接当空间面积；如果此任务空间依然不足，给出具体观测/冷启动条件阻塞，停止这一分支，不继续无限局部审计/网络扫描。只有受控来源和消费者实际满足各自门限才进入改变动作的闭环；estimated pose/唯一学习增量/原35秒评测仍按总计划分层验证。全研究未完成，automation保持ACTIVE，常规不发完成通知。


## 最终独立审查与作者封存范围
一次fresh-context只读whole-stage审查 `/root/active_observation_review`：Critical/Important均无。审查正文SHA `490c2564b823b6044eb0fc4aaa5cfb9ecadb145f439c2c8252a992ba66914ac5`；本段为其后作者追加，不冒充二次审查。独立128CPU测试2.53s通过，37采集源/79旧产物/367推理输入身份核对，181双RGB与362记录的namespace/query/实际source眼别帧时间和最近三过去双眼链、独立射线标签和全部5531/5087/226/207/206/568/520/512计数/p90/+307−1、181记忆、180history shift与61旧history、180请求、19字段301同前缀/181状态/61执行快照一致。另从首个安装后快照，用当前实测反馈重建895 applied/smoothed物理步和179后续执行快照逐值一致；初始None及最初5步不隐去。
审查独立核验完整333格扫掠/1.233261949m净距、原匹配转弯与自身停车、完整XY与六时域/14.2秒五次低速资格、1081可用非接触、181同步/隐藏markers、两视频完整181帧10fps，以及初16/末17字段报表非前缀科学量完全相同；检查三张实际图。没有重提ORB或重新执行模型推理/采集/训练；362预测重建/有限等价/未来payload替换仍为作者证据，不混作审查者独立运行。历史执行时间的外部证明、全部机体/曝光/资产认证也不在审查范围。
Minor：初次作者核验目录把仍被stdout写出的 `active_observation_verify.log` 按空文件SHA记录；1129新身份只有这一项失配，源/输入/数据/结果与gate不受影响。初次JSON和log保留并明确这项限制；最终作者输出到验证器已有排除的 `active_observation_verify_final.log`，重新散列已写完的初次log，最终log在文档同步记录另行封存。未改候选、采集、验证器或gate。审查者没有执行随后最终封存，最终核验文件仅是作者证据；无第二审查/重采/重推理。
下一只读消费者诊断沿用已评价过的这181帧，设计已接触本阶段标签，属于历史开发诊断，不能称fresh held-out；先冻结消费者语义/预算与仅观测输入，保存全部消费者输出后才重新读标签核验。此报告明确拒绝来源准入，全研究未完成。
