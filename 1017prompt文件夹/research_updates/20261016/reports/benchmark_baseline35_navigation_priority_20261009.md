# 传统双目记忆基线：导航评测优先

用户明确传统baseline不必先达到很高的逐点深度正确率。本阶段保持旧来源拒绝及原始gate，另立导航评测：固定相机/光照、双目方法、显式记忆、局部规划器、BankMap与roll-PID；不通过削弱baseline保证新方法赢。已知位姿、平坦地面与安全起步先验明确标注，不是完整纯视觉或已观测自由。最终主要看真实到达、碰撞、进度、耗时，来源正确率仅诊断。

|场景|证据|结果|目标剩余m|目标距离减少m|路径长m|可用contact检查/碰撞|
|---|---|---|---:|---:|---:|---|
|golden_e0_0001|新实际运行|early_stop|8.050870|0.788248|0.979740|1201/0|
|golden_e1_0001|历史封存前35秒|timeout|5.288328|1.311672|3.729316|2101/0|
|golden_e2_0001|新实际运行|early_stop|8.114051|0.785949|0.980948|1201/0|

E0/E2实际stage62883正常CLOSED0，各200请求/201capture/1001physics，因FrontView交接在20秒失败，没有执行后续路径跟踪，不能说完成了35秒或导航成功。每条两视频201帧@10fps实际解码；作者查看E0轨迹图，未看完整视频。E1仅原已闭合54.6秒的历史前35，351states/2101contacts，不是新仿真，不报告三条fresh成功率。

E0 41个双目来源epoch全无可用点；E2 41中39为空。它们说明来源输出缺口，不直接证明全部因遮挡，更不能把空观测当自由。地图脚下未知仍约186格；先验单独保留。E0/E2动作周期wall中位.376/.307秒、p90约2.19/2.15秒，不满足实际10Hz；仿真时间budget仍35，不能声称实时。

条件3测试、独立评分4测试及新增空来源诊断1测试，8GREEN；起步候选3测试+旧FrontView6测试共9GREEN。独立评测审查指出缺少盲区诊断，已补sampled空输出span/unknown-footprint并明确不是完整3D盲区测量。

唯一新局部候选只改变初始观察到planner的handoff：原6秒zero前缀后，连续五次当前XY速度<.05即可交接，不要求100个双目点；反弹清零、所有current/past源时钟与payload校验保留。不清unknown、不改PID/map/planner/映射/速度，空观测不是安全证书。ONE E0同初态35秒，actual stage20706 STARTED；启动前独立C0I0/9PASS，v2修正旧metadata与E0 CLI身份，旧v1保留。局部验收要求contact0、确实handoff/执行tracking及独立轴向进度至少比B0多.5m；仍未到goal≤.2不得称成功。

阶段尚未完成，当前候选结果待关闭核验；整体研究INCOMPLETE/自动接续ACTIVE。


## 唯一起步交接候选：E0实际关闭

20706正常退出0，350请求/351捕获/1751物理样本、实际35秒、2101个可用contact检查均无碰撞。6秒在五次当前XY低速确认后交接，290次实际非零tracking请求；旧B0在20秒停止，此候选完成35秒。目标剩余3.536651m、独立起点目标轴进度5.304243m，仍timeout，**没有到达导航成功**。

不能仅用35对20秒末态比较提升：同一20秒时刻，候选轴向进度比B0多1.699323m，实际在移动，而非停住拖延。完整35秒末态轴进度增4.514629m仅作为不同终止时长的描述指标。局部handoff功能screen通过，不认证来源、停稳动态模型、全导航或3D安全。

首请求分叉frame60/6秒，此前60request、61完整执行snapshot与14非raw物理字段（到dispatch前第300物理tick）逐值相同；初始左右RGB仍393770/344476个通道不同，因此不称整个视觉输入一致的单因果A/B。两个视频各351@10实际解码，作者查看候选轨迹与B0同图。68/71来源epoch无点，unknown脚下中位186格；成功交接主要移除流程阻塞，**未证明视觉记忆避障有效**。wall中位.389s/p90约2.206s，不满足实时10Hz。

接续同一冻结handoff规则在E2/E1的35秒迁移，最多各一条，没有第二个参数/控制候选，scene/occupancy独立正确；E1因handoff确实不同需新运行，不能把旧历史E1前缀冒充此版本。当前准备的transfer driver/protocol在执行前冻结，等待独立身份核验；不会因为本轮局部通过就称整个研究完成。


2026-10-09T04:17:24.529638+00:00 实际接续：E0 standby-handoff20706 CLOSED0，6s交接/290非零tracking/350请求，contact2101全可用0；35s goal3.536651028 TIMEOUT，matched20轴进度+1.699323489。局部起步功能通过不等于视觉/导航通过，RGB初始化差异披露。独立transfer审查C0I0，v2仅修comparison元数据后，same唯一helper E2→E1 stage34437 STARTED/status standby_handoff_transfer_status_v1.json，两条各至35s一次/no retry；源/记忆/planner/PID/速度/光照相机不改。继续34437，不重复启动。WholeINCOMPLETE/ACTIVE/noexternalblock/noGit；下一逐条seal/真实goal/contact/采集视频/延迟评测，随后以实测判断时域可达与局部执行瓶颈，不能再以来源95%作为传统任务先验baseline的准入门槛。


## 2026-10-09T04:23:39.824487+00:00 同一交接改动三场景已实际闭合

E0：35s TIMEOUT，目标剩余3.536651m，真实起终点轴进度5.304243m，无接触。E1：35s TIMEOUT，剩余5.709987m，轴进度0.898804m，无接触；后半程倒车回退，不能只用中途最大进度称成功。E2：首个真实contact25.919999s，评分位置取25.899999s之前capture，剩余5.243672m，轴进度3.662484m，COLLISION。E2 260请求261cap1301physics，E1 350/351/1751；双视频各261/351@10实际解码，source/raw seal独立核验。三条同一条件实跑均未到达，不是原严格RGB+estimatedpose成功率。6s交接均实现，只接受起步功能、拒绝可靠导航改善。E1比较旧54.6的历史前35s；各run初始RGB不同，不声称全部单因果A/B。

E2新必要核验没有用GT标签：53来源累计13214近场可信像素→3211 CAD/自体筛后点→1852高度内点→1125纹理双眼点；48/53来源为空，1115点集中24s，0高度上限拒绝，727纹理拒绝。不能认定全部是光照/上半遮挡，也不能删条件救分。当前E0/E2 uncapped main和final都.5，仅提高外层cap行为等价，未执行。来源精度不再是传统基线任务的前置门槛，但没有观测仍需诚实披露。

直接下一项有界对照：复用冻结SGBMStereo、原rig/CAD/高度/纹理/近场条件，固定6个E2旧曝光[0,60,130,210,240,255]，只换匹配器、不拟合网络/扫阈值。这是开发回放、不是fresh/online；若有不同实际障碍风险输入且无坐标/时钟错误，再封存一次同35s真正闭环源方法对照；无有效差异就拒绝不重复。整体INCOMPLETE，ACTIVE，没有外部阻塞。


2026-10-09T04:29:53.565532+00:00 同轮继续：固定六曝光SGBM probe session20172 actualCLOSED0（不启动物理），3epochs有不同非空riskcells，未读GTdepth/不拟合/不调参数，development非fresh/online。21CPU tests PASS1.40，独立prelaunch C0I0，Minor docstring已启动前修正stageSHA。ONE classical matcher composition35s E2 actualstage44345 STARTED/status sgbm_baseline35_e2_status_v1.json，唯一Fast→SGBM，rig/light/CAD/height/texture/map/planner/PID/速度/6s交接不变，knownpose任务先验不是严格视觉准入；host无遗留匹配进程。继续44345，不重复启动；原始目标/接触为导航验收，不仅延后contact或多source点就通过。无Git，WholeINCOMPLETE/ACTIVE/noexternalblock。


## 2026-10-09T04:37:50.256639+00:00 传统SGBM实跑闭合与首处分叉核验

SGBM E2 stage44345 normalCLOSED0，350req351cap1751physics/2101实际contact检查全部可用0，双视频351@10实际解码。35秒TIMEOUT，goal8.118060229m、净轴.785603952m、path3.853348033m；matched25.92轴比旧Fast少2.356093764m。无碰撞只是退回/未到达，明确拒绝导航改善。71来源35空 vs旧53来源48空仅不同轨迹采样缺口，不能称精度或共同时间覆盖改善。当前方法固定可评测，不以高精度gate作为baseline前置，但可靠导航仍失败。独立结果C0I0。

继续唯一first-fork60封存CPU47196 CLOSED0：两臂14非raw物理前缀精确；SGBM在.5s产生34点/4风险格，6s当前点0但保留历史，localtarget从(-2.05,-.05)改(-2.25,.65)，source→map→reference→request真实生效。纯冻结planner实际reference逐值复现，只有4风险格改回unknown的反事实恢复直线目标；未改actor/未把unknown写free。先封存source/plan再只读旧renderer标签：34点全有GT深度15.8–24.05m，却预测约2.9m，世界误差median17.8815/p9021.4944m；左右当前像素无exactblack，不能套黑色删除。这是重复纹理/错误对应的具体证据，不是接口未来泄漏，不拿GT删点或放宽判据。来源准确率仍仅诊断，旧gate不倒改。

下项直接完成这套固定传统SGBM基线的剩余条件覆盖：E0/E1各一次原35s任务先验+knownpose评测，所有模块/阈值固定、不再跑E2，不期待成功、不作为“已通过改进”移植，不启动原严格RGB+estimatedpose准入。它是记录一个诚实可复现baseline的完整结果，而不是换第二匹配器救分。完成后再按同一冻结方法检查estimatedpose输入/评测范围，随后有界学习增量；未满足的层次保留。WholeINCOMPLETE/ACTIVE/noexternalblock/noGit。


2026-10-09T04:42:02.628170+00:00 baseline coverage接续实际STARTED session31615/status sgbm_baseline35_remaining_status_v1.json，固定SGBM E0→E1各35s/350req最多1条，共2；独立prelaunch C0I0，条件metadata max1+stage2已前置修正。host无匹配遗留进程，source/control core与已闭合E2相同，不是改进移植，不重复E2，不触发原strict视觉准入。继续31615/每条件rawseal/log，不重复启动。第一分叉CPU核验只支持4risk cells影响同初态6s参考，不能单独证明整个映射或后半程退回归因；后验34点GT仅诊断。共同计分时域25.919999s来源缺口v2为SGBM19/52空、Fast48/52空，轨迹/RGB不同不是准确率；v1保留26s完整采集范围含postcontact，不能用于score比较。WholeINCOMPLETE/ACTIVE/noexternalblock/noGit。下一评测实际goal/contact/videos/状态，然后进入固定基线estimatedpose层接口核验；不再以来源高精度作为传统baseline准入，不无限扫描。


## 2026-10-09T04:50:33.194678+00:00：固定传统基线覆盖完成，接续估计位姿闭环

SGBM 同一配置的 E0/E1/E2 均正常完成实际仿真及视频导出，35秒实际到达为0/3（有平坦地面/安全起步区及已知位姿的 TaskH；不是原严格RGB导航成功率）。E0 剩余目标距离3.53285米、起终点轴进度5.30922米；E1 剩余5.89099米、轴进度0.71352米；E2 剩余8.11806米、轴进度0.78560米。均没有真实障碍接触，但E1/E2出现回退，因此没有碰撞不等于可靠导航。作者实际查看 E0/E1/E2 轨迹图；评估脚本解码完整左右拼接及第三视角视频，作者没有声称逐帧看完全部视频。原始来源、控制、正式视觉准入拒绝均保留。

已直接启动唯一 E0 位姿来源替换实验（session16721），注册及导航共同消费当前 native IMU/encoder 估计，初始 GT gauge、平坦高度先验和低层PID理想IMU明确保留。SGBM、相机、光照、地图、参考机制、增益及速度不改；最多1条、35秒、无重试。未来/过期/无效时间及未可用估计都拒绝，无真值回退。TDD四项缺失失败→通过，再独立审查发现NaN时钟漏洞，新增测试失败后修复；最终针对性27项通过。全实验目录裸pytest尝试有26项收集错误，主要第三方隔离包缺依赖、历史重复模块及pxr动态库；完整名称和原因保存于 native_pose_baseline_full_cpu_suite_v1.log，不声称全套通过。独立审查最终Critical0/Important0。

下一接续：检查 native_pose_baseline35_e0_status_v1.json/进程，不重复启动；正常关闭后执行 assess_native_pose_baseline35_e0_closed_v1.py --condition e0，核验原始封存、真实到达/接触、轨迹、两视角视频、来源/注册/导航身份和位姿误差。若未到达保留失败；不把小位姿误差作为导航成功，不要求旧95%来源准确率准入。本轮仍未完成学习记忆增量及原严格RGB/原35秒正式整体交付。


## 2026-10-09T04:56:14.904970+00:00：估计位姿E0实际关闭；E1/E2已直接接续

单条 native IMU/encoder E0 正常关闭：350请求/351采集/1751物理，2101 contact全部可用且0接触，两视频各351帧10fps实际解码。实际35秒超时，目标剩余3.576907592米，起终点轴进度5.269047629米，比同SGBM已知位姿版本少0.040172724米；终点XY估计误差0.023695395米，p90 0.017780156米。作者查看两臂轨迹图；整体几乎重合，不声称逐帧观看全部视频。这支持该E0中的位姿漂移不是主要进度瓶颈，不能泛化所有场景/纯RGB或认证安全。所有source注册epoch为estimated，所有navigation行native_pose_used=true，无GT fallback；初始gauge/高度先验/低层IMU特权保留。实际循环wall中位.3950秒/p90 1.6237秒，不声称实时10Hz。

独立审查准备评估器发现提前结束时误用基线35秒终点比较的Important。原v1按字节保留；TDD新增提前结束测试RED→GREEN，独立v2同时取双方cut前真实状态并记录采样时刻，review C0I0。旧remaining E0/E1双方均35秒结果不受影响，不追改；作者v2实际执行退出0，所有原始seal核验。

不因E0报告完成就停下：同一配置的 estimated-pose E1→E2覆盖已实际启动 session47749，stage native_pose_baseline35_remaining_status_v1.json，driver run_native_pose_baseline35_remaining_v1.py。仅episode/protocol/output变化，最多2条各35秒，timeout1200后45秒清理，运行失败即停，无重试。独立预审两协议2102源码/90输入哈希匹配、C0I0，Minor旧E0验收身份字段已启动前修正。基线覆盖不要求导航候选成功迁移，不把E0超时说成通过。相机、光照、SGBM、控制增益、方向交接、速度与risk语义原样。

精确接续入口：检查session47749和native_pose_baseline35_remaining_status_v1.json。E1正常关闭可执行 assess_native_pose_baseline35_remaining_closed_v1.py --condition e1；E2关闭同脚本 --condition e2。不得重复启动/重跑E0或knownpose三条件。封存结果、真实goal/contact、源/位姿使用、轨迹及视频后汇总两层传统baseline；候选超时明确拒绝。之后核验当前任务的因果action-conditioned学习增量前提及既有单次学习拒绝身份，不能偷偷重训第二网络救分，不能把taskprior结果冒称原strictvisual成绩。研究整体仍未完成，没有外部阻塞；实际仿真进程继续运行。


## 2026-10-09T05:13:28.358788+00:00：传统双目基线两层完整实际覆盖与首次分叉因果证据

同一固定 SGBM＋显式风险记忆＋局部规划＋roll-PID 两层均实际完成：

|条件|已知位姿35秒结果/目标剩余|估计位姿结果/目标剩余|估计位姿真实轴进度|
|---|---|---|---|
|E0|timeout / 3.53285m|timeout / 3.57691m|5.26905m|
|E1|timeout / 5.89099m|timeout / 5.85050m|0.75360m|
|E2|timeout / 8.11806m|collision 19.64s / 6.47545m|2.42632m|

两层各0/3到达。它们严格保持35秒时间规则，但有独立TaskH先验，不冒称原严格cold-start导航成功率，也不称纯RGB。E0/E1位姿来源替换后进度差约±4cm，E2不同初始图像且真实碰撞，不能把小位置误差理解成全域导航合格。估计E1最终误差.00301972m/p90.00477791m；E2最终误差.00491894m来自19.7s最后capture，仅offline诊断，contact19.639999561s导航评分严格取之前19.6s真实位置，没有用19.7s未来位置救成绩。独立review C0I0，E1原始2212/E2 1230封存全验、视频351/198帧各10fps；作者查看两条轨迹图，没有声称逐帧观看全部视频。

E2 firstrequest60之前14非raw物理字段精确相同，但初始RGB像素通道差左186391/右181511；最早.5s已知位姿运行有34风险点，估计位姿运行0点。唯一四格历史分解固定缓存RGB/预测分别与known/nativepose交叉，不新推理、不新物理、不读GTdepth、不动actor：A图像两种pose均34点/相同两坐标格，B两种pose均0。两原始对角逐值复现，支持最早差异跟随RGB与缓存匹配输出组合而非pose swap；没有分离RGB与匹配器内部原因，也没有归因全部后续碰撞。原v1四次数值完成后np.int64 JSON失败exit1，输出/日志保留，v2仅解析原打印整数行及验证四NPZ并写汇总，未重算source，独立C0I0。

继而执行唯一34点的历史时间支持诊断，固定6个当前曝光6.5/7/7.5/8/10/13s，复用原LK21窗口/3层/30迭代/epsilon.01/往返1px/灰差20，无阈值扫描；出生投影回查<1e-4像素。6.5s有33可用轨迹，近点预测重投影残差median2.4545px/p903.6631px，全部超1px；7/7.5/13s缺可用轨迹只记unknown；8s/10s仍有效2/10条但残差约50/29px。它只显示保留假设缺乏相同几何的时域支持；LK不是正确对应证书，不删actor风险、不称完整安全或准入。无新网络拟合。

已向学习阶段推进真实数据准备：4条历史完整35秒序列各349个0.1秒视频对/540查询网格，当前native IMU/encoder/实测关节/已知当次请求/上次请求特征先seal，再计算未来视频LK标签。可用LEFT-flow标签数72123/29489/31569/70820，均历史设计数据、不是fresh；压缩视频和近似光流不是3D/占据/自由真值。旧MODEL可靠性分类器拒绝保持，明确不具action-conditioned未来预测语义，不能冒称世界模型。当前没有训练或接受新模型；需双眼数据、一个完整冻结预测任务与新整序列封存验证，禁止相邻帧随机切分或未来applied/反馈输入。


2026-10-09T05:15:13.256276+00:00：双层基线完整关闭后继续了实际历史学习数据准备，右目补充session64478已启动。最新接续 baseline35_learning_data_continuation_started_v1.json；先检查输出/日志与进程，不重复计算左目或已关闭两层导航矩阵。当前只做历史fit-data可行性，尚未训练网络、没有fresh heldout/视觉encoder准入，不能把LK视频标签称3D安全/占据真值。新动作条件候选必须另冻结完整任务/比较/整序列验证，并沿用源身份、先验与未知标签。整个研究未完成，自动续做保持ACTIVE。


## 2026-10-09T05:27:38.375201+00:00：双目历史数据关闭；唯一短时视觉预测任务与fresh采集已启动

右目补充CPU实际exit0，四条历史序列可用近似LK标签71330/29976/39311/70993；原输入先seal，原始视频/source lineage在独立assessment_v2与artifact_manifest_post_lineage_v2补核验，旧v1字节保留，没有重算标签。左右均历史fit数据，不是fresh，不是3D/自由/占据真值。

唯一新候选已预先冻结：action_conditioned_dual_video_forecast_protocol_v1.json。23维当前/过去视觉运动、IMU/encoder/实测关节、当前已知请求与上次请求；0.1秒双目下一帧运动，单次train-only scaled ridge residual alpha1。四条历史整序列拟合，两个新横向偏移起点完整序列验证；每眼每序列移动可用查询≥100、移动帧≥10、meanEPE相对持续旧光流≤.85、p90≤5px。唯一候选失败即拒绝，不换网络/阈值/重拟合。它是低阶视觉运动预测增量，不冒称完整导航世界模型、M3、视觉encoder或深度/安全认证。

独立采集预审发现继承manifest旧hash/距离/认证Important；实际启动前已修正：Euclidean E0 8.89620705694m/E1 6.60170432540m，正式parser canonical hash重算，旧父条件clearance/oracle/humanreview单独历史归档；新start human_review needs_adjustment明确未经新人工认证，授权实验采集不冒称正式场景准入。原坏的pre-review字节保留。独立recheck Critical0 Important0。相机、光照、控制器/增益/协议/smoother/速度不变，只新完整序列横向起点±.15m。

已实际启动stage run_action_video_fresh_acquisitions_stage_v1.py，session11402，status action_video_fresh_status_v1.json。最多2条各原35秒，单条运行失败即停，timeout1200s后45s清理，不重试，不序列内reset。完整raw封存前禁止拟合；仍未读取fresh未来标签作设计。预测实现与四项CPU测试已执行：先缺模块RED，再4GREEN，含未来图像/请求替换和日志截断后前缀因果输入逐值不变。独立模型实施预审正在进行，尚未拟合、没有预测通过结论。

下一实际入口：检查session11402/status，两fresh完整封存且实施预审关闭后仅一次执行 run_action_video_forecast_fit_validation_v1.py；先四历史训练，再两个fresh全部预测封存，之后读取未来近似LKteacher判分。离线当前/过去reader不冒称在线接口；通过才最多2条只读在线shadow，失败明确拒绝，不以模型结果替代导航评测。两层传统baseline各0/3、估计E2真实碰撞结论保持。整体研究未完成，没有用户外部阻塞，自动续做ACTIVE。


## 2026-10-09T05:49:40.393053+00:00：唯一动作条件双目短时预测器已拒绝；真实控制消费试验直接接续

两个fresh偏移起点整序列均实际35秒正常关闭，分别350请求/351capture/1751physics/2101可用contact且0接触，双目/第三视角视频各实际解码351帧10fps。E0仍timeout，目标余3.641251515m、真实轴进度5.25874999981m；E1仍timeout，目标余6.033791707m、轴进度.570451804m。它们是预先冻结的预测验证采集任务，不冒称新正式E0/E1成绩，不把stage exit0称导航通过。

四条历史整序列仅拟合一次23维动作条件视觉运动ridge残差模型；两个fresh先完整rawseal，模型固定，两个整序列预测全部seal之后才算未来LKteacher。当前/过去图像flow、当前IMU/encoder/实测关节与已知请求，无未来feedback/applied/GTpose/GTdepth。teacher是压缩视频近似2D对应，只对inside且固定CAD排除的可用查询评分，缺失unknown，不是3D/自由/占据真值。

|fresh|眼|移动meanEPE px|持续旧光流baseline px|ratio|p90 px|判定|
|---|---|---:|---:|---:|---:|---|
|E0|左|1.996958|2.458731|.812190|3.831605|PASS|
|E0|右|1.968521|2.273417|.865886|3.818085|FAIL|
|E1|左|2.787094|2.897738|.961817|5.021830|FAIL|
|E1|右|3.936395|4.019301|.979373|7.641839|FAIL|

四组最低100移动查询/10移动帧覆盖均满足，但冻结每眼每序列ratio≤.85、p90≤5必须同时满足，因此唯一候选总体REJECT。E0局部收益不能泛化E1；没有refit/换网络/放宽门槛，不启动该模型在线shadow或动作消费。它仅是低阶下一帧视觉运动增量，不是完整导航世界模型/深度网络/新导航成功。独立review C0I0，所有原封存、特征shape(349,2,540,23)、模型身份、按原540批次重算预测逐值一致、四组指标独立复算；四CPU单测含未来图像/请求替换及日志截断前缀不变。作者实际查看误差图，视频均解码但未声称逐帧观看。产物 action_video_forecast_fit_validation_v1/assessment.json、closed_forecast_summary_v1.json、fresh_forecast_errors_v1.png、model.npz及全部inputs/predictions/teacher/seals保留。

继续做了有界历史因果诊断：risk age原v1因错找metrics.json在任何map分析前exit1，原失败文件保留；独立v2改用真实native_pose_assessment_v2 scored_end，E1 209/350地图含>现有10s raw TTL旧正格、peak36，E2 0/197。当前实际类RepeatedExposureMemoryGrid本就允许两个capture确认后静态持久化，此结果不证明TTLbug、错误占据、失败因果或自由空间，不能据此直接删格，更不能统一解释E2碰撞。独立review C0I0。

转向具体当前控制消费证据：first_tail_handoff_audit_v1实际核验E1第一参考，当前native position/旧参考/同帧地图复算guard逐值一致，不二次act、不用未来。15.5s endpoint距离.4696694029m、剩余arc.4696378868m，唯一reject reason为remaining_arc_under_lookahead，之前activeforward/no pending/tracking，随后rebuild第二参考并制动/倒车。这只证明重建条件，不预判bug。

唯一传统控制候选 endpoint_tail_retention_v1已预先冻结并实际启动session38736，stage endpoint_tail_e1_status_v1.json。仅第一版all-forward局部参考、当前forward/no pending/tracking、唯一短尾拒绝且endpoint>.2m时retain；.2复用原goal半径，hazard/projection拒绝不旁路，后续参考旧逻辑，原planner生成器、memory、mapping、PID、速度、动作范围、换向五采样与反弹清零、相机光照都不改。旧第一方向planner callback globals保持，源代码只替换原boundary一次guard调用，不二次act。CPU先RED后4GREEN含组合边界，独立用原frame155调用retain且reference version/state不变。独立审查metadata Important启动前修正并保留原preparedprotocol，最终C0I0。

单条实际E1/原35秒/350请求，接触或真实goal终止，timeout1200后45秒清理，不重试。比较同initial条件的native E1真实历史B0，不能假定render RGB完全相同或声称完整因果A/B，关闭后核验prefix物理/RGB；旧同流只读shadow未来不冒充替代轨迹。验收采用启动前冻结B0第一参考几何，真实endpoint≤.2、final轴进度至少增.5m、firstref横向RMS/peak不退化、0接触和有效请求改变；实际goal/35秒导航层另报。整体未完成，没有外部阻塞，不停止自动续做。精确入口：检查session38736/status，不重复两fresh采集/单次fit/历史矩阵；关闭后独立评估、记录接受/拒绝，不能调条件救分。


## 2026-10-09T06:07:23.321150+00:00：尾端消费候选实际拒绝；局部制动与续接几何已核验

单条 endpoint_tail_e1_run_v1 实际正常关闭（stage session38736 exit0）：原35秒350请求/351采集/1751物理/2101可用contact且0接触，双视频各实际解码351帧10fps。导航仍timeout，目标余6.259873528m，真实起终点轴进度.351923269m，相对实际历史native E1 B0减少.401677132m。第一参考106实际位置样本：横向RMS .049298415m/peak .090236544m，投影最大真实arc1.801338829m，最近参考端点.297272459m，未达到冻结.2m；B0为96样本、RMS .051091008m/peak .090260627m、arc1.629941808m、端点.467058191m。接触、横向误差、实际介入和请求改变检查通过，真实端点/最终进度增益两项失败，唯一候选REJECT，不重试、不放宽资格/.2/停止界。

介入15.5/16.0s确实改变参考消费。但实际B0和trial第一请求差frame89早于介入155，初始左右RGB不同403908/404134通道值；分叉前14非raw物理字段相同。因此历史对照是必要实测比较，不冒称同RGB完整因果A/B，也不能把整条最终退步唯一归因于尾端改动。评估器预关闭审查发现首参考最后执行区间漏终点capture，测试RED→GREEN后修正为每个实际version1区间的i/i+1并受评分cut约束；未改验收门槛。独立关闭review C0I0，原2212与评估后2215封存逐项核验。作者查看实际轨迹图，未声称逐帧观看全部视频。

继续实际执行只读停止链算术核验 audit_endpoint_tail_stop_rule_v1.py，exit0，不二次act、不重放policy、不读未来/GT反馈、不做新物理：当前native反馈在16.3s速度.224516791m/s，剩余waypoint长度.333547261m大于原V1停止距离+.1=.327201257m，仍tracking；16.4s速度.244040850m/s，剩余.316957847m小于停止界.361253573m，实际goal_braking/请求零，此时全局目标还距4.933576379m。原局部参考制动规则数值解释停止，并非已经到达全局目标；不证明停止距离物理保证或认定实现bug。165已安装参考版本2，明确不拿它反推版本1制动。独立review C0I0。

16.5s重建的hazard/raster检查实际均通过，唯一短尾拒绝仍在；尾端候选因tracker已goal_braking而不符合预先冻结资格，不是障碍屏幕触发。原风险age审计不支持删旧点/修改TTL来救这项结果。

继而执行唯一4次同地图名义几何查询 probe_tail_continuation_geometry_v1.py，exit0：当前native反馈/map在155/165两时刻，各比较原不限制首方向与既有minimum-two-forward几何诊断，map/target/.8707106781m margin/四点模型不变。155原几何reverse起步成功338nodes；限制两forward成功740nodes但之后仍reverse，且155实际retain，诊断新计划不是当时实际执行。165原计划reverse起步成功340nodes，与实际计划全部waypoints逐值复现；限制两forward不可达4nodes。此结果仅名义可行性，不准入强制前行/新动作候选，不放宽empirical域或margin。独立review C0I0。

具体尚未解决：局部有限参考的安全终点制动与下一段可执行续接相互制约；较早可行的两forward也仍需要后续reverse，不能通过延迟重规划等同解决。传统双目已知/估计位姿各0/3到达，估计E2真实碰撞，唯一低阶双目短时预测器拒绝保持；整体研究未完成，没有外部用户阻塞，不声称新的世界模型或导航成功。

接续应以已封存的同版本在线日志构建“下一段续接可行性事件”的只读接口：明确何时当前参考仍可跟踪、同方向续接何时丢失、制动请求与下一参考方向何时分叉，输出当时可用输入，禁止读取未来标签或实际反馈动作制定未来计划。只预先定义一个事件/一个候选，若没有可执行物理证据则先拒绝，不能继续尾端阈值/Oracle/网络矩阵。当前所有仿真/fit进程已关闭，不重启已完成矩阵、fresh采集或唯一fit；下一实际入口是读取 tail_continuation_geometry_probe_v1/assessment.json 和 endpoint_tail_stop_rule_audit_v1/assessment.json，建立事件接口及当前输入隔离测试，冻结单项闭环实验之前不改actor。


## 2026-10-09T06:16:01.509500+00:00：只读局部参考停止事件接口与真实在线单条运行已直接接续

未停在尾端拒绝报告。local_endpoint_event_v1离线接口及3项CPU测试完成；独立审查发现时间校验晚于payload读取Important，用trap测试先RED后独立local_endpoint_event_v2先校验时钟，旧v1及旧v2因果证据按字节保留；新增独立live adapter，合计5项CPU测试PASS。verify_local_endpoint_event_causal_v3实际exit0：155–164未来native feedback/当前之后policy rows替换或截断输出逐值不变，原request bytes unchanged。读取的是current native＋上次完成tracker cursor/reference，不重做projection；零请求仅当时构造的停止假设，不声称当次真实after-act请求预测，离线证据不称online接口通过。旧源码/测试原字节身份恢复核验，新版本另存，无倒改原结果。

经独立prelaunch Critical0 Important0，2105源码/92输入哈希全验，Minor旧stage2/旧时间身份启动前修为当前单条，已实际启动 run_local_endpoint_live_e1_stage_v1.py，session66787，status local_endpoint_live_e1_status_v1.json。每实际循环在唯一policy.act之前读取current snapshot并立刻写local_endpoint_live_before_act.ndjson；结果不参与request选择，每循环先清空避免陈旧字段。原native-SGBM B0感知、memory、planner、rollPID/gains、速度/动作、换向规则、rig/light、TaskH先验不变，不使用已拒绝tail候选或短时视觉模型。

仅1条原35秒/350请求；真实contact/goal/35秒原终止规则，wall timeout1200+45清理、无重试。此项只验证真正在线的事件接口；轻量shadow on/off测试只证明调用/不可变性，不冒充同RGB同physics全程A/B。实际导航结果另报，online接口通过不代表模型/控制/正式导航通过。当前真实运行进行中，整个研究未完成。接续：先检查session66787/status，若仍运行不重复启动；正常关闭后执行 assess_local_endpoint_live_closed_v1.py、实际验证封存/clock/raw/视频，记录接口与导航分别结果，然后按证据冻结单项参考续接研究，不回到网络或尾端阈值矩阵。


## 2026-10-09T06:58:46.891299+00:00：在线事件已关闭；当前地图换侧证据与唯一短倒车消费者准备

只读 local_endpoint_live_e1_run_v1 实际退出0，350请求/351capture/1751physics/2101可用contact且0接触；视频实际解码351帧10fps。仍35秒timeout，目标余5.850665987m、真实轴进度.753398460m。289个有效事件全部是tracking-reserve，时钟/调用前日志/几何重建接口通过，但没有真实stop事件，停止分支没有在线准入；不强迫事件、不再补跑。评估器v1误用native64速度而非实际PolicyObservation32失败记录保留，独立v2实测float32语义重建逐值通过，v3只补范围，不改旧字节。独立review C0I0，不能把接口通过写成模型/控制/导航通过。

继续关闭了历史因果核验而非重跑矩阵：三条E1前两参考均下侧→上侧，新增参考首段reverse；四格position×map查询显示固定position也随map换侧。当前map排除了保存的那一条下侧路线（19/51点不足原.87071m裕量），不证明所有下侧路线不可行。30新增风险格追到1011原始来源点，852有有效旧渲染深度，159未知；有效子集深度误差median.007903m/p90.030774m。这是输出后的offline诊断，不能称完整3D安全/源模型准入，也不支持把换路简单归因于双目乱报。初版JSON np.int64失败保留，独立v2实际重读同一旧数据，非新采集/拟合。审查均C0I0。

一条可选同侧几何路线名义倒车从5边减少到1边，但未达到原“首两边forward”条件，optional_same_side_route_geometry_v1仍REJECT，没有执行该候选。现独立冻结不同目标的唯一short_initial_backing消费者：原路线和一个当前风险图＋已知过去侧向hint的可选路线，只在初始名义reverse边数严格减少时选择；查询最多一次、原路线fallback、原方向交接/普通安装/PID/速度/源/相机不变。不是强制保持旧侧、不强制执行反转、不改cursor，不把名义边数减少当实测通过。

纯选择与实际组合边界CPU三测试PASS；组合委托原安装但mock安装不能认证真实换向。实现与设计独立C0I0；driver/protocol审查发现旧身份字段I1，启动前保留准备字节并修正，待最终recheck后最多一条E1原35秒实际运行。预冻结局部验收：可用contact0、真实轴进度≥1.5m且较历史B0增≥.5m、实际执行选中参考、实际倒车且向首cusp位移≥.15m；正式到达仍≤.2m/35秒独立判断。若没有实际E1到达不触发E0/E2，不扫阈值/重试/拟合新模型。研究整体未完成。


## 2026-10-09T08:14:46.356276+00:00：唯一短倒车运行中断；不记导航结果，运行监督与实测时间预算接续

short_initial_backing_e1仅启动一次，session15220现已不存在，pgrep实际核验无相关运行。failure.txt显示source capture内SIGTERM→SystemExit143，关闭又发生XIO/pulse abort；不能由此证明信号来源或控制bug。原status running字节保留，是陈旧记录，不再当作实际计算。实际仅6.5秒：65执行请求区间、64完整policy rows、pending64保留、66capture、326physics、391可用contact全部0，两视频实际解码66帧10fps。0候选事件，65请求和14非raw物理字段与真实历史B0相同。候选没有介入，local gate/null、formal导航/null，不叫35秒timeout、性能REJECT或成功，不以部分goal余6.55728m/轴.04273m冒充正式分数。独立中断审查C0I0，385封存/2110source/93input哈希全验；脚本进程不存在字段由作者外部pgrep验证，审查另独立核实，并非脚本自行证明。单条无重试预算保持，未重新启动同实验。

随后继续实际CPU工作：新增三个完整组合回退测试，missing/same/longer均保留原路线、原状态、一次查询上限；总6PASS，原冻结测试源码不改。运行监督独立helper先missingmodule RED后3GREEN；独立review复现普通启动异常仍starting的I1，新增测试RED→GREEN修复为runtime_failed/失败类型，最终状态写入失败也恢复signal handlers；当前4PASS，待复核。新helper只给未来独立阶段使用，没有修改本次已冻结driver/补跑候选；SIGKILL/断电无法捕获，仍需PID外部核验，不保证后台永远计算。

对同一旧nativeE1第二参考做两个有界只读实测审计，无新物理/二次act：名义首五reverse约1.5m，实际最大投影1.42323m、最近首cusp.07701m；真实首次进入原.2m cusp邻域30.7秒。方向锁存17.4秒进入reverse、joint1实际速度低于-.1为17.64秒；32.4秒锁存forward，原35秒只剩2.6秒。此后真实轴进度.39478m、实际路径.39739m，目标6.24548→5.85050m。说明B0确实走过倒车段，长倒车消耗预算而后续仍未到达；不声称锁存方向等于机体运动、不断言短倒车候选已改善或首次交接是全部失败原因。历史单段审计审查待完成。

研究整体未完成，最新接续文件short_initial_backing_interrupted_continuation_v1.json。当前没有运行中的仿真，不把旧running或自动触发当持续计算；未重训任何模型、未调PID/速度/相机/光照/阈值，不启动E0/E2。先关闭监督器与两单段审计复核，保留候选未评估层，再据真实时间预算设计下一项独立有界验证，不能偷偷重复本单条或放宽35秒。


## 2026-10-09T08:21:55.888350+00:00：运行监督和预算审计复核关闭，候选仍未完成物理评估

独立审查监督器修复及两项B0 cusp审计C0I0；4监督测试PASS，6消费者选择/组合测试PASS。重要范围保留：完整ref2窗口的最大投影/RMS不是纯倒车运动段；forward方向锁存不等于实际速度转正。无新物理、无重试中断候选。

继而实际完成已有三条native基线的总行程/请求时间预算核验，没有重跑三场矩阵。E0采样位置弦长5.9093m、真实轴5.2690m、效率.8917；至少直线8.6391m才到原.2m goal邻域。E1弦长4.1539m、轴.7536m、效率.1814、必要直线6.4m。E2按19.64s接触截止，仅用19.6s最后真实位置，弦长2.6176m，不外推碰撞后到35秒。这些是既有实测不足，不是最大可达能力上界，也不证明任务不可能。

旧预算v1错误将假设前6秒无位移的数值称实际必要速度，review I1后保留旧字节，独立v2改用6秒实际位置至goal距离；剩余29秒必要理想直线平均速度E0/E1/E2为.296754/.219543/.298853m/s，旧数值明确反事实。路径仅采样捕获位置弦长，非连续物理弧长。两预算审计输入与算术独立C0I0。

按已发出的main请求符号划分真实区间（不是按机体实际方向）：E0正请求29秒贡献轴+5.23579m，零请求6秒仍+0.03325m；E1正12.1秒+1.92401m、负13.8秒−1.31969m、零9.1秒+0.14928m。反向请求下的位移抵消与时间消耗是具体瓶颈之一，不能归因唯一失败原因。图已实际生成并由作者查看，资料目录新增“导航进度与时间预算.png”。

额外有界指令身份核验E0：290实际planner行、1450主轴分量，uncapped与最终issued都恰为+.5。对这些当前输入，任何只把末端正向ceiling提高到≥.5的方案在min(.5,ceiling)后仍是+.5，行为等价；不能称它为速度改善，也没有为它启动无效仿真。其他场景不自动泛化。

下一消费接口须明确“名义参考main幅值的生产”与“末端cap”区别，并同时保留名义四点运动域不支持任意提速/瞬态的拒绝。精确入口：run_short_initial_backing_e1_v1.py 的实际MRO；command_domain_cap.py；benchmark/policies.py OraclePathPolicy._planned_main_command及实际main合成；local_plans/frame_060.json与policy_records.json；据当前同版本测量冻结一个独立、实际能改变请求的验证，不修改旧短倒车单条预算，不同时改参考、mapping、gain和速度。自动研究整体未完成；不存在实际正在运行的进程，不能承诺始终计算，后续从最新入口继续而非重做这些审计。
