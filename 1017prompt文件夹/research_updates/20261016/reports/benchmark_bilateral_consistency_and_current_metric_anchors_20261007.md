# 双眼一致性与当前历史锚点：有限诊断结果

同曝光双眼一致性在本次三个时刻没有可用点对，不能据此为被拒绝的 Hero 深度提分。另一方面，右眼仍能看到并核验旧的度量几何：235/229/190 个历史点，正确234/229/190，左眼没有。这说明一只眼丢失观察时，历史几何不一定消失；但旧点只有4–5个小空间格，不能补出邻近未观测区域。

本轮复用已封存的六个开发包，没有新模型推理、训练、真实动作、视频或导航成功率。已知位姿是特权；不是估计位姿或 fresh held-out。原 Hero 全查询拒绝、原双眼记忆标准、控制和原35秒规则保留。

## 唯一双眼一致性消费者的可行性

冻结当前时刻、原 .10m 世界距离和 stride2 偶数格点。将左眼已保留世界点投影到右眼，只有该格点是右眼原保留查询且回投对应原左眼查询才允许点对；没有从 dense 输出填入被过滤点。时钟核验先于几何 payload。source-only 选择与输入 custody、六包 subset seal 都在新诊断标签评分前完成。

真实结果：235/245/255 三对均0点对，六包为空，大小和新区域覆盖均失败，`REJECT_CONSENSUS_CONSUMER`。空包正确率为不适用，不是100%。四个测试从 scaffold RED4 到 GREEN4，包括20cm共同错误仍可能相互同意的夹具；它说明一致性的逻辑局限，**不等于真实数据已经测到共同错误点对**。

source-only 分层核验：235L有4378点投到另一眼图像内，但没有一个通过另一眼当前 CAD 中心；235R有27个图内、同样全遮挡。245/255左右投影全部在另一眼图像外。原保留 partner 查询均0；没有在 reciprocal/gap 阶段丢掉可用点对。这个结论限于当前预测点及对应眼，不能宣布全部环境永久不可观察。

数据：`bilateral_hero_consistency_v1/{input_custody_v1.json,prediction_seal.json,assessment_v1.json,zero_pair_stage_attribution_v1.json}`；选择和评分是两个实际独立进程，先封存、后读取标签。旧模型输出与旧评分未改。

## 不改变旧标准的单眼历史锚点核验

18s的1501个已核验静态旧点、原当前 per-eye std≥5/NCC≥.8、原10s几何年龄，另要求所有当前7×7 bilinear CAD/FOV tap clear。保留旧世界点、来源ID/时钟和原预期当前轴向深度；只测已保存 Hero depth 在这个投影处的 innovation，没有拟合偏移、生成纠正点或刷新旧几何时间。新 custody/六包测量先封存，之后当前 renderer endpoint gap≤.10m，不足标签留在分母。

| 时刻/眼 | 锚点 | 旧XY bins | 当前正确/总数 | Hero−旧点预期轴向中位数 |
|---|---:|---:|---:|---:|
| 23.5 L | 0 | 0 | 不适用 | 不适用 |
| 23.5 R | 235 | 5 | 234/235 | +.01625m |
| 24.5 L | 0 | 0 | 不适用 | 不适用 |
| 24.5 R | 229 | 5 | 229/229 | +.00241m |
| 25.5 L | 0 | 0 | 不适用 | 不适用 |
| 25.5 R | 190 | 4 | 190/190 | +.01549m |

三组非空 current geometry 精度99.57%/100%/100%，至少20点和2格通过，结论仅为 `LIMITED_CURRENT_ANCHOR_MEASUREMENT_ONLY`。旧世界点与当前 renderer 点的有限误差中位数 .00304/.00286/.00300m。不能为没有锚点的左眼认证，也不能认证新物体或正式来源。

这些锚点处模型 innovation 很小，而本次新区域当前深度偏差中位数约+.086–.104m。这不支持把所有误差解释为单一全局距离偏移；没有实施“用真值平移模型”的补救。当前右眼可见的旧点只覆盖先前同一小局部，不是新几何或足够安全的绕障地图。此前18s路线已经穿过未观测真实障碍，28.5s TTL失效更晚；单眼保留旧点不能被宣传成解决主要导航失败。

数据：`current_anchor_innovation_v1` 的原始 NPZ、input custody、source measurement seal、records、assessment；三时刻/双眼都列入，不挑最好一帧。新诊断均作者亲自执行，没有伪称独立 review。完整项目在双眼 helper 后 **505 passed /25.72s**，8个已有解析射线测试和4新测试联合12通过；对应真实日志保留。

## 已接续的实际观察实验

下一项已经实现独立的一次4秒观察消费者：原观察交接后，两个不同 source 更新的可靠当前点数连续低于100时，最终请求归零，让原 smoother制动、原roll PID的倾侧目标回到0；4秒后必定恢复 B0，不等待深度好转、不重新触发。规划、地图、控制器增益、速度范围、相机、光照和原五次低速确认均不改。它测试实际运动与相机视野恢复，不能只重放旧图像。

五项事件/因果/退出/不修改输入测试实际 RED5→GREEN5，全项目 **510 passed /26.61s**。独立新 runner 保留原 B0一次调用、分开 nominal和final request、只有这个动作消费环节变化；源码差异和协议已保存。计划为 `docs/superpowers/plans/2026-10-07-neutral-bank-visibility-recovery-probe.md`。首次新分支代码审查进行中，尚未启动物理实验；启动后以 `neutral_bank_visibility_status_v1.json` 和实际日志为准。

它只有一次原35秒开发任务，验收同时看真实执行差异、观察期间接触、全部新source包的双眼几何正确率，以及新区域至少20正确点/2bins，不能靠停车宣称导航成功。历史 B0只作描述性参照，严格图像前缀一致性没有保证。失败不扫持续时间、角度、速度或换网络。全文研究仍未完成，正式原35秒 E0/E1/E2未触发。


## 2026-10-07T04:47:37.718632+00:00 first visibility review closed and actual simulation STARTED
FirstNEWbranchreviewC0I0M1/no blocker personally5CPUtests/1287sourceSHAs(original1283exact)/exactrunnerdiff/capture-resetASTunchanged/B0onecall+probeonecall. Full510PASS26.61authorinspectedonly. M1inheritedoldprotocoldescriptivefields closed byindependentexplicitprotocol_scope_supplement_v1 preservingprotocolbytes; nobehavior change/reviewloop. Originalnohup shelllaunchdidNOTsurvivepreconditionhashstage (no status/namespace/hostprocess); preservedadministrative_launch_failure_v1, no model/capture/actualsequence then. Sameunchangeddriverforeground managedexecsession91886 nowACTUALrunning, pid1196935/child1196936, start2026-10-07T04:43:49.247879+00:00. ActualFastwarmup.814s/CAPTURE0/30/60/90/120seen; no scientistfirstfailedsequence/repeat. Deadline900swall/single35s knownpose2.6m/yaw−20, originalrig/light/weights/schedule/gains allfrozen. Notformalnav/newSR/sourceadmission. DO NOTduplicate/restartdriver orpolloldcandidate/Herostatus.
Postclose exactnextentry verify_neutral_bank_visibility_interface_v1.py (causaloriginalhandoff+newconsumer reconstruction fromactualnominal/finalrequest/sourcecounts, prefix/futuretraps noGT), then source/executionreconstruct reusedqualifiedverifiers inindependentnamespace, then assess_neutral_bank_visibility_v1.py ALL eventwindowupdates/min100twice/.95BOTHnativeendpoints/new20points2bins/noeventcontact; contactkeys simulation_time_s. Reportrealbankgravitytrend/executedaction andmetadata/decodedvideos/navoutcome separately; keepfailure/NOTEXERCISED honest. Source-only and GT-novelty supplementnotmaskgate rescue. WholeNOTcomplete/formalE0E1E2NOTtriggered/automationACTIVE. Mustcontinueactualprocess untilclosed, notstopatstagepreparation ornotification.
OldprefixSHA 4c4431625cd1b6b71a5c21475c4e4712d9f5b54e88a5f861213f0720c307fa42; oldprefixbytes 5574


实测观察序列已结束，候选被拒绝。完整核验、轨迹、视频与行政失败见 [单次停车稳倾侧观察报告](benchmark_neutral_bank_visibility_probe_20261007.md)。原准备记录及旧拒绝结论保留。
