# 稀疏双目覆盖与终点可接近性：实际结果

本报告分开感知、空间记忆和控制规划的结论。整体导航研究尚未完成，原始35秒E0/E1/E2未触发，学习记忆/世界模型尚未准入。相机安装、光照、PID、旧数据和验收标准保持不变。

## 双目与记忆

唯一局部双目修正使用当前RGB、当前CAD遮挡和估计位姿；7×7块、±4视差窗口、0.5步长、原冻结相似度条件。倒车视角保留451/4918条点记录，10cm内准确率99.33%、误差p90为7.15cm；旧正面起步视角保留240/36712条，准确率100%、p90为1.54cm。两者均没有两次单帧达到100点，原覆盖门槛失败。旧正面数据只属于历史适用性回归，不能当作fresh或只改变yaw的A/B。

跨帧累积后，倒车451条记录仅对应3个XY网格，正面240条仅对应10个。使用既有20cm MODEL支撑盘做独立解析诊断，分别覆盖真实障碍185个平面网格的38个(20.54%)、71个(38.38%)。该解析盘不是正式runtime映射器，不证明隐藏体积安全，不能写成观测自由空间，也没有调整半径来提高覆盖。记忆保存看见过的区域，不能把未看见的区域变成已知。

原“±1像素”宽度是深度敏感度，不是经过标定的概率置信度。它仅覆盖倒车18.40%、正面52.08%的实际点误差。因此不会把它作为安全保证，也不会把旧pose_sigma=0当作零位姿误差。所有选择/点输出封存后才读取真值标签。

## 唯一终点规划候选

对历史native R1第一处最终目标重规划事件(frame480)做CPU检验。父版本只要求名义终点落入20cm位置半径；候选额外检查当前名义终点朝向：目标在前方，且几何接近目标所需曲率在冻结必要范围[-0.827225,+0.796603]内。这只是必要几何条件，不是动态/停车预测，也不是全域模型。只支持已有前向方向，未标定的倒车转弯没有被引入。

原父版本参考逐点重建完全一致；独立搜索函数去除新增参数和接收谓词后AST完全一致。保留20cm位置容差、3000节点、原四种动作、30cm弧段、成本、安全距离、地图、跟踪器、PID与速度，不修改旧源文件。

候选在3000节点内未找到合格终点，明确拒绝进入GPU。56个被扩展到目标邻域的节点中，27个前向、14个同时面向目标，仍没有一个符合曲率条件；最近节点距目标仅1.21cm。这说明“位置很近”并不充分说明当前朝向能继续靠近目标，但不能证明真实机器人或全局路线必然不可达。没有提高预算、缩小网格、放宽条件或试第二个候选。

## 已完成与未完成

本轮实际完成一个冻结双目候选的两域离线评估、来源空间覆盖/宽度核验、一个终点接近条件的CPU搜索实验。此前实际14秒倒车观测、60秒native导航与provided-MODEL诊断的失败保持原结论。新候选没有在线运行，不报告新成功率。

当前有两项独立障碍：新视角的准确点覆盖不足；已有名义动作和位置终点条件不足以保证可跟踪的终点接近。下一项控制工作应首先检查搜索的离散状态合并是否丢弃目标邻域中本来满足接近条件的到达状态，使用相同已封存请求与动作，不扩大标定或扫描参数。只有这个因果问题有证据，才决定是否存在一个可隔离实现修复；不能因本轮拒绝直接换模型或放宽门槛。感知一侧应保留当前可见表面与未知边界，尚不能接入学习增量或宣称盲区闭环已解决。

## 证据入口

- experiments/stereo_memory_navigation_20261004_v1/frontal_local_binocular_refinement_preflight_v1/assessment.json
- experiments/stereo_memory_navigation_20261004_v1/sparse_support_footprint_width_v1/assessment.json
- experiments/stereo_memory_navigation_20261004_v1/terminal_node_admission_preflight_v1/{inputseal.json,plans_pretruth.npz,outputseal_pretruth.json,assessment.json,bounded_search_attrition_v1.json}
- terminal_node_admission_GREEN_v2.log：6个针对性CPU测试通过；初始模块缺失RED与脚本生成失败日志保留，均不属于科学实验重跑。

作者亲自运行了CPU实验、核对封存输出与源码差异；本报告不声称亲自完整观看视频或完成在线控制验证。独立首轮审查与额外回归测试见后续追加记录。


### 2026-10-07T23:29:54.222368+00:00 — Terminal candidate and causal contracts CLOSED
Terminal sole candidate CLOSED REJECT_BEFORE_GPU: original nativeR1 frame480 parent reference exact; added only optional terminal acceptance, parent AST exact afterremoving2hook edits. 3000budget exhausted,56expandedgoal tests,closest1.209cm/noaccepted. First scoped review C0I0M1, reviewer personally6tests+4primitive/3direction equivalence/signbounds/seals; author17focusedpass3warnings4.37, Minor rotated/asymmetric/zero-reverse cases persisted7PASS1.89; no rereview. Firstgeneration-script StopIteration and repeated-keyword SyntaxError happened before scientificstage, preserved logs.
Passive state-merger V2 CLOSED EXACT:164 generatedgoal-neighborhood endpoints,85cost-pruned,zeroeligible,so no eligible endpoint lost by merging in this neighborhood; upstream pruning/global reachability unresolved. V1 administrative diagnostic acceptancecallback logged extra calls and failedexacttrace beforeseal; invalid scope supplement withdraws premature author exactclaim, oldbytes retained; V2 purepredicate usesknowncoarsegoal,no search/action change,56originalcalls/result exact.
Passive same-search interior audit CLOSED EXACT:1402collisioncheckednear-goal samples,7eligibleinteriors/0eligibleendpoints (indices1..4). Supports fixedarc endpoint limitation hypothesis only, notphysical/dynamicproof. First eligible interior contract presealed and reconstructed by original ancestry,neverbest-of7:partial.1m/path2.2m/end.194151m/nominalclear1.487587/coveredcommands but2switches vsparent0=>REJECT_FIRST_INTERIOR_REFERENCE_NO_EXTRA_SWITCH_GATE. No secondreference/control/model/threshold/budget/rig/GPU/35 trial. Current/source/oldnav/learner rejects preserved.
Scientific report reports/benchmark_sparse_stereo_support_and_terminal_approach_20261008.md andactualdiagnosticfigure saved; author viewedfigure, no newvideo/onlineclaim. No active process; wholemission/learning/formal35 incomplete. Next work must leave this closed terminal/source-candidate branch, first reconcile latest overalltask plan with the now-explicit coverage and mixed-direction constraints, then freeze the next independent perception-memory integration question; do not relaunch closedsearch/7reference variants orrepeatmatcher/motion/calibration matrix. This turn has consumed its execution budget; exact closedstage entrypoints/results preserved for next heartbeat, not an externalblock or wholecompletion.
