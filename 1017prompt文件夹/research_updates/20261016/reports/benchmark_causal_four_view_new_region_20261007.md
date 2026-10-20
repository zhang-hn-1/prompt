# 四视图确认与新区域覆盖诊断（2026-10-07）

本轮单一规则 **REJECT**。确认点的精度通过，但没有覆盖足够新的障碍区域，因此不接入地图、控制或正式导航。此前29.7秒接触失败、严格来源前缀失败、估计位姿及旧特征/模型拒绝结论保持原样。整个研究尚未完成，正式35秒E0/E1/E2未触发。

## 实际执行与信息边界

复用已经结束的 `qualified_hazard_navigation_run_v1` 全部60次曝光，这是已检查的开发序列，不是fresh held-out、新场景或新的在线运行。每次只向过去最多四个、两秒以内的曝光查询；保留每个原始source frame/query/partner/time。候选几何来自冻结Fast/CAD原始提议，不先认为可靠。当前与过去两眼分别需要7像素块/std≥5/时间NCC≥.8，过去和当前左右眼各需要跨眼NCC≥.8，并要求两台相机各自移动≥.05米。没有像素搜索、新网络、阈值扫描、年龄刷新、free标签或动作改动。

所有60次提议和pair证明先封存，之后才读取renderer depth和states评分。Known camera poses属于明确特权，不能称作RGB-only/nativeIMU/估计位姿验证。旧Fast与旧ORB/LightGlue源全域拒绝不变。

## 冻结门槛与结果

每个非空current/source group需要双端几何正确率≥95%，真值不足保留分母；同时在18秒之后实际XY速度≥.05m/s的至少一次更新中，需要至少20个点覆盖原18秒hazard cloud没有的两个canonical10cmXY格。没有空组通过、聚合分数救失败或事后修改口径。

- 14个非空组、3547个点实例，3544正确、3真值不足、0有限错误；每个非空组精度门槛通过。
- 仅21.5秒发现4个点落在两个新格，低于20点门槛。其他非空曝光的新格点为0，覆盖门槛失败。
- 重复source/frame像素在不同current曝光中会重复统计；报告逐次unique source IDs，不把3547点实例当成独立障碍或面积。
- 另记录与原始seed XY点云≥10cm距离作为描述，不新增或替换原冻结门槛。绝对格可能发生边界量化，不能由两个格自动推出有效新区域。

|时刻(s)|确认点实例|新格点|新格数|真实XY速度(m/s)|
|---|---:|---:|---:|---:|
|14.0|162|0|0|0.009|
|14.5|171|0|0|0.017|
|19.5|870|0|0|0.112|
|20.0|1317|0|0|0.111|
|20.5|1022|0|0|0.117|
|21.0|1|0|0|0.112|
|21.5|4|4|2|0.136|

## 失败来自哪一层

在同一已关闭数据上进行了独立只读层次诊断，没有改动提议或原gate：

1. **模型输出缺失先发生**：20.0秒有3054个符合原range/CAD/bodyheight域、能在两眼对应到同一renderer表面的采样像素，但冻结模型的confidence与range联合条件在这些像素上通过数为0。20.5秒2568个对应像素，仅1个通过。该统计尚未加入所有最终伙伴约束，不能当作源准入。
2. **同时双目几何失去覆盖后来发生**：21.5秒仍有810个真实同表面双眼像素，其中786个距seed XY≥10cm，但模型通过数为0；23.0秒只剩2个，23.5秒及之后为0。此时左眼仍可能有大量墙面像素，不能用左右都各看到墙/共同格代替同一表面真对应。
3. **原始源缺少新几何**：确认器只能检查已有提议，无法凭重复的旧墙面图像生成缺失深度。单纯延长TTL、保留旧seed或调PID无法修复这一来源覆盖缺口。

这区分了网络/来源失败与真实FOV/遮挡丢失，不证明匹配一定可解，也不将renderer可见性写入决策。当前无新控制候选准入。

## 因果与版本核验

作者实际执行截断到19.5秒的40次source-only重建：150个pair文件、3450字段逐项一致；80个future RGB/pose trap在载荷前拒绝。未读取renderer/actions，未做archive-wide future-file poisoning。旧frozen输入、原predictor与计划SHA核验一致。新检查代码、assessment、图和行政失败保留；assessor曾因空数组bool类型与JSON int64序列化失败，均发生在成功评分文件写出前，仅修正数据类型，不改变预测、标签、门槛或重跑物理。首次调用错用不存在的系统python也无科学产物。此新诊断目前作者核验，不能套用旧控制候选独立审查。

先前qualified-hazard独立审查已追加关闭：C0/I2/M1，两项Important为原导航/来源前缀准入阻塞；RGB与source首次差异在frame0。unknown布尔字段仅表示意图，实际23个拒绝格实例0个floor、1个仍由其他合法支持occupied。拒绝提议不会产生新的free观测，不意味着所有格必须unknown。旧字节保留，无重跑或安装。

## 已接续的工作

正在复用相同60曝光核验**同一眼的时间视角**是否可在至少.05m运动和.5°视差下再次看到同一真实墙面。只读真值用于物理可观测性上界，不将GT伙伴送入网络/控制；它不是对本次四视图规则的救分。没有足够时间视角则不启动另一匹配模型；存在视角也只允许另行事前冻结一个几何输入验证，旧特征/模型拒绝仍保留。需要先把可靠新区域几何与可执行参考连起来，才有理由继续完整视觉导航；不能把当前局部精度通过冒充导航完成。

## 产物

- `causal_four_view_new_region_v1/protocol.json / prediction_seal.json / records.json / assessment_v1.json`
- 所有pair的source/UV/NCC/baseline原始证明；`prefix_verification_v1.json`
- `coverage_failure_audit_v1.json / mutual_visibility_audit_v1.json`
- `four_view_diagnostic_result_v1.png`
- 原actual run的双目、第三视角视频和物理记录保持不变，未新运行或宣称本次有新视频。

报告写入时间：2026-10-07T02:42:24.006883+00:00。


## 2026-10-07T02:48:31.627254+00:00 temporal visibility CLOSED; ONE ray sweep source diagnostic STARTED
All60sameeye past4within2s offlineupperbound:235left31505/right20859,24538563/28861,25540011/32983 same-surface points withbaseline.05/parallax.5. GTpartnersNEVERpredictorinput. Six source-onlystandardcvrectify2slag alpha1 gotnegativefocal/0currentvalid -> fixedrectificationprobeREJECT/noalpha scan/noFastinvalidwarp. NewONEindependentanalytic raydepthsweep specifiedbeforecode/data, current235/245/255eachleft/right/currentpast1s2s. Knownpose/CAD/RGB ONLY;depth.5..3step.02/full7pxpatch/std5/minbothNCC.8/twocambaseline.05/parallax.5/uniquenessmargin.1 outside±.1. No globalplaneprior/newnetwork/threshold tuning, no action/free.5meaningfulTDDscaffoldRED→GREEN fixtures geometry/texturedplane/future/CAD/textureless/stationary; full482PASS22.81s beforedata. TWOtemporalendpoints+currentGT geometry≥.95allnonempty/100hazardsatleastcase+post23.5newcoverage20pix2binsinBOTHeyes frozen; developmentnotfresh/estimatedposeNOTadmitted. ActualCPU predictor session80981 running, source-only SIXoutputs sealBEFORElabels; checktemporal_ray_sweep_diagnostic_v1/prediction_seal.json andpredictionlog beforeduplicate. No simulatorrunning. Aftersealassessall6/current+twopast endpoints andlabelinsuffdenominator; rejectalone/noothersteps/NCC/networklag scans. Oldfourview/control/ORB/LightGlue/nativepose/global-source/navrejects unchanged. WholeNOTcomplete/automationACTIVE/formaloriginal35E0E1E2notrun.

OldprefixSHA f18d2572a4cd3aa2961d77b49a62c7fd842f478861d6a5c8f6053b8c2c432a10
