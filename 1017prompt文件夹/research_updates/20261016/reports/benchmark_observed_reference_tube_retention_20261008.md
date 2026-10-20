# 已观察参考管道保留：唯一消费者实验（2026-10-08）

## 问题与范围

上一条真实双目记忆导航在42.6秒开始停滞：当前桥栅格距离.824621m不满足.870711m combined screen，而名义剩余参考.900m仍通过。原guard与当前参考误差.071492m均在范围内。旧导航REJECT与原门槛保留；本实验是独立标注的消费者条件改动。

唯一改动不是统一降低安全裕量。对原guard通过的旧参考，保留名义后缀原.870711m检查；只在桥连接同一投影、当前距离≤原.2m allowance且EXACT continuous后缀到已记录风险格中心距离−.2≥原.6时允许继续原参考。桥每一点位于投影点半径.2内，三角不等式证明当前桥与每个已记录风险格中心的.6距离。名义后缀碰撞/域外/原guard失败均不例外，旧 combined PASS维持。

此证明仅针对已记录风险格中心；.2未来跟踪量没有被认证，停车尾迹、未观察空间、3D安全、来源全域及物理运动仍不被证明。条件TaskH允许受控未知探索，但不是把未知写成自由。所有风险点、source时间、相机、光照、PID、速度、映射、五次停稳、方向规则、规划器均原样。只修改 armed reference-retention consumer。

## 真实准入证据

10几何单元测试和6既有前缀测试通过；最初RED为模块缺失，明确不是10个行为RED。reviewer独立10测试通过。同一次首次审查最终Critical0/Important0/Minor0；首次reset capture的执行前contact/goal检查遗漏修复于首次实际运行前。

封存旧开发日志的current/past重放：426个前置请求与B0实际相同，427th（frame426）首次改变，从零请求改为+.5/−.317435。当时名义suffix raster.900/continuous.861955、bridge.824621、投影误差.071492；continuous minus.2=.661955≥.6。未来状态替换保持相同。这是接口/几何必要准入，不是另一计划物理真值或fresh validation。

独立B0影子工厂加入相同固定front-view采集前缀、相同RepeatedExposureMemoryGrid；427实际历史请求及65/121/273三个截断/未来替换精确重建。native import/MRO/reset无act、2078源码/84输入SHA核验通过。相同已知位姿静态条件，非纯RGB位姿估计。

## 实际运行

唯一候选已于13:00:30 UTC启动，`observed_reference_tube_navigation_status_v1.json`、`observed_reference_tube_navigation_run_v1/`，最多1200秒墙钟，不重启。原54.6秒目标剩余≤4.1829005/真实轴进度≥2.427377623/无接触gate不变，只有同一序列通过才延长至120秒。原35秒E0/E1/E2仍未触发。只读B0 shadow截至首次请求分叉，没有另一计划未来真实反馈标签。

结果尚未关闭封存，不报告导航通过。届时分别核验source/membership、执行链、memory/maps、请求因果、真实影子on/off、真进度/碰撞/到达和视频。失败明确拒绝，不再换阈值/增益/网络/候选救分。整个研究仍未完成。

## 13:15 UTC：实际关闭、完整核验及非干预拒绝

实际13:00:30—13:07:14 UTC退出0，546请求/547采集/2731物理/110双目推理。原封存SHA256 `23080517baedf08a60169bf117edb2cd661bfa90a6c14b6a354bfeb5f4e8a00d`；3277接触测量均可用，contact0。54.6秒真实目标剩余5.692222m、目标轴进度0.960883m，未达原门槛，未延长120秒。

**控制候选：REJECT_NONINTERVENTION_CANDIDATE_NOT_EXERCISED。** 在线B0影子全546请求相同，例外条件当前边界触发0次；不能归因候选改善或恶化。**导航：REJECT_UNIQUE_OBSERVED_REFERENCE_TUBE_NAVIGATION。** 当前规划位置裕量最终1.923538m，旧42.6秒桥接停滞未复现；这次整个失败不能归咎于它。

分层证据：2730执行tick的实测反馈重建完全一致；546地图/来源快照与TTL、两次epoch确认完全重建；546请求及65/121/273未来替换与截断精确；真实在线影子日志逐值重建、影子on/off不改变候选请求和参考，65/155/545未来文件读保护通过。来源69,880点中98.7149%≤10cm，无条件p90=.038866m、898bad；GT场景障碍成员99.1857%。都是选定表面/已知位姿子集通过，不倒改旧全域来源拒绝。动作墙钟p50=.327857/p90=2.162758秒，0动作≤100ms，非实时。原35秒、估计位姿动作层、新学习增量仍未触发。

### 实际失败分段

初始观察11.6秒；正向请求12.3秒；倒车18.1秒；方向确认零请求12.6秒；无路线零请求0秒。10次规划均成功安装。最长末段倒车9.9秒损失X进度.878m；另两较长倒车损失.344/.288m。不是停车等待或漏碰撞判定造成的此次失败，不能用局部投影指标代替目标进展。两段547帧视频均解码计数；作者查看原始双目frame400、实际轨迹和最终第三人称截图，并抽取六个关键时刻，未声称全视频逐帧观看。

### 继续执行的有界跨运行因果核验

旧front-view导航与本次的真实物理字段（除独立raw绝对原点外）在1055tick之前一致；首次请求差异在frame211，对应1056tick。早在frame0 RGB已有差异，但相机/球心位姿一致：左图平均绝对差.5587灰度、p90=1，右图平均差.4278；差异不是修改相机/光照的授权，更未证明某一个渲染随机机制。预测与留存点也从frame0不同，第一次参考几何从frame116不同。两次首次参考都只含一次R→F，不能说新参考多了一个初始cusp。

为了区分当前地图与已产生的参考/控制状态，先冻结4个当前输入对照：相同实测frame211输入，交换A/B当前/过去地图来源状态与A/B已有控制/参考状态，不使用未来物理。AA和BB实际请求精确复现；AA/AB均+.5/−.523599，BA/BB均零。由此本次第一动作差异在该对照中跟随已经形成的控制/参考状态，而不是仅换当前地图就逆转。B的旧参考保留检查触发重建后，进入F→R确认；A继续当前F跟踪。它不能证明任意来源噪声都无关——较早观测已经参与产生不同参考。

第一次2×2工具v1断言失败：用旧B0 diagnostics schema与新helper的屏幕字典比较。v2只去掉这个跨schema比较，保留全部实际请求/参考及相同测量断言，四个对照执行通过；原v1文件/失败保留，原物理与门槛不修改。

实际证据已保存 `observed_reference_tube_progress_assessment_v2.json`、`observed_reference_tube_intervention_assessment_v1.json`、runtime/source/membership/memory/causal/online-shadow报告、`observed_reference_tube_crossrun_first_difference_audit_v1.json`、`observed_reference_tube_prefix_RGB_variation_v1.json`、`observed_reference_tube_source_reference_2x2_assessment_v2.json` 和 `observed_reference_tube_failure_segments_v1.json`。几何候选不再次运行、不安装为通过模型/控制器；下一项工作只针对已经定位的正常运动重新规划与立即换向请求接口，不扩大Oracle/NN/阈值矩阵。整体研究未完成。
