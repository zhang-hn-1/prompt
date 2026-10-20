# 固定相机负反证空间记忆：真实在线导航与停止原因

本阶段实际运行两条完整对照，均未通过导航。R1的局部来源覆盖通过，不代表位姿、控制、完整导航或原35秒benchmark通过。整个研究仍未完成，学习增量及原E0/E1/E2未触发。

## 实验与身份

采用新条件任务：原固定相机X=.20,Y=±.30,Z=.05、原光照、平坦地面先验、安全起步区.7m、未知探索；同起点[-2.2,.15,.45]和目标[2.2,.15,.45]。60秒总预算含14秒固定观察：主轴始终0，6–8秒bank+10°，8–10秒−10°，其余0。内部不重置物理、PID、smoother或方向状态。14秒后R0冻结最后当前强来源集合；R1冻结全部过去强来源核心，删除被当时NN端点射线反证的MODEL假设。最近query pixel<=sqrt(2)、opticalZ差>.20为冻结规则，没有删点真值或阈值扫描。

保留旧来源域拒绝。MODEL不是实测占据或自由；ground/safe-start先验单记；类别模型约2秒出生、汇总池约14秒出生分别保存；60秒静态持续仅是假设。位姿仍依赖一次初始gauge、理想bodygyro/编码器与floorheight先验，不是原生IMU或纯RGB定位，也没有完整3D证书。

控制保持现有roll-PID .4/.1/.7、副轴速度supervisor .3、原smoother、主轴未知cap+.5/−.3、原planner/方向锁存/五次低速确认。规划半径.870710678（.6+.2+栅格项），保留原参考连续与raster .6检查。

v1实际R0在14秒首次导航帧发生capture-clock接口错误，R1未运行。完整原始前缀封存，不视为导航失败或成功。独立v2仅让观察阶段通过原packet验证并消费帧时间，不调用act、不推进tracker/planner或reset。真实控制器2个CPU回归通过，全套631测试41.02秒。v1与v2数据均保留，不能把已见前缀当全新跨域held-out。

v2 protocol SHA256 `f257ce6eb913cc720492a5c0ac086f5e189775cf1e0a3736068a3ac5fa2d5982`绑定1411项源码/配置。R0实际2026-10-07 15:18:31–15:27:00 UTC，wall509.500秒；R1 15:27:01–15:35:27 UTC，wall505.665秒，均exit0，无强杀。每条600次请求、3000物理派发、601双目/第三人称视频帧，实际在线调用而非事后回放。

## 分层结果

| 项目 | R0最后当前来源 | R1历史来源+负反证 |
|---|---:|---:|
| 局部物理足迹覆盖 | 171/185=.924324，拒绝 | 185/185=1，通过 |
| MODEL可能面积 | 1.68m² | 2.04m² |
| retained raw核心 | 590 | 804 |
| 历史反证数（R0仅诊断） | 33 | 32 |
| 真实占据核心误反证 | 0 | 0 |
| 所有贡献类别视图最小precision | .994923 | .992496 |
| 位姿3D RMS/peak | .011725/.024836m | .001971/.004507m |
| 姿态误差p90 | 1.792385°，通过 | 2.619401°，超过2°，拒绝 |
| 实际终止 | 60秒超时 | 60秒超时 |
| 最后XY/XYZ目标距离 | .214930/.220670m | 3.268264/3.268646m |
| X轴真实净进度 | 4.562509m | 1.145808m |
| 实际通过障碍x>.75 | 是 | 否 |
| 实际路径长 | 8.001541m | 2.922511m |
| contact/collision帧 | 0/0 | 0/0 |
| 球体代理最小间隙 | .426011m | .388428m |
| 导航零请求帧 | 62 | 351 |
| NN来源wall p50/p95 | 2.032391/2.306440s | 2.072181/2.320679s |
| 最终准入 | REJECT | REJECT |

两条起步max位移均.155327m，14秒末连续27个真实XY低速采样。两眼同曝光0差。原NN每次两次网络forward及warmup，暂停physics计算，2秒级wall时延不能称10Hz实时系统。全部安装的名义参考满足原map .8与独立真圆柱 .8检查；保留桥/后缀满足独立真值 .6。球体代理间隙不等于完整CAD的3D保证。

传感器链分别核验3001份实际理想传感器、3001预测、9015位姿字段、1210来源字段、3600actor字段；过去截断501样本一致，未来query拒绝。源重建先于GT，每臂419原来源字段、原池/所有射线proof/冻结seed精确重建，未二次调用有状态policy。执行clone每臂2995物理步smoothed/applied与快照精确一致，地图3220字段精确一致，所有measured-free/hazard时间仍−inf。

两臂141采样物理/关节/状态/smoother、估计位姿和历史输入一致，29份CAD mask一致；RGB、NN、source packet并不相同，possible-set只13/29一致，全physics-prefix档案也不精确相同。因此**不是严格配对A/B**，不能将结果差异归因于唯一消费者，更不能报告记忆使成功率下降。单臂拒绝依据仍有效。

## 有界停止原因核验

R1 frame265/26.5秒首次永久零请求，此后33.5秒没有非零请求。此前实际初始参考独立横向RMS .165935m、peak .321407m，实际进度1.764837/初始参考2.694582m。原.2m经验tube未覆盖该运行，不能称已认证执行误差。

在首次停止当时，原guard后缀clearance .914308m、bridge连续.625943m，但bridge原raster只有.583095<.6，故合理拒绝保留参考。当前估计root[-1.034807,.778875]的原规划EDT同为.583095<.870711，不能重建。唯一只读反事实：相同冻结地图在原guard投影[-1.220943,1.020333]的EDT .943398，原planner能找到route。说明实际偏离使重建起点进入原膨胀区域；没有放宽检查或瞬移机器人。

附近MODEL栅格[-.700000,.250000]，最近保留核心[-.666995,.092711,.65]；原真实圆柱半径.35，这个MODEL核心处在真实柱外。类别precision/覆盖通过并不能保证每个3D定位核心都准确。当前真实root到真实圆柱表面.944951m；未发生碰撞。停止后仍缓慢漂移.327309m，零请求不等于物理完全静止。

R0确实绕过障碍但60秒未到达。frame510的kinodynamic返回success元数据、0长度/1 expanded node，adapter因不足2 waypoint没有安装plan并给出零动作。原v2 scorer误以为每个success元数据都有文件而报FileNotFoundError；失败日志保留，独立v3仅核验该0长度/1节点/零动作分支并只评价实际安装的参考，科学gate不变、运行源码未改。不能将这类元数据当真实到达。

原因不能简化为首次换向bug、碰撞误判或M2没通过。实际执行偏离与来源可能范围偏宽叠加；当前冻结池不消费14秒后的新几何，已有移动观测能否及时反证保守假设还未核验。

## 证据与接续

实际原始数据/双目/第三人称视频：`experiments/stereo_memory_navigation_20261004_v1/negative_ray_memory_R0_run_v2`及`negative_ray_memory_R1_run_v2`。两臂post-close SHA分别`166d6842c90e5a1f56257744bd9a81602cd84acd2aca94632164fb752dd2bb20`、`60f8e156930995a21153c0f98967fdf927a51615eef2160631b76175afd95110`。

核验：`negative_ray_memory_{R0,R1}_{sensor,source}_verification_v2.json`；最终`negative_ray_memory_actual_assessment_v3.json`及轨迹`negative_ray_memory_actual_v3.png`；唯一因果核验`negative_ray_memory_stop_cause_v1/audit.json`、`stop_cause.png`、`actual_video_frames.png`。作者实际执行核验并亲自看了轨迹/参考/视频抽帧。此前一次feature review C0/I3/M0，作者修复strictprefix/独立真guard/独立错误文件；没有声称二次review或reviewer跑过GPU。

下一项有界实际工作：在唯一停止边界，只用截至26.5秒已有actual双目端点和估计位姿，核验移动反证是否改变阻塞、是否误删除真实占据、覆盖95是否保持。它是历史因果核验，不是新轨迹标签/在线成功；若失败不调.20/NN/PID/安全半径救分。通过才独立冻结动态记忆消费者及完整序列准入。当前R1的pose拒绝也保留，不能用该诊断替代估计位姿导航验证。学习增量/原35需相应前提，不跳层。

## Subsequent bounded moving-ray audit (2026-10-07 15:56 UTC)
At the unique frame265 boundary,54 past actual NN/estimated-pose packets veto36 cores, retain800 and preserve185/185 coverage,1.96m2 area,0 true-occupied-core veto. The original planner still reports observed_obstacle_blocks_route. REJECT_UNIQUE_MOVING_RAY_AT_STOP_BOUNDARY; no dynamic-consumer simulation or threshold change. Actor outputs were sealed before GT scoring. Next is exact factual controller reconstruction and discrete-lookahead/saturation measurement, not another source-rule scan.
