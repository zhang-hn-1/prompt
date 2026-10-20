# 动态机体mask来源上的10秒记忆实际局部验证

唯一primary改变current→history10s，源网络/当前三关节CADmask/knownpose/场景/2m任务/.3未知cap/PID/规划器/相机/光照均不改。ONE实际序列，原35秒含6秒观察，在22.6秒到达、终距.193028772m、接触0。局部实现任务通过；“记忆导航增量”拒绝：与已执行current全部请求、applied、物理轨迹相同，没有客观行为改善。旧history35秒失败及旧source gate保留，不能把新源版本成功追写为旧baseline成功。整个研究尚未完成。

## 实际运行身份
run_masked_memory_navigation.py从已封存current-mask runner逐项逆派生字节一致。PriorRGBPolicy(memory_mode=history)，PointSupportMap同10s TTL，不改记录/源/query时间。新增ReadOnlyCurrentMemoryShadow独立current-only对象，在同一实时已mask过滤包/当前observation/goal上调用；无未来输入/真深度/Oracle路径。旧base-mask包仍存日志，未用于任一在线consumer，本次科学改变只有记忆方式。

UTC2026-10-06 20:28:24.923422→20:34:46.236418；退出0。226动作/227同步双RGB/1131物理采样/46真实双目推理，0曝光偏差/0物理接触，无内部重置。两视频227帧10Hz全解码。物理仿真暂停做推理和CPUmap，非10Hz实时保证。路径2.332308m，真实目标方向进度1.828553m，直线横向RMS.039155/peak.104423。不是完整6.6mE1或正式成功率，也不是末端停稳。

## 记忆来源与因果接口
historical map峰值65格（occupied+floor，来源在过去，不表示当前body可通行或有效地面覆盖）。作者按完整来源记录/query时间/TTL逐帧重建226history地图1356数组完全一致；同流current影子2712诊断字段一致，226请求差异0。缓存来源时刻不刷新，未来payload trap拒绝，prior不变观测自由、unknown不变free，真实深度和位姿独立truth只用于离线诊断；knownpose在线权限始终明确。

与实际current-mask基线比较：全部226请求、1130 applied ticks、1131×17非raw物理字段、227执行状态完全相同；no regression true，但实际request/applied变化均0，history_increment_pass=false。不能把持久化65格等同改善，也不能将未执行影子的计划当作替代未来真实标签。RGB重置仍可能有渲染差别，实际两个序列结果相同不证明所有图像/来源都相同，不把源变更前后差值当作纯记忆因果。

runtime对影子逐次断言主动作/guardedtracker边界/物理clock与世界位姿、线角速度、关节/关节速度不变，独立对象与拷贝输入。小CPU测试及实际assert通过；没有另开完整物理shadow-off运行，不夸大为全面运行隔离证明。

## 实际测试/数据证据
新增2fixture先import RED→GREEN，连同4个sourcepair回归共6pass3.96s；上一完整379suite不在本次重复跑，上一review发现的输入快照Minor另有独立预调用回归1pass1.04s。source1264预封存+关闭后产物2993SHA核对；46来源/当时CADmask1012字段、全部history/current-shadow地图验证，两全视频。旧CAD四姿态FK/rendererID验证已完成且冻结，此次未重做标定。删3393源点的离线诊断含2269准确/848错误/276非有限真值，不把全部删除称坏点，不以删点提升条件精度宣称全域模型通过。

产物：masked_memory_history_run、masked_memory_navigation_assessment.json、masked_memory_navigation_actual_and_shadow.png；原始两完整视频/全physics/maps/currentmask/sourcepredictions/provenance/readonlyshadow均保留。评估脚本由前阶段封存评估器改名、primaryhistory/影子current同maskedsource重建和实际prefix比较派生，明确是作者离线核验，不冒充第二次实际在线接口。

## 下一项控制消费者：名义请求被外层cap覆盖
本阶段只做一次基于现有实测数据的预算/请求链核验（masked_memory_budget_audit.json），没有新增标定/拟合/动作/路径改动：166个运动块的原tracker请求全在.5/1/1.5rad/s，最终全被同一后置unknown cap压到.3；166个BankMap调用中161个实测速度低于冻结稳态插值域，返回clamped。运动阶段实测平均XY speed .131354m/s/p95 .184302；原6.6m目标在29s运动预算要求直线平均.227586m/s，已知Oracle路径8.835802m的算术需求.304683m/s。这是当前有限序列与算术预算证据，不是全域可达性证明，也不是证明所有失败归于速度。

下一项只验证unknown主轴cap .3→现有名义默认1.0（保持高曲率旧.5调度、aux角、PID/aux速度限制、planner、memory10s、源mask/rig/light原样）。该值来自已验证原控制器默认调度及已有Oracle执行域，不扫阈值，也不重做响应矩阵。先同2m/35s实际候选对已执行22.6s基线，同一实时source有原.3 history只读影子；只有真实到达/无接触/无runtimeerror、时间和终距无退化、独立进度/横向误差及实际执行差异通过，才进入完整目标。提高未知空间速度增加风险，仍是用户已授权的显式prior/受控未知仿真实验，不能转成安全认证。所有旧source/正式benchmark/estimatedpose/世界模型层次仍待验证；不因本报告或局部任务结束停止续做。
