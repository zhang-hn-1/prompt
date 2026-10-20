# 局部参考连续性：58次交接与两次侧切换的因果核验

2026-10-05 04:41UTC heartbeat。一项有界、实际执行的归档数据实验已完成；没有新仿真/控制改动/模型推理/训练。完整研究未完成，automation ACTIVE。H=`experiments/stereo_memory_navigation_20261004_v1`，旧数据源 H/`depth_local_curve_run/`。

## 结论

两次局部目标侧切换的 **几何触发来自新增已观测障碍**：固定机器人位置，仅替换地图就足以切侧；只替换位置而保留旧地图不会切侧。旧整条A*路线在新图上各有10个顶点不满足原.6m栅格规则，最小clearance .4m。因而不能把两次重新选路直接判为planner bug或无理由抖动。

但“远处完整路线失效”和“当前已经安装的短段应立即丢弃”是两件事。7.5s旧局部短段剩余2.3134m，当前位置距它.045706m，连续polyline距已观测hazard至少1.027154m、原栅格最小1m；它仍通过两种必要检查。16s旧短段剩余2.3164m/投影误差.003219m，原栅格最小.6m通过，但连续polyline proxy仅.567948m；这在15.5s安装时已经如此，**不是本次新增hazard把局部短段变窄**。

预先规定的“两个切侧事件都可保留旧参考”必要筛选未通过，因此拒绝这个无条件保留提议，未启动控制A/B。不能通过删除连续检查、调整.6或宣称栅格通过即动态安全来制造通过；没有倒改旧35s timeout和gate。

## 方法与冻结预算

完整读取 mission293行、主计划252行、ledger394行、最新阶段报告89行和旧local spec/plan；上次真实run finished/exit0。2798旧产物和5旧文档身份精确核验。新设计/计划：`docs/superpowers/specs/2026-10-05-reference-continuity-audit-design.md`、`docs/superpowers/plans/2026-10-05-reference-continuity-audit.md`。用户已授权自主研究，保留H隔离/unborn master，无无关Git提交、清理或原项目写入。

预封存 **04:50:01.580440UTC**，77源与2798输入SHA；测量 **04:50:01.698168—04:50:02.974850UTC**，退出0。实际预算：58个原.5s replan事件，首个无旧参考N/A，其余57个；各测当前状态和安装时状态共114组几何。只在frame75/160做 **8次原grid A***：旧/新地图 × 旧/新实测XY。没有Hybrid重跑、额外矩阵、别的配置或物理alternate labels。

`reference_geometry_status` 使用旧已安装polyline，连续线段最近投影后保留剩余suffix，显式加入当前XY→投影点的bridge。按原.6m hazard-centre threshold独立报告连续距离；在suffix/bridge≤.05m间隔采样、取原rint格坐标和padded EDT/.6规则，另报告栅格检查。Unknown不改free，地图只读；hazard为空返回null clearance，不凭空观测地面。缺失/重复/NaN/Inf路径拒绝，不变成空安全路径。

必要筛选是同时通过两检查、remaining arc≥原lookahead .5m、projection distance≤.5m；它只是本提议的诊断screen，**不是正式导航验收、安全证书或针对旧结果的新gate**。连续polyline与原nominal primitive曲线/栅格格点不同，.567948不能直接证明真primitive或真实球壳发生危险，也不能直接认证原planner实现错误。实测速度/roll/关节/停止尾迹没有被这个几何消费者预测。

## 两次切侧的2×2实验

所有输入为对应当前或过去观测图、实测XY及已知全球goal；不读未来运动作为输出。目标是A*前2.4m所选点，其world-y为：

| 事件 | 旧图/旧XY | 旧图/新XY | 新图/旧XY | 新图/新XY |
|---|---:|---:|---:|---:|
| 7.5s（旧7.0s） | -1.4 | -1.5 | +1.4 | +1.4 |
| 16s（旧15.5s） | +2.2 | +2.4 | -2.0 | -2.0 |

这隔离了 **此次确定性A*目标侧选择** 的输入触发；不隔离实际35s超时的原因。两事件新图分别新增3格、2格hazard，来源坐标保留在assessment；用已保存的旧完整grid route检查新图，均10个旧顶点失效/min .4m，位置在更远的障碍端部。切换不是“当前位置变了导致最近投影跳侧”的实证。世界y符号也不是形式拓扑分类的证明。

四个对角组合重现旧原实际map/XY时保存的整route与target精确一致。非对角虚线路线是 **几何反事实**，没有真实执行，不能把旧B0轨迹当它的标签。

| 旧已安装短段（当前状态评价） | 7.5s | 16s |
|---|---:|---:|
| 剩余arc m | 2.313414 | 2.316381 |
| current→reference距离 m | .045706 | .003219 |
| suffix连续clearance proxy m | 1.027154 | .567948 |
| bridge连续clearance proxy m | 2.843664 | 1.298287 |
| 原栅格min clearance m | 1.0 | .6 |
| raster / continuous | pass / pass | pass / fail |
| 本提议必要screen | pass | fail |

57个交接的44个通过screen；7个连续距离不足、6个栅格距离不足，本次两类不重叠；没有projection>.5或剩余arc<.5的拒绝。这是B0实测状态下的几何统计，不是44次安全动作/导航成功，更不能推演保留参考后仍经历同样状态。

![当前观测和局部参考](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/reference_continuity_events.png)

图中灰/红点只来自过去/新增已观测hazard，没有完整真值occupancy。黑线为此前真正安装的局部曲线；虚线为固定当前位置、替换观测图的A*路线。图片已实际检查。

## 测试、实际数据与验证

六个literal行为测试先missing-module RED，再callable RED6→GREEN，总H套件 **154 passed / 4.39s**。覆盖线段中间投影、后方hazard排除、bridge碰撞、长线中点漏检、空hazard不seed、无效几何。未修改旧封存测试。首个作者命令因Hcwd配W相对路径全部写入失败，仅生成file-not-found pytest日志，归档 `reference_continuity_path_failure.log`；之后正确路径创建，非数据后参数更改或模拟失败。

代码 `reference_continuity_audit.py`、`run_reference_continuity_audit.py`；协议 `reference_continuity_protocol.json`；全部58行/8反事实 `reference_continuity_assessment.json`；8条完整route NPZ在 `reference_continuity_routes/`。旧第三视角/双目视频保留供运动证据，**本轮无新视频/导航结果**。

作者验证 **04:52:36.920731UTC**：77源/2798输入不变，114组由独立scalar线段投影/距离计算复核，5809个栅格sample用暴力nearest hazard-centre＋padded边界距离核验，4条对角实际原route/target完全一致；0额外grid searches。各hazard来源时间不晚于map时刻。旧完整路线的新图失效坐标作为独立描述witness保存于 `reference_continuity_verification.json`，不是新增控制/物理实验。初始assessment和日志保留，未追改筛选。

## 接续工作：一个有有效性保护的参考消费候选

关闭“两个侧切换一律禁止”提议。下一项可明确测试 **只在旧短段仍通过两检查且未耗尽时保留它**；一旦新观测使短段失效、投影过远或remaining<.5，即交还原planner重建，允许真实合法换向。7.5s有效局部prefix与远处global route失效的分层证据支持这一候选的必要性，但本轮未实现/未准入，不能称已经解决。

下一次先冻结唯一consumer和状态化测试，证明失效/耗尽会触发原重建、不锁死方向、不等待无法开始换向的条件，保留五低速/反弹/smoother/PID。随后最多一条同初态、同传感/unknown-risk域、同6s前缀和原35s golden E1对归档B0；只改变reference替换时机，不改planner算法、速度、映射、增益、相机或旧gates。验收必须真实到目标/空间检查点、独立参考进度和接触；几何省调用、停车或path_index前移都不算成功。未知风险仍非完整3D证书，curve可保留不等于短时物理模型已通过。

本轮只有这个归档几何实验；没有偷偷追加控制试验。RGB来源/估计位姿/学习增量/正式E0/E1/E2仍待验证，旧拒绝保留，研究继续ACTIVE。

## 独立审查与作者收尾

以下为审查返回后的作者附录，未冒称对附录进行了第二次独立审查。审查完整原文与六项 scope 裁决保存在 `reference_continuity_review.json`；审查正文 SHA256 为 `cc7a132a8e672b003456dc16b9a6e40d468de232a1c2db5373e843757b60a0b6`，原正文逐字保持。仅一次 fresh-context 只读审查：Critical 0 / Important 0 / Minor 0，无需修改冻结实验或测试。

审查者亲自运行六项测试（6 passed / 0.15s），检查77源和2798旧输入身份、58事件出处、全部114组投影与连续几何、八份路线及四个历史对角线、两条整条旧栅格路线失效见证并查看图。作者154项全套测试与5809个栅格标量样本为作者证据，审查者未重新运行全套或逐个重算全部栅格样本。双方无新增物理仿真；审查者无规划搜索、文件/Git修改。

作者逐条接受审查排除范围：

1. 不判断保留参考能否改善实际导航，因为没有另一条物理轨迹。
2. 不认证动态制动、三维碰撞安全或未知空间可通行；这里只是观测障碍的名义几何。
3. 0.567948m 的连续折线量不证明原运动 primitive planner 有缺陷，两者几何定义不同。
4. 因子对照只隔离地图总体变化与XY对目标侧选择的影响，不隔离单格障碍、换参考对35秒超时的贡献。
5. RGB来源、估计位姿、学习增量和正式E0/E1/E2尚未取得本轮验证。
6. 带有效性检查的参考保留仍是未实现的下一阶段假设，不宣称控制器已通过。

作者最终收尾只追加任务记录并核验来源/文档/产物身份，不重算科学结果；最终核验见 `reference_continuity_final_check.json`。本轮归档几何阶段可交接，整体研究未完成、自动续做保持ACTIVE。下一阶段实现与受限真实控制实验另行封存，不把本次拒绝改成通过。

收尾首次文档命令使用不存在的 `python` 别名，退出127且未写入文件；换用既有环境的绝对路径后成功。该作者命令失败单独保留，不计为仿真失败或新实验。
