# 独立完整序列校准：保守残差门控实验（2026-09-26）

**实际完成 4 条新序列的仿真采集、数据核验、冻结模型特征提取、单个门控校准与历史回归。原 28 项验收通过 16 项。** 候选仍拒绝，封存 fresh 测试未触发。 本轮没有修改基础运动网络、执行限速或导航控制。

## 1. 本轮只检验什么

上一轮门控使用基础网络的训练内残差，验证/测试没有触发匀速回退。本轮检验：将校准数据换为基础模型从未拟合过的完整新序列，在同一门控结构和规则下，是否能减少错误修正。

复用逐字节相同的 175 参数门控：当前本体17维 + 0.4秒状态趋势17维，线性 sigmoid 输出五个时域的缩放系数；系数<0.5精确回退匀速，否则按系数缩放冻结网络残差。标准化只来自新校准数据；Adam lr=.01、L2=.01、固定500步、零初始化全部不变。未来实测位姿只生成校准标签或评估，不作为预测输入。系数不是已经验证的概率。

基础模型仍使用冻结的双目特征与本体历史；新增数据有真实双目图像。本轮不证明视觉贡献、地图预测或候选动作反事实排序已经可靠。

## 2. 实际新增数据与完整序列隔离

预先固定 4 条×24秒，50Hz物理/状态、10Hz双目；共 **4800 条状态、960 对双目图像、924 个校准窗口**。每条开头完整重置，内部按请求计划连续执行，不重置物理状态或 smoother。校准数据处于旧训练使用过的 easy_1/easy_2 场景，但请求、种子和初始朝向是新的；它是新序列，不是新场景或 fresh 测试。

| 序列 | 种子 | 朝向 ° | 请求模式 |
|---|---:|---:|---|
| easy_1_cal_forward_exchange | 2026092601 | 47 | 前进、转向反转、停车、倒车、停车 |
| easy_1_cal_reverse_exchange | 2026092602 | 137 | 倒车、转向反转、停车、前进、停车 |
| easy_2_cal_speed_histories | 2026092603 | 227 | 不同前后速度、停车与再起步 |
| easy_2_cal_low_and_reversal | 2026092604 | 317 | 低速停走、前后换向与转向 |

没有按单帧随机拆分：这四条全部只做校准；旧验证/测试各两条仍为历史回归；原先封存的四条 easy_4/easy_5 fresh 序列不参与校准。与上一轮相比，校准总量从192秒变为96秒，状态分布也不同，因此不是仅改变“独立性”的纯消融。

以下报告实际运动而非只报告请求。低速定义为世界XY速度<0.05m/s，前进/倒车为沿瞬时偏航前向速度>0.05/<−0.05m/s；指标不必构成互斥完备分区。接触标记来自每条240个视觉时刻的几何标签，并非1200物理步的接触求解器计数；未因结果不佳而删序列。

| 校准序列 | 实测前进 s | 实测倒车 s | 低速 s | 路程 m | 接触标记 /240 | QC |
|---|---:|---:|---:|---:|---:|---|
| easy_1_cal_forward_exchange | 7.98 | 8.48 | 7.08 | 2.792 | 11 | 通过 |
| easy_1_cal_reverse_exchange | 4.70 | 12.56 | 3.26 | 3.593 | 0 | 通过 |
| easy_2_cal_speed_histories | 8.94 | 9.14 | 3.32 | 4.377 | 2 | 通过 |
| easy_2_cal_low_and_reversal | 8.10 | 6.62 | 5.60 | 2.692 | 0 | 通过 |

**数据审计例外必须单独说明：** 第三条speed_histories在完整采集后，被旧V2正式审计拒绝：没有超过16.1°的applied转向目标。其冻结请求本来最大就是16°，这是本轮预检未发现的覆盖约束冲突。首轮pipeline/status、失败日志、全部请求和原始数据均保留，未追改为通过。三条其他序列通过原审计；四条均通过全部29项结构/因果一致性检查。

在三条采集完成、任何特征提取及校准拟合之前，新增独立calibration-only eligibility口径：仅放行指定第三条、指定请求SHA、唯一高角度覆盖错误；任何额外错误、其他序列或fresh数据都不能使用该例外。它是事后披露的数据适用范围修订，不是完全预注册的审计通过。正式V2审计结果仍是3/4通过；模型28项门槛未变。

独立审查确认第三条实测副轴曾超过16°，但此事实不等价于applied目标超过16.1°，不能用于推翻旧审计。

复用原采集审计，检查真实相机时钟/帧递增、状态/图像对齐、协议裁剪、完整请求逐步一致，以及使用实测历史反馈的 smoother/applied 精确重建。每条首个实测位置/姿态单独保存；它是首个物理步后的测量，不冒称精确的重置前坐标。保留全部原始文件指纹、种子、命令文件、源代码和执行日志。

![四条新校准序列的实测轨迹](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/calibration_trajectories.png)

图中终点只是固定24秒请求序列结束位置，不是导航目标；红点是几何接触标签。轨迹以各自初始偏航方向显示，方便观察前后移动。

## 3. 实际校准与比较

只将924个新校准窗口传入 fit_gate；原训练、历史验证/测试及 fresh 都不参与拟合或标准化。固定500次更新，校准目标 0.167689 → 0.067439，未根据验证成绩选轮。整个采集、编码、校准和条件评估流程耗时约 14.33 分钟；采集耗时见逐序列执行记录。

下表为原验证/历史测试各94个相同窗口。XY为平均欧氏误差cm，yaw为平均包角绝对误差°。旧门控指上一轮训练内残差校准，基础模型指未加门控的冻结加速度残差。

| 集合 | 时域 s | 匀速 XY | 基础 XY | 旧门控 XY | 新校准 XY | 新校准 yaw |
|---|---:|---:|---:|---:|---:|---:|
| val | 0.1 | 0.0881 | 0.0874 | 0.0840 | 0.0892 | 0.0496 |
| val | 0.3 | 0.5850 | 0.4720 | 0.4489 | 0.5052 | 0.2359 |
| val | 0.5 | 1.5051 | 1.1163 | 1.0661 | 1.2025 | 0.5517 |
| test | 0.1 | 0.0763 | 0.0643 | 0.0627 | 0.0665 | 0.0393 |
| test | 0.3 | 0.5497 | 0.3448 | 0.3501 | 0.3976 | 0.1958 |
| test | 0.5 | 1.4636 | 0.7997 | 0.8415 | 1.0018 | 0.4209 |

![独立校准结果及回退比例](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/independent_calibration_results.png)

原请求变化分组仍指未来半秒内含变化，不等于预测起点已经换命令。

| 集合 | 时域 s | 匀速 XY cm | 基础 XY cm | 旧门控 XY cm | 新校准 XY cm |
|---|---:|---:|---:|---:|---:|
| val | 0.1 | 0.0484 | 0.0805 | 0.0752 | 0.0734 |
| val | 0.5 | 0.6664 | 1.0297 | 0.9481 | 0.8825 |
| test | 0.1 | 0.0448 | 0.0565 | 0.0519 | 0.0491 |
| test | 0.5 | 0.7411 | 0.6282 | 0.5847 | 0.6058 |

## 4. 实际回退与错误修正

以下比例独立于验收分数，检查门控是否真正改变选择。原门控在验证/测试五个时域均为0/94回退。

| 集合 | 时域 s | 新门控回退 | 平均保留系数 | 原XY方向错误数 | 其中仍保留数 |
|---|---:|---:|---:|---:|---:|
| val | 0.1 | 23/94 | 0.5931 | 29 | 22 |
| val | 0.3 | 11/94 | 0.7414 | 20 | 17 |
| val | 0.5 | 13/94 | 0.7386 | 14 | 11 |
| test | 0.1 | 18/94 | 0.5890 | 27 | 21 |
| test | 0.3 | 24/94 | 0.6302 | 12 | 11 |
| test | 0.5 | 25/94 | 0.6389 | 10 | 8 |

方向错误通过事后真实残差与预测残差的XY点积<0定义；只用于诊断，不进入门控输入。新校准集上的分数是拟合内表现，不能作为独立验证。逐序列/全部五个时域的实际使用比例在各 gate_usage.json 中保存。

按实际XY误差核验，历史test半秒的25次回退中，21次舍弃了原本优于匀速的基础预测，仅4次避开基础预测的劣势；不回退的窗口中，仍有18次基础预测差于匀速。缩放有时也能改善保留的预测，因此这些计数不能替代最终误差；最终半秒平均误差已由0.7997cm升至1.0018cm。

**结论：触发回退的能力出现了，但选择回退时机的能力仍未验证通过。** 0.1秒请求变化组比旧门控稍好，仍差于匀速；这种局部收益不能抵消半秒整体退步。新门控不是可部署的安全兜底。

## 5. 原门槛与 fresh 触发

原28项模板逐项保持一致，实际通过 **16/28**。半秒相对旧速度残差以及未门控加速度残差的比较对象均保持不变。

| 未通过检查 | 实际比值/数量 | 门槛 |
|---|---:|---:|
| val / sequence_0.5_xy | 1.014844 | 1.0 |
| val / request_change_0.5_xy | 1.324365 | 1.0 |
| val / overall_0.1_xy | 1.012512 | 1.0 |
| val / preserve_parent_0.5_xy | 1.132665 | 1.05 |
| val / request_change_0.1_xy | 1.516174 | 1.0 |
| test / preserve_parent_0.5_xy | 1.158821 | 1.05 |
| test / preserve_parent_0.5_yaw | 1.058209 | 1.05 |
| test / request_change_0.1_xy | 1.097514 | 1.0 |
| val / preserve_acceleration_0.5_xy | 1.077222 | 1.05 |
| val / preserve_acceleration_0.5_yaw | 1.081191 | 1.05 |
| test / preserve_acceleration_0.5_xy | 1.252766 | 1.05 |
| test / preserve_acceleration_0.5_yaw | 1.186852 | 1.05 |

**历史门槛未通过，因此未采集/读取任何预封存 fresh 输出，未消耗其测试独立性。候选拒绝。** 这四条新数据已用于校准，今后也不能再称为fresh测试。

## 6. 实验边界与验证证据

- 门控源码与上一轮字节一致；沿用七项行为测试，本轮预检 **7 passed、3 warnings，2.00秒**。没有重复将上一轮1210项回归冒充本轮新执行；本轮未改产品源码。
- 续跑适用范围另有7项定向测试通过，验证额外schema错误、其他序列、fresh或变更请求不能被该例外放行。
- 原正式V2覆盖审计的两项测试本轮重新执行，**2 passed、40 deselected，1.05秒**，确认原拒绝规则仍在。
- 每个实际评估集合完整验证target删除/替换NaN后输出逐位不变；请求后缀变化的前缀预测差按原门槛核验。
- 保存完整源码/父实验指纹、新数据指纹、实际命令、QC、特征缓存、500步校准历史、最终门控权重与原始逐窗口评分。独立审查在启动前确认隔离、比较对象和条件fresh路径。
- 没有根据碰撞、速度覆盖不足或预测失败重新设计序列；没有增加结构或扫描截断阈值。
- 正式导航未启动，planner、Oracle、关节限制和35秒规则未改；不报告新的导航成功率。

证据：[冻结实验规则](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/protocol.json)；[四条新序列](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/collection_manifest.json)；[实际运行](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/resume_collection_summary.json)；[明确披露的校准覆盖修订](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/calibration_coverage_amendment.json)；[原采集失败记录](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/status.json)；[独立原始数据复核](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/independent_raw_review.json)；[原28项验收](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/assessment.json)；[新数据指纹](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/calibration_dataset_sha256.json)；[错误修正诊断](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/postfit_diagnostics.json)；[独立校准研究权重](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/gate.pt)；[独立重算与隔离核验](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/independent_postfit_review.json)；[最终核验](/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1/final_verification.json)。

原pipeline.py保留用于复现首次覆盖检查失败；resume_pipeline.py保留此次实际续跑入口与严格例外。两者均拒绝覆盖已有状态。重新完整复现须使用独立目录，保留原请求与模型/门槛，显式采用已披露的校准适用范围，并记录新的数据指纹；不能直接重复运行来覆盖本次记录。环境保存在runtime/resume_runtime.json。

## 7. 结论与下一项优先工作

本轮完成了真实新增数据和一次固定方案的验证，结果是拒绝，而不是缺少运行。原模型、旧门控和新门控都保留各自原拒绝结论；历史检查通过项从24/28降到16/28并非导航成功率变化。没有证据表明整个世界模型路线不可行，也没有证据把这次退步唯一归因于数据规模、输入不足或线性门控表达。

建议暂停继续叠加可信度模块或扫描阈值，保留未加门控的加速度残差作为研究对照。下一项可直接利用这四条新增运动序列，预先固定一次基础残差网络的数据扩充对照，检查它能否学会这些真实运动响应；旧验证/测试只作历史回归，原封存fresh仍留作最终验证，训练前冻结方案与门槛。该方案是待验证的下一步，不是本轮已证明有效的修复。

面向导航还必须分别证明：对已知请求的预测可靠、对候选请求的选择有用、在线闭环确有收益。当前仍停留在第一层，不能用厘米级总体误差或完成校准代替后两层验证。
