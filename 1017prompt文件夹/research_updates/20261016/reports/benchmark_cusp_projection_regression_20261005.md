# 未完成换向时跳过短反向段：CPU复现与最小修复

2026-10-05，08:44UTC阶段。**已复现并修复一个具体的参考投影缺陷，CPU回归通过；新候选尚未物理运行，导航仍未通过。** 原跟踪器在倒车交接点尚未停稳时，会跳到后面的前进段并取消倒车。独立版本只在“当前参考方向≠已接受方向”时保持该交接点，让原制动/五次低速确认继续处理；方向确认后恢复原投影。没有修改相机、PID、速度、planner、映射、协议、smoother或停稳标准。

工作区 `/home/an/Documents/ChatGPT/球型机器人导航`，H=`experiments/stereo_memory_navigation_20261004_v1`。上一轮原35s候选timeout/目标剩3.671378m、RMS.572390/peak1.105993、0接触及限定改善拒绝全部保留；较强的归档guarded B0也不替换。本阶段没有新仿真/视频/导航成功率、训练或规划搜索。整个双目记忆导航研究仍未完成，automation ACTIVE。

## 1. 复现来源与根因

先核验上一阶段106来源/5670旧输入/44闭合产物/2762关闭后实际输出/5文档SHA，均原样。此前完整读取的mission/mainplan/ledger旧前缀保持，读取新追加和最新报告；截断批次不冒称完成整份新读取。原项目只读，沿用已有隔离研究目录和SHA身份，无无关Git提交/删除。用户授权自主研究，日常可逆实现不重复审批。

依据真实 `root_direction_run` 的frame175已知参考和frame180过去policy状态，核验连续181/182两个当前观测。原 `_project_to_reference` 只从**当前索引的下一点**开始寻找第一个与accepted方向相反的点。第一次投影可进入reverse交接点；下一次当前点已经reverse但accepted仍forward，而下一个点又forward，扫描放行后续forward段。`OraclePathPolicy.act`先投影再做方向资格，desired回到forward就取消pending。长反向段的历史B0能换向，不证明这个短F/R/F模式无缺陷。

唯一假设：**未被接受的当前交接点必须纳入方向边界。** 不是假设机器人应提前反转，也不是调小低速阈值或增加制动速度。实际当前位置到交接点的距离保留，未用候选索引制造评价误差下降。

## 2. 最小独立实现与真实身份

`H/cusp_projection.py` 的 `with_cusp_boundary(base_class)` 只覆写 `_project_to_reference`。原reference与accepted不同：返回当前位置到当前点的真实距离，索引不前进；相同：直接调用原方法。原 `_handle_direction_change`仍使用XY速度连续五次10Hz低于0.05m/s、反弹清零，不加新计数/方向锁存/停车前置条件。非法accepted在此独立投影边界明确拒绝，避免无定义状态变成永久等待。

`make_cusp_oracle`沿用冻结BankMap JSON/with_confident_speed/原Oracle协议、lookahead=.5、drive model、增益；仅在Oracle基类前加投影子类。实际内部链为 `with_confident_speed.<locals>.ConfidentSpeed → make_policy.<locals>.MappedBankOracle → with_cusp_boundary.<locals>.CuspBoundary → OraclePathPolicy`。新外层 `CuspProjectionPolicy → RootDirectionPolicy → GuardedReferencePolicy → DepthLocalCurvePolicy`只在episode构造/reset安装新inner实例，继承原 `act`和root `plan_boundary`。没有action期间替换inner/reset/改变planner。

RootDirectionPolicy是**上一轮被拒绝的诊断比较器**，不是已接受的新基线；本版本保持它的根成本、guard和参考生成，后续只改变投影才有单因素对照。较强guarded B0保留为独立历史比较。本阶段不把root adapter已安装None校验Minor一起修掉；正常轨迹未出现此状态，单独未来版本仍需在重建前拒绝None。投影边界的非法状态检查不能冒称完成了planner入口校验。

合法重规划可以换掉旧反向段；新参考当前方向与accepted相同即委托原逻辑，原handler可取消过时pending。本修复不把旧交接永久锁在已被替换的路径，不增加无法开始换向的循环等待。

## 3. 先失败再修复的真实测试

11个literal fixture使用真实继承tracker、动作协议与映射，不调用planner/simulator。初始可调用实现委托原代码，**7 failed/4 passed，1.25s**，失败正是两个方向短反向段越过、首次交接无法保持、真实act丢失低速计数及两个非法accepted未拒绝。不是missing module/不可调用假失败。唯一修复后**11 passed，1.23s**；全H **195 passed，8.72s**，RED/GREEN/suite日志均保留。

覆盖：F/R/F及R/F/R；首次无pending仍能进入交接点；两次.04低速后.06反弹清零、再四次.04仍制动、第五次才接受并发出反向请求；确认后原投影释放；普通同方向完整act/raw/protocol/diagnostics/mapping与原tracker精确相同；合法参考替换取消过时pending；外层继承方法身份和episode-only初始化；非法None/stopped在投影前拒绝。fixture中的第五次反向请求是CPU控制输出，不是实测机器人反向运动或该段完成。

## 4. 唯一封存两时刻实测输入对照

协议 `cusp_projection_protocol.json` 于08:53:56.762833UTC封存109来源/8487历史与case输入，唯一审计08:53:58.904760—08:53:58.951319UTC结束exit0。只两臂×两次act=4次科学调用，无新grid/Hybrid查询或物理序列；unit tests与这个审计预算分开。

case封装：已知frame175完整参考；frame180实际过去的9个policy资格/投影字段；frame60首次参考reset以后至180的121条mapping记录；frame181/182当前RGB/proprio/action/quaternion history、XY/yaw及goal。只重建**控制实际消费的policy状态**，不是完整物理/PID/smoother快照。保存request与expected diagnostics只用于运行后核对原臂，不是candidate的输入。

| 当前实测输入与控制结果 | 18.1s/frame181 | 18.2s/frame182 |
|---|---:|---:|
| 实际XY速度m/s | .247591 | .231750 |
| 原臂索引/accepted/pending | 1/F/R | 3/F/None |
| 修复臂索引/accepted/pending | 1/F/R | 1/F/R |
| 原臂raw main/aux rad | 0/0 | .5/-.582204 |
| 原臂protocol main/aux | 0/0 | .5/-.523599 |
| 修复臂raw/protocol | 0/0 | 0/0 |
| 修复臂低速计数 | 0 | 0 |
| 修复臂来源 | direction_change_braking | direction_change_braking |

原臂两个请求及完整diagnostics与旧实际日志精确相同。181两臂状态/动作相同，因此182实测观测仍属于共同前缀；**首次当前动作分叉182以后立即结束**，没有用183以后的旧轨迹作候选真标签。182候选仍在真实速度.23175m/s制动，不强制倒车；当前点真实距离.105060m，与原跨段点.096085m分开。修复改变了这次raw与protocol请求；尚未送入物理执行，不能称applied改变/实际停稳更快/实际完成倒车。

![已知参考与共同前缀上的当前决策；没有候选后续真实轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/cusp_projection_decision.png)

## 5. 验证范围和未解决项

作者08:55:18.621803UTC核验109源/8487旧输入SHA、封存先于审计、4调用/原两请求精确/181共同前缀/182索引与层语义/assessment SHA，未追加act/搜索/仿真。独立review、逐条范围裁决及作者行政闭合分开记录，不把后追加报告冒称二次审查。

接受的仅是**projection CPU回归与当前请求接口修复**。物理候选、原35s真goal/contact/独立参考进度/RMS/空间检查点、完整反向段是否完成尚未验证。确认方向后下一次仍沿用原nearest-waypoint规则，可能提前接近下一交接；本修复不认证所有短primitive的实际完成。真实停稳/动力学尾迹、后续合理replan与全局进展影响必须在闭环测量，不能从CPU两帧推断。

RGB深度覆盖/来源可靠性、estimated pose、唯一学习记忆/世界模型增量和正式E0/E1/E2均未准入，所有旧gate和相机位置冻结。已保存代码、case NPZ/JSON、协议/assessment/verification、RED/GREEN/suite/audit日志和决策图。旧真实双目/第三视角视频仅作为旧运行context，未生成候选视频。

## 6. 下一项有界实际工作

独立review通过后，事前封存**唯一CuspProjectionPolicy原35s同起点goldenE1**，对归档root_direction_run作唯一投影改动的因果比较，不重跑旧矩阵；同时报告较强guarded B0，不能仅胜过较差比较器就准入基线。相同6s前缀、rig/depth-knownpose风险域/root成本/guard/mapper/PID/速度/五次反弹规则保持。第一分叉前物理/关节/完整执行与来源历史核验，分叉后查实际applied、真实XY/主轴方向、交接段是否执行，独立冻结参考/goal/contact验收。

导航仍必须真实goal≤.2m、原35s零接触，限定改善不放宽旧横向/进度标准。若只是延长停车、改变索引、或候选仍超时/横向退化，应保留拒绝；只修控制实现缺陷不保证任务成功。不得顺手改root-cost、速度、PID、几何阈值、参考、映射或补状态矩阵。本CPU阶段不追加实际运行，下一heartbeat从已封存代码和review结论接续。

## 独立审查与作者收尾

唯一fresh-context只读审查 `/root/cusp_projection_review`（gpt-6-astra/high）：0 Critical、0 Important、1继承已披露Minor，无新增发现。允许关闭有界CPU阶段，不准入实际导航或替换基线。已审查正文SHA `371ca02bcca7f03dfb6e82b6e4f6b0d85538df0b9470bb1a2123102e846d6122` 保持原字节，以下行政作者append未做第二次审查。

继承Minor：已安装accepted=None可先进入root planner，投影拒绝不能替代重建前校验；正常归档路径未出现。作者按Minor明示保留，未来独立版本先无搜索fixture RED→GREEN，本次唯一投影改动不夹带修复。实际资格后下一投影恢复原规则，没有证明短反向段物理完成。

审查亲自执行11无搜索literal tests（11 passed/1.32s），核验109来源/8487输入SHA、report/protocol/assessment/verification与包身份、case180九字段/60–180共121mapping/175全参考/181与182所有当前输入及原请求，阅读真实MRO/执行调用顺序与图。作者RED/GREEN/195全suite及唯一两臂×两act科学审计的exit/时间/result只检查记录，不冒称审查重跑。完整范围与保存返回要点见cusp_projection_review.json/review_notes.md。

作者接受并逐条记录10项declined及若误判代价：实际applied/停稳/反向、资格后段完成、全35s导航、长期停车replan推进留真实闭环；整体冻结模块不重新认证；None planner入口Minor保留；私有任意篡改通用恢复不新增；RGB/estimatedpose/learning/formal未准入；旧矩阵只核验身份不重跑；freeze后行政append/closure与未来物理协议不冒称独立已审材料。这些范围限制不免除下一阶段实际验证，也不抹去本阶段已证实的投影回归修复。

作者最后只核验来源/历史与case/阶段产物/同步文档SHA、旧前缀/唯一审查/封存顺序和gate，追加mission/mainplan/ledger；未再act/search/sim。原参考、rig、PID/速度/五低速反弹和全部旧失败保持。当前CPU regression PASS、导航未验证/旧REJECT、whole research未完成，automation ACTIVE。下一项独立预封存唯一原35s同初态projection闭环因果对照，同时保留较强guarded B0比较。
