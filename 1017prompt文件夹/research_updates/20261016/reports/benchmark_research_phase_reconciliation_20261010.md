# 导航基线、学习验证与当前接续（2026-10-10）

本次实际完成12项已封存条件的机器可读汇总，核验55个引用文件SHA；没有重跑导航或拟合。原汇总程序第一次因episodes/conditions字段名称不同退出，旧程序/失败日志保留；v2显式比较相同数值字段，实际退出0。它不是新的导航实验或成功率。

| 方法 | 位姿条件 | E0 | E1 | E2 | 到达条件数 |
|---|---|---|---|---|---|
| 冻结传统双目+显式记忆+roll-PID | 已知位姿 | 超时 | 超时 | 超时 | 0/3 |
| 冻结传统双目+显式记忆+roll-PID | 估计位姿 | 超时 | 超时 | 真实碰撞 | 0/3 |
| 滚动参考末端资格变体 | 已知位姿 | 33.4秒到达 | 超时 | 真实碰撞 | 1/3 |
| 滚动参考末端资格变体 | 估计位姿 | 超时 | 超时 | 超时 | 0/3 |

这些是TaskH：固定相机/光照、平坦地面、安全起步区与受控未知探索，保持35秒规则。已知位姿为特权诊断；估计位姿仍有初始规范与理想低层IMU。不能把两层合并，也不能把它们写成原严格cold-start或纯RGB成功。多次同版E1对照重放不增加独立场景样本数，变体差异不是完整矩阵因果改善证明。

双目观测、来源标签、空间风险记忆、局部规划与roll-PID闭环已经真实实现、运行、封存。当前欠缺的是稳定的估计位姿导航结果及学习增量的完整验证；不是仅停留在采集双目图像。传统基线无需达到完美双目精度，但不能刻意削弱或修改门槛制造新方法领先。

学习记忆风险分类器已完成一个冻结模型的whole-sequence离线验证：E1没有floor样本，不能估计floor误报抑制；E2有65个floor与10417个hazard，floor风险减少84.6%，hazard保留99.97%。整体为unassessed_label_coverage，不是整体模型通过，也不是证明所有世界模型无效。按该实验原预算，没有追加数据/重训/换网权限，且没有触发其条件化在线两臂预算。学习准入限制不能被当成所有传统跟踪修复的前置条件。

## 实际推进的下一控制假设

已新冻结正常forward的纯几何输入候选：保留当前实际lookahead与完整参考，只将该次pure-pursuit几何变换的heading替换为经五个当前/过去10Hz测量资格核验的运动course；root姿态仍供其他环节使用。它与旧仅诊断的course-heading规划可行性不同，与已拒绝的额外yaw-aux消融、横向速度aux反馈和短参考候选不同。

本次已实际实现纯函数与临时instance composition/target钩子，缺模块RED→3项CPU PASS。原lookahead与composition分别只调用一次，finally恢复钩子，不二次调用policy，不改observation yaw，不改PID/停稳/反向规则。低速或不合资格返回原几何。

**尚未完成：**真实工厂/执行域测试、时钟/提前返回/制动与旧字段污染核验、独立审查闭合、真实protocol/applied行为差异及独立事件分叉对照。当前物理/solver/fit预算均为0；这3项CPU测试不构成候选准入或导航结果。独立审查正在进行，下一直接接续这些具体工作，不重复旧curvature、IMU或网络矩阵。

证据：research_phase_reconciliation_assessment_v2.json、research_phase_reconciliation_progress_v1.png、forward_course_geometry_consumer_design_v1.json、forward_course_geometry_v1.py、forward_course_geometry_adapter_v1.py与RED/GREEN日志。图已本人实际查看。原始导航和模型运行仅核查已有证据，本次没有再次采集或训练。整体研究未完成。


## 实际CPU接口续接（新增，2026-10-10）

已补上真实当前工厂、正式PolicyObservation、五次连续测量/单次Oracle act/lookahead/composition测试；正式请求与原PID/smoother/supervisor五tick CPU结果真实改变。原请求副轴−.01352rad，候选+.52360rad，第一tick CPU applied分别−.01047/+ .01047rad。这是synthetic空risk、固定零反馈的执行接口证据，不是实际运动或导航改善。

真实时钟测试发现原adapter在有效但跳到100秒的packet以及不连续history上先读取载荷；保存RED后v2在读取观测/目标前按原10Hz检查clock，异常终端拒绝，不复用旧geometry。另planned-zero nominal waypoint仍改了composition：保存RED，v3使该资格回退，不改请求规则。注意nominal zero waypoint可被原有效main回溯到非零；日志回退不代表实际零请求。

真实factory覆盖normal、low-speed、negative-main coast、short history、goal、no plan、真实_failure_subreason、reverse/pending、invalid/stale/nonfinite/future/gapped时钟、原composition异常恢复、reset及同次新参考重建。最终16PASS，独立审查v3 C0/I0/M0；旧v1 Important仅在CPU接口层关闭。参考重建测试经真实owner安装新斜线参考后，当次lookahead正确消费新参考，未复用旧target。

已准备同版35秒known E1 driver/durable stage，并事前写出事件、真实进度/碰撞/到达/时延及分层范围。新的scorer尚为draft，只有AST语法检查；它还需解决composition时刻状态与事后braking状态区别、独立root/target时序核验、事件/前缀/接口分类与测试。尚无完整identity envelope、完整prelaunch独立审查或物理预算；没有启动新的仿真或拟合。接下来直接完善这些具体执行入口，不能把16PASS写成导航通过。


2026-10-10 接续：新前进course几何消费者仍未完成导航验证。本轮新增composition前状态与root/target快照，真实工厂及评分器合并30项CPU测试通过。独立审查实际发现并保留两项评分来源漏洞（共同污染目标、主轴历史自证），已逐项补入当前参考/lookahead重建、同刻原始物理主轴与XY核验、原始几何parent校验、同次local_plans时钟/版本/waypoints绑定。新版完整身份2210源码/123输入冻结，独立复核进行。所有这些均为接口与评分准备，没有新物理成功率，原传统known0/3/native0/3及rolling known1/3/native0/3保持不变。下一步复核闭合后执行唯一原35秒knownE1双臂；总体研究未完成。


2026-10-10 实际新双臂已闭合：69介入/首分叉70/prefix精确；候选原35timeout净.928078<B0.975779，拒绝/no retry，详细报告 /home/an/Documents/ChatGPT/球型机器人导航/reports/benchmark_forward_course_geometry_e1_20261010.md。个人轨迹图已查看；独立rawclosure C0I0M0。下一记忆再观测标签可用性已实际1947图扫描/原seal绑定，map自由0，不拿未知或先验训练。总体未完成。
