# 来源记忆真正接入在线规划：E1受控对照（2026-10-04）

本轮完成了在线记忆消费者及两条实际同起点仿真。**记忆确实保留了被遮挡的障碍，轨迹振荡减小，但两条仍发生计分内接触，可靠导航候选没有通过。** 这不是完成的RGB导航，也不产生新的正式E0/E1/E2成功率。完整研究仍在进行。

## 1. 接入的具体环节

此前SpatialMemory只有独立原型和测试，实际导航仍调用旧地图。本轮新增ProvenanceGrid，真正被`provenance_policy.InformationPolicy.act`调用并由原风险A*消费标签；不再只做旧日志回放。来源标签分为未知、当前地面、历史地面、当前障碍、历史障碍。

两个arm都关闭旧`seed_body`的自由写入。脚下0.4m圆盘只保留为几何诊断mask，不写地面、不写观测时间、不参与自由认证。两条初始都有8格实际投影地面，但脚下49格仍为未知，脚下观测自由数为0。没有用先验冒充观测。

- current：每帧仅保留当前深度投影的地面/障碍。
- history：地面保留10秒；测得障碍持续保存，不读取未来，也没有预测地面标签。

沿用同一固定相机、照明、mask、完整采集事务、点投影、自体距离0.55m排除、旧几何tracker、roll PID、增益、协议、smoother、supervisor、方向锁存和五次低速确认。两条都使用点表示，没有表面插值；实际A*与0.6m观测障碍清距保持相同。

两条都属于**真深度+真位姿的特权诊断**（位姿sigma=0假设）。仍明确允许未知空间运动，经过未知支持时主轴请求限制为±0.3rad/s；这不是安全证书。RGB模型的拒绝结论保持不变。独立全图只保存到离线标签文件，未插入policy packet。

## 2. 实际对照和身份

运行前冻结`online_provenance_protocol.json`，只运行golden_e1_0001的current/history两条，不扫描地图或控制阈值。原35秒预算、前6秒零请求、物理接触v2终止规则保持不变；因为实际接触，两条在预算用完前终止。

actual class为`provenance_policy.InformationPolicy`，继承冻结旧InformationPolicy的act；显式设置新belief后消费原tracker/latch，使用原风险请求上限。源码、PID配置、正式协议和原始实现SHA均保存于各运行identity。前6秒全部17项非raw物理/请求字段逐项相同，两个raw初始化原点本次也相同。地图来源本身是本轮实验因子，没有宣称两条地图相同。

参考评估独立于策略path_index。本地原先没有E1 common_reference文件，因此按**冻结runtime原始A*实现、0.4m半径+0.1m裕度、同一冻结占据图与起终点**复现其独立评估参考并在运行后归档。长度8.836753m与原manifest完全一致。参考没有根据候选轨迹调整，也不是在线策略最后一次产生的路径；其生成时机和哈希单独记录。

| 实测指标 | current | history |
|---|---:|---:|
| 计分终止时间 | 28.8s | 31.4s |
| policy调用 / capture / 物理样本 | 288 / 289 / 1441 | 314 / 315 / 1571 |
| 非零请求块 | 228 | 245 |
| 沿独立完整参考终态进展 | 1.8649m | 2.3140m |
| 实际目标距离减少 | 2.5499m | 2.5009m |
| XY总路程 | 7.1937m | 3.1431m |
| 终态目标距离 | 4.0501m | 4.0991m |
| 横向RMS / peak | 0.7772 / 1.6583m | 0.7532 / 1.2108m |
| 峰值实际XY速度 | 0.5208m/s | 0.3163m/s |
| 首个计分接触 | 28.78s | 31.36s |
| 结果 | 接触失败 | 接触失败 |

history沿参考进展增加约0.449m，峰值横向偏差减少约0.448m，明显减少current的反复左右摆动。但目标距离减少略小，接触仍发生；**延后2.6秒失败不算导航成功**，总路程较小也不能单独作为成功证据。主轴请求上限并不等于实际XY速度上限。

![真实轨迹与障碍保持](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/figures/online_provenance_comparison.png)

## 3. 记忆解决了什么，验证范围是什么

独立离线占据图是冻结2D物体足迹代理。先逐帧记录曾经由传感器命中、且与该图占据格一致的障碍，再检查其不在当前观测中时是否仍保留。

| 来源检查 | current | history |
|---|---:|---:|
| 遮挡后仍保留的已测占据代理格，采样峰值 | 0 | 30 |
| 遮挡后忘记的已测占据代理格，采样峰值 | 25 | 0 |
| 地面自由标签与占据图重叠格，采样峰值 | 0 | 0 |
| 脚下先验写成观测自由 | 0 | 0 |

history通过了障碍保持的组件验证：观测障碍离开视野后仍影响真实在线规划。全部live快照可从当时及过去传感器packet独立重建，标签/地面时间/障碍时间/来源码逐项精确一致；各帧来源计数总和为160000，非未知标签都有相应测量时间，没有未来时间。

“自由与占据图重叠为0”仅是**采样快照上的2D代理检查**。它不证明完整连续时域、全3D机体扫掠安全，不证明未知格没有障碍，也不把地面可见性等同于所有机体高度可见性。两条碰撞直接说明这些组件通过不能代表导航通过。

## 4. 接触前执行链：停车请求不是已经停稳

history在30.5s最后进入持续零请求，路线状态为`observed_obstacle_blocks_route`。当时真实XY速度约0.1200m/s。正式smoother仍从主轴0.3rad/s按既有斜率下降，**smoothed/applied主轴目标直到31.1s才为0**；不是零请求瞬间立刻归零。

从30.5s发出零请求到31.36s首个真实计分接触，机器人继续行驶约0.10645m，净位移约0.09992m。结束前没有达到连续五个10Hz速度低于0.05m/s的停稳资格。30.5s物理行对应发出新请求之前的旧0.3目标，不能误当作新零块已经执行；后续原始请求为0、实际目标逐步下降均由实际日志核验。

current在接触前仍持续非零请求；其当前地图确实遗忘了部分已测障碍。history保留了障碍但仍接触，因而不能把整条失败都归因于遗忘，也不能只调整smoother就宣称解决。现有A*只检查当前2D标签与静态清距，尚未对真实执行链延迟、停止尾迹、未观测体积和可执行曲率进行统一检验。

这次证据把下一个控制工作具体化为：**在实际局部规划消费端检查制动过程与执行扫掠，区分当前可观测支持和未知探索，并核验请求曲线是否可被现有roll跟踪器执行。** 不能继续只增加地图面积，也不应同时改PID增益、相机、速度、占据范围和planner来掩盖原因。

## 5. 验收结论与实物证据

- 来源/在线接口：通过本次有界验证，真正消费了新记忆；脚下先验没有伪装成观测。
- 障碍保持：在本次冻结2D占据代理和实测快照范围内通过；不是全3D安全证书。
- 可靠导航候选：拒绝，两条均实际接触；不报告新的导航成功率。
- RGB、估计位姿、实测局部运动消费者、学习记忆/世界模型增量：未完成，不由接口通过替代。

新增4个地图测试和1个实际policy消费者测试均先RED后GREEN，隔离研究目录36个CPU测试通过。两条实际运行均退出0，保存完整请求/物理日志、传感器packet、来源快照、源码版本和视频。原阶段报告原字节/哈希及历史结果保留。

视频采用旧runner，current各294帧、29.4s，history各320帧、32.0s；含初始帧及计分后5个物理尾帧，最后五帧不走新采集事务且不参与成功/接触评分。

[current第三人称](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/online_provenance_runs/current/episode_golden_e1_0001/third_person.mp4) · [current双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/online_provenance_runs/current/episode_golden_e1_0001/stereo_view.mp4)

[history第三人称](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/online_provenance_runs/history/episode_golden_e1_0001/third_person.mp4) · [history双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/online_provenance_runs/history/episode_golden_e1_0001/stereo_view.mp4)

证据目录`experiments/stereo_memory_navigation_20261004_v1`：`online_provenance_assessment.json`、`provenance_contact_boundary.json`、`online_provenance_evaluation_reference_identity.json`、`online_provenance_status.json`和各实际运行identity/日志。下次继续一个有界局部执行/制动消费者实验，不重复这两条矩阵，也不把任务转回M2调参或再次拟合双目网络。
