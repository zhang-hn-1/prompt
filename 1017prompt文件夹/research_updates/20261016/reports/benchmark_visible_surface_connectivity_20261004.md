# 当前相机视角下的局部地图起步连通性：连续地面补全与闭环对照

日期：2026-10-04。实验版本：visible_surface_connectivity_20261004_v1。

## 结果与范围

已完成四个起点 × 两种地图表示的 **8 条全新闭环仿真**，每条执行原始 35 秒规则。唯一候选是把已有连续地面观测中的离散点连接成受限地面小片；控制器、相机位置和安全半径保持一致。

**候选增加了部分观测自由格，但没有形成安全起步通路，也没有改变实际执行行为。因此拒绝把它作为导航改善。** 这次没有跑通 E0/E2，也没有产生可接受的新视觉导航基线。

本阶段交付了另一项明确证据：在这四个起点、当前视角、当前“未知阻断 + 0.6m安全半径”的地图规则下，观测内补洞和增加像素密度都无法消除起步阻断。它限制的是此表示与决策规则组合；不能据此宣布所有视觉导航方法不可能。

输入是当前双目**渲染真实深度**、真值位姿和历史地图，属于感知几何上界诊断。它不等于 RGB 导航或世界模型成果。之前 DA3Metric/UniDepth 两候选的拒绝结论保留；本轮没有重新拟合、训练或执行 RGB 候选。四个起点是已经使用过的开发样例，不是新的 held-out。

## 任务承接与固定条件

用户确认的任务是：查清可见地图无法起步的来源，只改一处有证据支持的地图环节，运行 E0/E2 和两个困难起点，保留碰撞与真实进度验收。实验源码与输出都写入独立文件夹，历史报告、成功 Oracle 和历史实验保持冻结。

- 当前相机相对 base_link 球心：X=+0.20m、Y=±0.30m、Z=+0.05m；方向、内参、640×480 分辨率和本体遮挡掩膜相同。
- 地图：40×40m、0.10m网格；未知、已知自由、占据分别编码 -1/0/1。仅机体当前半径0.4m占地可播种自由区，已有障碍不能被播种或地面覆盖。
- 规划：机器人半径0.4m加0.2m裕度，共要求0.6m净空；每0.5s重规划，10Hz请求，一米几何目标。方向反转仍要求连续五次速度低于0.05m/s，反弹清零。
- 执行：原协议/smoother/supervisor、roll PID kp=0.4/ki=0.1/kd=0.7、physical_contact_v2、35s、0.2m目标半径。
- 四个起点：golden E0/E2、hard_0 E1 0003/0004。每条重新完整初始化，序列内部不重置物理状态或控制器。

本轮没有改变导航增益、速度策略、动作范围或安全距离。历史全地图组 E0/E2 到达、0003/0004碰撞作为既有执行背景保留；本轮八条新运行均为“当前可见深度”，没有重新宣称全地图组为新结果。

## 来源排查：空洞与真正未观察区域

原表示把有效地面落点格标为自由，把机体高度范围命中标为占据；未知阻断。其缺点是地面像素稀疏时会留下小的地图孔洞。但是，初始原始分辨率复算证明，这不是四个起点起步阻断的充分解释。

相同首帧、相同掩膜和距离/高度过滤的离线复算如下。全部原始像素只用于诊断，未作为另一个在线控制候选。

| 起点 | 原4像素步长落点自由格 | 全像素落点自由格 | 唯一连续地面候选自由格 | 三种表示的起点净空 | 三种表示可到达安全区的最大半径 |
|---|---:|---:|---:|---:|---:|
| E0 | 733 | 1984 | 821 | 0.412m | 0.0m |
| E2 | 229 | 554 | 265 | 0.412m | 0.0m |
| hard_0 E1 0003 | 523 | 1214 | 614 | 0.412m | 0.0m |
| hard_0 E1 0004 | 398 | 943 | 423 | 0.412m | 0.0m |

三种表示在每个首帧都只能保留起点一个安全连通格；没有达到规划器要求的0.3m最小有效位移。起点0.412m净空低于0.6m要求，提升远处自由格数量没有连接起点。

在机体前向坐标、左右±0.6m带内，最邻近有效观测地面落点在前方约 **1.038m**。四个起点的前方0.5m证据格，在候选地图中仍为未知。这个数是四个初始姿态下的测量，不是整个运行过程固定不变的视野界限。

[初始地图与安全区对照](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/figures/initial_map_support.png)

[自由格数量与起点净空](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/figures/support_counts.png)

另对前方0.4–1.1m、左右±0.6m做了当前相机可见性诊断：每个高度104个空间探针。低处世界高度0.05m的探针，在四个首帧中都没有双目视线证据：91个点不在任一眼图像范围内，剩余13个点虽进入至少一眼图像范围，但全部被本体遮挡掩膜剔除。中部高度0.4m只有约24%–31%有视线证据。不能把中部射线穿过的格直接当作地面以及整段碰撞体积已观察为空。

[近处不同高度的视线证据](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/figures/near_field_visibility.png)

这份可见性检查只评估当前图像、当前深度、遮挡掩膜和相机坐标，不读取全图去决定动作。视线探针比例也不是整格通行证书。初始盲区可能同时涉及视场、机体遮挡和选择的有效深度范围，本报告没有将全部缺失归因于某一个光照或遮挡原因。

## 唯一候选与安全边界

候选只在同一当前帧内，将采样图像网格相邻的地面命中构成三角形并栅格化。

1. 三角形三个顶点必须有限、根部距离≥0.55m，世界地面高度在[-0.05,0.05]m。
2. 三条三维边长度均≤0.25m；仅标记三角形内部格中心，不外推向相机、机器人或未知区域。
3. 更细的原始像素也必须支持该小片：每个闭合5×5原始像素块全部有效、无遮挡、深度0.45–8m、满足地面和根部距离过滤。任一原始内部像素无效、被遮挡或命中障碍，都拒绝该片。实现保守地移除触碰未通过小片的顶点。
4. 障碍优先级保持不变。自由小片不能覆盖已有障碍；累积地图只使用当前与过去帧。

它继承原基线的平面地面/2.5D占据表示假设，连接原始观测内连续地面，**不是完整三维碰撞体积认证**。它没有填补相机前方的未观察地面，也没有将高处射线或全图真值用于补空。

审查在候选执行前发现：仅检查4像素步长三顶点，无法保证中间原始像素无遮挡。四个已保存首帧未发现实际违例，但代码缺少保证，所以补上原始25像素检查。规则在首条候选运行中已经执行，未从候选结果调参。

来源记录保留了一个时间细节：严格规则说明文件在首条候选运行开始后落盘，不能声称全部材料都在首条仿真前完成预注册。该条运行还加载了删除无用导入前的辅助模块；两版函数AST完全相同，原字节和实际SHA均已保存。控制源、落点更新、参数和模型没有因此变化。原冻结协议不倒改，实际执行身份以逐条记录为准。

## 八条实际闭环 A/B

每条均重新运行 Isaac 物理与渲染，而不是旧日志回放。两组同一起点的初始位置、姿态、本体历史、动作历史和目标逐元素一致。评估参考使用上轮已冻结的初始全图参考，仅在离线计算实际几何进度和横向误差，两组可见策略均没有在线读取该参考或全图。

| 起点 | 地图表示 | 终止 | 时间/s | 主轴非零请求块 | 实际累计路程/m | 真实参考进度增量/m | 终点离目标/m | 横向RMS/峰值/m | 物理接触 |
|---|---|---|---:|---:|---:|---:|---:|---:|---|
| E0 | 落点基线 | timeout | 35.00 | 0 | 0.617 | 0.495 | 8.296 | 0.117/0.225 | 无 |
| E0 | 连续地面 | timeout | 35.00 | 0 | 0.617 | 0.495 | 8.296 | 0.117/0.225 | 无 |
| E2 | 落点基线 | timeout | 35.00 | 0 | 0.618 | 0.544 | 8.356 | 0.008/0.037 | 无 |
| E2 | 连续地面 | timeout | 35.00 | 0 | 0.618 | 0.544 | 8.356 | 0.008/0.037 | 无 |
| hard_0 E1 0003 | 落点基线 | timeout | 35.00 | 0 | 0.617 | 0.470 | 7.445 | 0.146/0.274 | 无 |
| hard_0 E1 0003 | 连续地面 | timeout | 35.00 | 0 | 0.617 | 0.470 | 7.445 | 0.146/0.274 | 无 |
| hard_0 E1 0004 | 落点基线 | timeout | 35.00 | 0 | 0.617 | 0.000 | 8.654 | 0.289/0.542 | 无 |
| hard_0 E1 0004 | 连续地面 | timeout | 35.00 | 0 | 0.617 | 0.000 | 8.654 | 0.289/0.542 | 无 |

零请求时仍有初始化后约0.62m的实际运动，不能把它写成“完全停住”或主动导航进展。没有碰撞也不能证明候选更安全，因为两组都没有启动有效前进。两种地图表示的实际raw请求、applied指令和XY轨迹逐元素相同；因此本候选对地图有影响，对执行行为等价，导航验收拒绝。

[八条真实轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/figures/trajectories.png)

[连续帧累计地图面积](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/figures/temporal_map_support.png)

| 起点 | 落点基线初始/最终自由格 | 连续地面初始/最终自由格 | 请求、执行与轨迹逐元素相同 |
|---|---:|---:|---|
| E0 | 733/1162 | 821/1165 | True |
| E2 | 229/379 | 265/394 | True |
| hard_0 E1 0003 | 523/653 | 614/654 | True |
| hard_0 E1 0004 | 398/767 | 423/767 | True |

每秒保存的地图快照记录了净空与连通区；该时间采样用于地图诊断，完整350个10Hz动作记录用于判断是否实际启动。每条351次相机采集均验证当前时间、渲染不推进物理和诊断标记隐藏。

## 解决了什么、仍缺什么

本阶段排除了“增加像素密度就能在首帧解除起步阻断”的解释，并通过完整闭环否定了本次连续地面补全候选能恢复起步的假设。两者都增加地图信息，但在所测范围没有连接起步安全区。全像素表示只做了首帧复算，没有做其长期在线闭环，不能把这项结论扩大到其所有未来历史。也验证了当前帧补全实现不会跨过原始遮挡/无效像素，物理对照没有偷偷改变安全规则或控制器。

**仍缺少的是能够合法取得近处起步区域信息的机制，以及消费这类有限观测的决策定义。** 当前严格规则要求周围0.6m证据，但只承认0.4m机体占地，现视角又没有覆盖中间所需区域。一个仅积累当前帧的地图，在本次自然晃动中没有解除这项冲突。仅凭这四条失败不能证明主动观察无效。

下一项控制工作应直接检验起步观测条件：在已确认安全的开放平面，测试一个小幅机体姿态观测动作是否能获得新的近地面证据，同时测真实位移、晃动和停止余量；再确定可移植的起步接口。若实际硬件不能获得这种观测，就必须明确选择真实测得的初始安全区域先验，或允许并评估风险的探索规则。先验、未知和观测自由必须分别标记，不能把先验或预测伪装成当前图像看见的空地。

本轮没有追加这些改变动作的实验，避免把地图候选失败后的第二个控制方案混进同一A/B。也没有为了产生成功率缩小安全半径、填盲区、延长35s或重训深度模型。困难场景的跟踪/制动问题仍沿用上一阶段证据；本轮可见组未启动，不能用它来验证新的困难场景跟踪改进。

## 测试、证据与复现入口

完整独立实验测试集 **40项通过**：保留上轮26项，新增7项小片安全边界、2项策略消费与正常可观测地面正对照、5项原始像素内部支持。7项小片测试和5项原始像素检查均保存了实现前缺失功能失败与实现后通过记录。独立审查复跑40项，并检查随机三角形栅格化与原始遮挡模式。

每条完整350动作块、351相机对，共2808相机对；八条重新初始化闭环，16段视频保存并解码核验首末帧。原落点基线相对上轮的完整请求、applied和XY轨迹也逐元素复现。CPU正对照中，提供真正连续且足够大的可观测地面时，策略可生成到目标路线与非零驱动请求，排除了候选入口永远停机的错误。

证据入口：

- [冻结原始范围/门槛](../experiments/visible_surface_connectivity_20261004_v1/stage_protocol.json)
- [严格原始像素规则补充](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/native_support_amendment.json)
- [执行版本与时间说明](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/provenance_clarification.json)
- [首帧三种表示/可见性数据](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/initial_support_audit.json)
- [八条仿真实际命令与结果](../experiments/visible_surface_connectivity_20261004_v1/run_assessment.json)
- [运行状态与启动参数](../experiments/visible_surface_connectivity_20261004_v1/run_status.json)
- [完整测试输出](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/cpu_tests.txt)
- [小片测试失败记录](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/tdd_red.txt)
- [原始像素检查失败记录](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/native_tdd_red.txt)
- [最终验证记录](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/final_verification.json)

实际视频：

| 起点 | 落点基线第三视角 | 连续地面第三视角 | 连续地面双目 |
|---|---|---|---|
| E0 | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/points/e0/episode_golden_e0_0001/third_person.mp4) | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/e0/episode_golden_e0_0001/third_person.mp4) | [双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/e0/episode_golden_e0_0001/stereo_view.mp4) |
| E2 | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/points/e2/episode_golden_e2_0001/third_person.mp4) | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/e2/episode_golden_e2_0001/third_person.mp4) | [双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/e2/episode_golden_e2_0001/stereo_view.mp4) |
| hard_0 E1 0003 | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/points/h3/episode_hard_0_e1_0003/third_person.mp4) | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/h3/episode_hard_0_e1_0003/third_person.mp4) | [双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/h3/episode_hard_0_e1_0003/stereo_view.mp4) |
| hard_0 E1 0004 | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/points/h4/episode_hard_0_e1_0004/third_person.mp4) | [视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/h4/episode_hard_0_e1_0004/third_person.mp4) | [双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visible_surface_connectivity_20261004_v1/runs/surface/h4/episode_hard_0_e1_0004/stereo_view.mp4) |
