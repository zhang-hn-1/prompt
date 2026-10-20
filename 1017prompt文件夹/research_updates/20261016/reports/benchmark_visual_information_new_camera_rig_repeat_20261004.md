# 新相机位置下的视觉信息与米制深度复测（2026-10-04）

已按前方 20 cm、上方 5 cm 的新安装位置完成六条实测采集序列、66 对双目图像的两模型复测，以及八条完整闭环地图／可见深度导航对照。当前开发对照中，完整地图到达 2/4，可见真值深度到达 0/4。两种深度模型均未通过冻结精度门槛。

本次没有形成新的可接受 RGB 导航方法。相机坐标修正已落实，但导航、感知、模型验证分别判定，不能把安装正确视为导航问题已经解决。原成功 Oracle 的 golden 3/3、hard_0 5/7 历史结果保持独立。

## 复测范围与新旧数据身份

| 项目 | 上一阶段 | 本次 |
|---|---|
| base_link 中相机位置 / m | (0.10, ±0.30, 0.10) | (0.20, ±0.30, 0.05) |
| 相机方向、内参、分辨率 | 原安装方向，640×480 | 保持原方向与内参，640×480 |
| 固定 base_link 掩膜覆盖 | 左 38.28%、右 38.27% | 左右各 46.24% |
| 感知模型与权重 | DA3Metric Large、UniDepthV2 ViT L | 同一官方冻结配置、同一权重 |
| roll PID | kp=.4、ki=.1、kd=.7 | 保持原值 |
| 执行链与终止规则 | 原协议/smoother/supervisor、physical_contact_v2、35 s | 保持原值 |

为隔离安装位置变化，六条采集在实际 Isaac 物理中重放先前保存的完整请求序列，仅在序列开头重置，序列内不传送物理状态、不重置 smoother。重新拍摄完全相同的 66 个采样时刻，不复用旧 RGB 或旧深度。每条序列末尾继续执行至旧请求序列完成；模型只读取指定的 66 对当前 RGB 与已知内参。

六条序列的采样时刻中，新旧位置、yaw、姿态历史、本体历史、动作历史逐元素最大差均为零。66/66 对图像内容均发生变化，测得相机外参平移最大误差约 1.61 微米。这保证了本次是相同运动状态下的新相机观测对照。

这些场景与请求已被看过，属于重新采集的开发数据，不是全新 held-out；像素和相邻帧相关。重放采集只用于感知测量，不计作闭环导航成功。后续八条是从头重置、重新在线决策的闭环运行。旧实验数据、模型输出和拒绝结论未追改。

原始视野随相机移动改变，新旧有效障碍点分母因此不同。本次使用新安装位置对应的真实 CAD mask，不能复用旧 mask，也不能把新旧比例直接当成完全相同像素上的改善。掩膜覆盖固定 base_link 部件，不代表所有可动机体遮挡均已建模。

冻结协议：[感知协议](../experiments/visual_information_new_rig_20261004_v2/PROTOCOL.md)、[同控制器对照](../experiments/visual_information_new_rig_20261004_v2/CONTROL_DIAGNOSTIC.md)；[采集命令及固定采样清单](../experiments/visual_information_new_rig_20261004_v2/capture_protocol.json)。

## 两种深度模型的实际结果

保持原评分口径：真值光轴 z 深度 0.5–3 m；排除 base_link 遮挡；距根中心至少 0.55 m；障碍高度 0.05–1.5 m。点三维误差 = 光轴深度绝对误差 × 像素光线范数。无效输出保留在分母，当前所选障碍点的无效预测数均为零。

冻结接受门槛：每个关键 E1 子组至少 100 点，至少 90% 点误差 ≤0.30 m，且 p90 ≤0.30 m。关键组为 E1、0003、0004、直接 E1。没有拟合真值尺度、筛掉难点、训练或换配置。

| 序列 | 障碍点数：旧 → 新 | DA3 误差≤0.30 m：旧 → 新 | DA3 p90/m：旧 → 新 | UniDepth 误差≤0.30 m：旧 → 新 | UniDepth p90/m：旧 → 新 |
|---|---:|---:|---:|---:|---:|
| E0 | 4842 → 3304 | 5.0% → 0.0% | 1.025 → 2.488 | 76.7% → 0.0% | 1.549 → 3.136 |
| E1 | 74639 → 54479 | 2.8% → 0.0% | 1.725 → 1.803 | 14.3% → 14.9% | 2.600 → 2.334 |
| E2 | 87994 → 54320 | 42.5% → 40.0% | 1.037 → 0.845 | 33.8% → 4.7% | 0.941 → 3.089 |
| E1 0003 | 13448 → 705 | 50.4% → 10.2% | 0.724 → 1.287 | 11.3% → 36.3% | 2.802 → 1.533 |
| E1 0004 | 31918 → 26556 | 21.7% → 24.0% | 0.924 → 1.473 | 29.5% → 35.2% | 1.075 → 1.816 |
| 直接 E1 | 71096 → 54302 | 29.0% → 56.6% | 1.369 → 0.742 | 0.0% → 22.6% | 2.250 → 1.236 |

![新旧安装深度比较](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/figures/old_new_depth_comparison.png)

例如，DA3 的直接 E1 合格点比例提高，但仍未达到 90%；0003 的新障碍点数大幅下降，表明当前采样条件下其可见评分支持显著变少。不能将一个序列的改善覆盖其他关键组的失败。

| 模型 | 新精度 gate | 双目一对 p50/s | p95/s | 首对含预热/s | 10 Hz p95 预算 |
|---|---|---:|---:|---:|---|
| DA3Metric Large | 拒绝 | 0.181 | 0.183 | 0.773 | 未通过 |
| UniDepthV2 ViT L | 拒绝 | 0.215 | 0.219 | 0.672 | 未通过 |

两模型各严格加载同一官方 checkpoint，在 RTX 3060 上串行处理左右眼。两模型推理时没有运行 Isaac，没有用真值深度、未来帧或真实地图作为网络输入。软件逆时针旋转 90° 转正，同时变换内参，预测再转回原始像素坐标。没有联合双目匹配或视觉定位训练。confidence 仅作原有探索性排序分析，没有变成风险概率或改变 gate 分母。

[完整误差与 gate 数据](../experiments/visual_information_new_rig_20261004_v2/depth_assessment.json)；[confidence 探索性结果](../experiments/visual_information_new_rig_20261004_v2/confidence_assessment.json)。

![新位置实际RGB与模型深度](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/figures/depth_views_h4.png)

## 八条重新闭环运行

保持完全相同的几何诊断控制器、路线生成逻辑、速度策略和 PID。O-map 初始接收完整地图；O-visible 只使用当前左右相机渲染深度、当前位姿/标定与从 t=0 累积的地图，不读取未来、视野外地图或 Oracle 参考。未知区域阻塞，机体占地只初始化 0.40 m，净空要求仍为 0.60 m。

这两组使用真值位姿；O-visible 还使用渲染真值深度，因此是感知上界诊断，不是已完成的纯 RGB 导航。共享控制器仍是上一阶段的独立诊断控制器，不能替代已成功的 Oracle。

| 起点／条件 | 旧 → 新终止 | 新时间/s | 新实际轨迹长度/m | 新终态目标距离/m | 新非零主轴请求帧 |
|---|---|---:|---:|---:|---:|
| E0／完整地图 | 到达 → 到达 | 14.00 | 8.838 | 0.151 | 140 |
| E0／可见深度 | 超时 → 超时 | 35.00 | 0.617 | 8.296 | 0 |
| E2／完整地图 | 到达 → 到达 | 14.00 | 8.811 | 0.138 | 140 |
| E2／可见深度 | 超时 → 超时 | 35.00 | 0.618 | 8.356 | 0 |
| H3／完整地图 | 物理碰撞 → 物理碰撞 | 9.50 | 5.307 | 3.863 | 95 |
| H3／可见深度 | 超时 → 超时 | 35.00 | 0.617 | 7.445 | 0 |
| H4／完整地图 | 物理碰撞 → 物理碰撞 | 16.80 | 8.528 | 5.693 | 160 |
| H4／可见深度 | 超时 → 超时 | 35.00 | 0.617 | 8.654 | 0 |

![新位置实际导航轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/figures/trajectories.png)

以上仅为四个固定开发起点的诊断对照，未启动全套正式场景，也不报告新的总体导航成功率。到达沿用原目标半径 0.20 m 的资格，没有新增到达后停稳规则。

## 可见地图启动与控制失败分开看

首米名义路线的 0.60 m 管道覆盖如下；它用于解释地图连通性，不是速度相关制动认证。

| 起点 | 旧已知空闲比例 | 新已知空闲比例 | 新占据格 | 新未知格 | 新起点空闲净空/m |
|---|---:|---:|---:|---:|---:|
| E0 | 33.5% | 24.3% | 0 | 174 | 0.412 |
| E2 | 36.4% | 24.7% | 0 | 174 | 0.412 |
| H3 | 34.9% | 24.1% | 0 | 176 | 0.412 |
| H4 | 20.6% | 20.6% | 0 | 189 | 0.412 |

![首帧实际可见地图](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/figures/initial_observability.png)

四条可见组仍没有非零主轴请求，均从初始阶段缺少已知可通行路线；物理上仍累计移动约 0.617–0.618 m，不能写成完全静止。即使深度准确，该保守表示仍未打通机体占地与可见地面之间的可通行连接。
完整地图 H3：终止为物理碰撞，首次物理接触 9.46 s，横向峰值误差 0.199 m，副轴请求达到协议边界的帧比例 0.0%。
完整地图 H4：终止为物理碰撞，首次物理接触 16.80 s，横向峰值误差 0.537 m，副轴请求达到协议边界的帧比例 60.7%。

H4 末态原请求为 [0.0, 0.0]，实际 applied 为 [-0.9694837331771851, -0.1790803223848343]，真实 XY 速度 0.349 m/s。请求置零不能当作已经物理停稳。

相机修正改变观测，没有同时修复几何跟踪与停止尾部。这些困难全地图失败不能归因于视觉网络，也不表示原成功 Oracle 的历史表现退步。

## 实际验证证据与下一步

26 项 CPU 测试通过：保留原 22 项测量/地图/控制器测试，新增 4 项新安装、旧安装拒绝、图像时间/尺寸和掩膜身份检查。六条采集及八条在线运行均检查当前图像时间、物理时钟和额外渲染状态不变；诊断标记保持不可见。两种在线模式的输入差异限于允许的环境信息通道，四对初始本体状态逐元素一致，新旧在线初始状态也逐元素一致。

实际接触继续读取 physical_contact_v2；保留四层请求／执行信息、50 Hz 物理反馈、每条运行第三人称和双目视频。推理结果先落盘再评分。新旧比较、源文件和 checkpoint SHA256、输入 RGB/真值 SHA256、相机外参及视频首尾解码另有验证记录。

[完整新旧结果比较](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/old_new_comparison.json)；[最终实测与身份核验](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/final_verification.json)；[实际命令与退出状态](../experiments/visual_information_new_rig_20261004_v2/stage_status.json)；[测试结果](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/final_tests.txt)。

下一步应在这套已核验的新安装位置上处理两个具体环节：先让同一个局部执行器在完整地图下通过困难起点，动作资格同时覆盖转弯执行与制动尾部；再解决可见地图启动连通性，明确哪些空闲来自实际测量、哪些来自可验证的几何假设。保留未知区域语义，不能通过停车、盲倒车或把未知直接当空闲获得分数。

两种冻结感知候选目前不接入 RGB 闭环。没有重训世界模型、拟合新动力学网络、扫 PID 或改变安全裕度；这些不属于本次安装位置复测。需要新的结构改进时，应先在对应失败层设计单独对照。

代表性新视频：

- [E2 完整地图轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/map/e2/episode_golden_e2_0001/third_person.mp4)；[对应双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/map/e2/episode_golden_e2_0001/stereo_view.mp4)
- [H3 完整地图轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/map/h3/episode_hard_0_e1_0003/third_person.mp4)；[对应双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/map/h3/episode_hard_0_e1_0003/stereo_view.mp4)
- [H4 完整地图轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/map/h4/episode_hard_0_e1_0004/third_person.mp4)；[对应双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/map/h4/episode_hard_0_e1_0004/stereo_view.mp4)
- [H4 可见深度轨迹](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/visible/h4/episode_hard_0_e1_0004/third_person.mp4)；[对应双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/runs/visible/h4/episode_hard_0_e1_0004/stereo_view.mp4)

![相同起始状态的新旧相机画面](/home/an/Documents/ChatGPT/球型机器人导航/experiments/visual_information_new_rig_20261004_v2/figures/old_new_camera_views_h4.png)
