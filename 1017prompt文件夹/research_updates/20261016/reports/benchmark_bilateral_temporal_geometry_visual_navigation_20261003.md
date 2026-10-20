# 双目时间几何视觉导航：实际闭环、渲染边界与拒绝结果

日期：2026-10-03。工作目录：/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1

## 结论与本轮交付

本轮回到视觉导航：实现了一个真正消费**左右 RGB**的局部导航候选，并完成实际 GPU 闭环。候选从两只眼各自的过去图像估计局部空间点，合并障碍记忆，触发并选择左右局部曲线，再由原跟踪器执行。它不读取 Oracle 路线、占据地图或渲染深度来做动作；定位/姿态仍由 benchmark 提供，**不是纯视觉定位，也不是新世界模型网络**。

清除渲染诊断提示后的最终四条对照显示：无视觉 B0 在 E2 14.0 s 到达，候选在 E2 5.4 s、E1 7.5 s 真实碰撞。**拒绝本候选，未获得导航改善。** E1 比 B0 更晚碰撞不算成功；两只眼确实改变请求、接口验证通过，也不代表空间估计或导航通过。

同时修复了实验输入边界：原 runtime 会把 OraclePath、ActualPath、Start、Goal 画入场景。首次隐藏父节点没有生效；本轮通过真实 USD 复现、逐个隐藏实际几何节点，并在每次实际采样前保存可见性查询，确认四种诊断标记不再输入相机。没有移动相机、改灯光、改 PID、改碰撞规则或改 planner。

## 唯一候选与冻结条件

预先冻结的[开发协议](../experiments/bilateral_temporal_geometry_navigation_20261003_v1/PROTOCOL.md)规定只尝试一个估计器/控制消费候选，无参数扫描。旧 M1/M2、残差网络、Oracle 控制均未修改。

| 环节 | 实际实现 |
|---|---|
| 对照 | 正式 `GoalGreedyPolicy` + 既有 roll PID；没有视觉、没有 Oracle 路线跟踪 |
| 候选 | `BilateralTemporalPolicy(GoalGreedyPolicy)`；唯一覆写的 `act` 每块调用几何一次，再调用父类动作映射 |
| 允许输入 | 当前/过去左右 RGB、XY、yaw、四元数、自身状态/动作历史、PointGoal |
| 相机 | 原固定光心 base=(0.10, ±0.30, 0.10) m 与原光轴；未变 |
| 执行 | 原 protocol/smoother/supervisor、roll PID 0.4/0.1/0.7、主副轴限制 |
| 参考生成 | PointGoal 方向上的既有连续模板，左右偏移 2.3 m，单调投影、前瞻 1 m |
| 障碍记忆 | 0.1 m 格、三秒；未见区域保持未知 |
| 裁决 | 原 35 s 到达规则与 physical_contact_v2；未增加“停车就通过”条件 |

两只眼分别使用各自最多三张过去图像，寻找多视图一致的纹理/深度；并非把当前左右画面强行当作共视充分的普通双目。搜索 96 个 optical-Z 假设（0.55–8 m），5×5 NCC 块，至少两个支持视图、平均 NCC≥0.85、每视图≥0.70、远近峰间差≥0.05、视差跨度≥4 px。所有阈值在评分/实际运行前冻结。

正前方 0.55–3.5 m、横向±0.60 m 内至少六个障碍格，连续两次 10 Hz 判定后锁存绕行。左右曲线按与**已经观察到的点**的最小距离择优；这一步没有要求绝对净空合格，是本轮明确暴露的设计缺口。无足够证据时仍进行未认证的名义 PointGoal 跟踪；因此这些是可能碰撞的有界仿真开发实验，不是安全部署策略。

## 渲染提示问题与修复证据

共完成三版各四条实际运行，合计 **12 条完整序列**。只第一组之前有一次 CLI 参数错误，未进入完整仿真；失败记录保留，没有当作运行结果。

| 数据版本 | 渲染边界 | B0 E2 / E1 | 候选 E2 / E1 | 解释 |
|---|---|---|---|---|
| 原四条 | 诊断标记可见 | 成功 14.0 / 碰撞 5.9 s | 碰撞 5.4 / 7.6 s | 保留原拒绝；不能证明无 Oracle 图像提示 |
| clean_render_v1 | 父节点写 invisible，但子几何未隐藏 | 成功 14.0 / 碰撞 5.9 s | 碰撞 5.0 / 7.1 s | **隐藏无效**，目录名字不能作为证据 |
| clean_render_v2 | 实际几何隐藏，每次采样前核验 | 成功 14.0 / 碰撞 5.9 s | 碰撞 5.4 / 7.5 s | 最终清洁输入对照 |

CPU 真实 bundled USD 复现：父 prim 类型为空，`Imageable=False`；即便父属性为 `visibility=invisible`，子 Sphere 的 `ComputeVisibility=inherited`。修复只对真实 Imageable 子节点设置 invisible，更新曲线顶点后仍不可见。新增两个测试通过，保留修复前测试失败记录。

第二版在首次渲染前保存四个节点的实际查询，在每次相机采样前再次断言并保存：

| 完整运行 | 检查采样次数 | Start / Goal / OraclePath / ActualPath |
|---|---:|---|
| B0 E2 | 141 | 全部每次 computed_visibility=invisible |
| 候选 E2 | 55 | 同上 |
| B0 E1 | 60 | 同上 |
| 候选 E1 | 76 | 同上 |

见[实际查询汇总](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/render_boundary_verification.json)、[第二版冻结说明](../experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/RENDER_BOUNDARY_ADDENDUM.md)及[实际前后画面](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/render_boundary_actual_frames.png)。E2 1.5 s 左目旧画面中的诊断目标球在新画面消失；第三人称画面中的其他场景彩色物体是原场景内容，不是本次隐藏的四个诊断节点。

这也限定了历史结果的解释：此前左目触发版本 golden 3/3 的实际通过轨迹仍保留，不能倒改成失败；但它们在同一会渲染提示的 runtime 上运行，当前没有完成该旧控制器的清洁输入复验，不能凭“策略未打开地图文件”就宣称严格排除了 Oracle 图像提示。

## 最终实际导航结果：clean_render_v2

仅 golden E2/E1 各一组 B0/C，对照同一初态。不是 fresh 场景，也不是完整 E0/E1/E2 集的新成功率。

| 序列 | 视觉绕行事件 | 终态时间 | 首次真实接触 | 终态目标距离 | 实际走行 |
|---|---:|---:|---:|---:|---:|
| B0 E2 | 无 | 到达 14.0 s | 无 | 0.139 m | 8.811 m |
| 候选 E2 | 2.1 s，左侧 | 碰撞 5.4 s | 5.32 s | 7.147 m | 1.918 m |
| B0 E1 | 无 | 碰撞 5.9 s | 5.84 s | 4.049 m | 2.600 m |
| 候选 E1 | 2.4 s，左侧 | 碰撞 7.5 s | 7.46 s | 4.300 m | 3.344 m |

四条无 runtime exception，真实接触传感覆盖完整，冻结源码 SHA 均匹配。碰撞不是由离线占据图安全裕度代替真实接触判定。B0 E2 成功是原规则的进入目标区；终态速度约 0.725 m/s，本轮没有额外验证到达后停车保持。

两组初始 XY、姿态、速度、关节、smoother、applied action 差值均为零。E2 首次动作差在第 21 块，前 106 个 50 Hz 物理样本一致；E1 在第 24 块，前 121 样本一致。请求、clipped/smoothed/applied、关节及运动状态逐项差值零。两组 raw 全局仿真时钟各相差一个 warmup tick（−0.02 s/−1 tick），已单独披露；按重置后的相对时钟和动作块对齐，不能宣称原始绝对时钟也相同。见[实际结果与前缀核验](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/navigation_assessment.json)。

![实际闭环轨迹与 RGB 局部曲线](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/actual_visual_navigation_and_local_curves.png)

上图背景地图只供事后评估；在线策略没有读取它。红色为实际候选轨迹，绿虚线为提出的完整曲线，不是已走完的轨迹。下图红点来自当前/过去 RGB 估计；灰区未见，不表示自由。E2 原本可直行，误绕后撞侧墙；E1 有绕行动作，但没有实际通过中央障碍。

## 双目消费与因果接口验证

对实际记录的全部允许输入重建请求，与 GPU 在线请求逐项一致；重放时阻止 `open`/`np.load` 输入，策略本身没有地图/深度/日志读取实现。对前 40 块截断未来，或替换后续 RGB、位置及目标，前缀请求均不变。

| 序列 | 全部在线块重放 | 未来截断/替换 | 置空左目改变请求 | 置空右目改变请求 |
|---|---|---|---:|---:|
| E2 | 54 块完全一致 | 40 块前缀一致 | 19/40 块 | 19/40 块 |
| E1 | 75 块完全一致 | 40 块前缀一致 | 16/40 块 | 16/40 块 |

这证明两只眼有实际决策作用，不再只是 shadow。置空检查使用同一条已测状态序列，只能证明接口消费；没有把重放当作另一策略的实际轨迹或成功率。详情见[接口审计](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/interface_audit.json)。

## 空间估计错误与拒绝原因

### 历史门槛保持不变

旧完整 E0/E1/E2 回归预先要求：每条≥30 个有效场景深度、误差≤0.5 m 比例≥80%、p90≤1 m；E1 4 s 前触发；E2 假中央阻挡帧≤10%。其原评分如下，**拒绝结论不倒改**：

| 历史序列 | 接受的深度点总数 | 有效场景点 | 全部点误差≤0.5 m | p90 | 中央阻挡帧比例 |
|---|---:|---:|---:|---:|---:|
| E0 | 1,863 | 1,214 | 28.34% | 7.853 m | 88.57% |
| E1 | 405 | 231 | 47.16% | 7.853 m | 53.33% |
| E2 | 1,371 | 1,094 | 35.67% | 6.757 m | 84.51% |

这些历史数据包含诊断标记且已用于开发，不是新的 held-out。clean 目录的旧 gate 副本仅是运行前保留的历史状态，不是清洁渲染感知验收。在线试运行是协议预先允许的有界开发探测，不宣称感知先通过。

### 清洁运行的真实 RGB 预测评分

渲染深度只由独立 runtime 保存，事后评分；在线模型没有拿到它。下表统计候选实际走过轨迹上全部接受点，不随机拆相邻帧，不冒充新场景验证。

| 序列 | 左 / 右接受点 | 全部接受点≤0.5 m | 可测非近体场景点 | 这些场景点≤0.5 m | 场景 p90 |
|---|---:|---:|---:|---:|---:|
| E2 | 68 / 112 | 18.89% | 137 | 24.82% | 3.313 m |
| E1 | 127 / 156 | 15.55% | 102 | 43.14% | 1.620 m |

“全部接受点”中真值不在 0.45–10 m 的点保留为不可测/不合格，没有按零误差通过；这导致两个全部点分布的 p90 不可测。非近体场景表只统计可测部分，不能用删掉坏点后的指标宣称总体通过。p90 使用最近秩，并明确记录不可测情况，历史文件未重算。

被预测成障碍高度的点，按真实深度反投影粗分来源：

| 序列 | 近体区 | 地面 | 实际障碍高度表面 | 其他/未知 |
|---|---:|---:|---:|---:|
| E2 | 16 | 27 | 60 | 17 |
| E1 | 106 | 8 | 48 | 2 |

近体区指到真实球心≤0.55 m 的几何代理，**不是实例分割验证的刚体身份**；不能把所有近体点都严格称作机器人自身。表内是重复点观测，不是独立障碍数量。真实机体可见区域、地面及重复纹理被错误解释为静态远处障碍，是本轮清楚的错误模式。详见[记录点评分](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/recorded_depth_score.json)。

触发时 E2 中央 12 格中，5 格有地面来源、8 格有真实障碍高度来源（来源可重叠）；后者真实横向位置约 −0.682 至 +0.773 m，侧壁点被移进中央区域。E1 中央六格有两格近体、两格地面、两格实际障碍高度来源，并非纯净的前墙检测。

更直接的控制缺口是：触发时 E2 左/右曲线与已观察点的最小距离只有 **0.039/0.028 m**；E1 只有 **0.029/0.010 m**。机器人半径 0.4 m；这些估计连自身尺度都不满足，候选仍选择“相对更大”的一侧。因此即使几何准确，当前择优规则也不能证明选择的曲线安全。未知区域未标自由，但仅记录“未知”并没有约束执行，这也是未完成的环节。

相机几何/时间检查显示：optical 旋转矩阵最大元素差约 1.0–1.6e−7；观测相对时间差<1.7e−7 s；采用固定 root 高度 0.4 m 的光心近似最大误差 **0.0461 m**。高度近似不能宣称精确，但没有证据把米级深度错误归结为坐标旋转或时间错位。见[触发点来源、姿态与时间核验](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/online_geometry_audit.json)。

CPU 推理时间中位/p90：E2 64.0/121.1 ms，E1 49.1/83.3 ms。E2 p90 超过 10 Hz 的 100 ms 预算；仿真按动作块推进，不等价于已完成真实实时系统部署。

## 验证分层与后续方向

| 层次 | 结论 |
|---|---|
| 基本几何/控制与渲染回归测试 | 13/13 通过；合成成功不代表真实画面可靠 |
| 实际渲染输入边界 | 第二版通过；前两版不通过或未排除提示 |
| 当前/过去输入、双目消费与因果接口 | 通过 |
| 真实空间量准确性 | 不通过 |
| 已观察净空/未知空间的控制资格 | 不通过；相对择优仍执行明显不足净空的曲线 |
| 实际导航改进 | 不通过；候选两条均碰撞，E2 劣于 B0 |
| 完整正式导航基线 | 未完成；本轮不报告新 E0/E1/E2 全集成功率 |

下一项应针对**视觉空间解释与路线执行资格**，不是继续调 Oracle 或换世界模型网络：先分离已知机体遮挡、地面与场景表面，在独立完整序列上验证近距离障碍/侧壁位置；再要求候选在球体尺度和未知空间约束下确实具备可执行证据，避免两条都不合格仍择一。停在未知区不能作为导航通过，后续仍须证明真实通过障碍并在 35 s 内到达。不能靠调整这两条场景的阈值把当前候选追改成“接受”。

旧单目连续曲线及原 PID 保留为历史开发参考。本轮只修改隔离实验与渲染诊断包装，没有替换正式基线。下一次涉及旧成功策略，也应使用本轮已核验的清洁渲染边界。

## 原始证据与观看入口

- [13 项测试的原始输出](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/verification_full.log)，[渲染测试修复前](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/render_boundary_red.log)与[修复后](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/render_boundary_green.log)；测试环境使用正式项目与 bundled USD 库。
- [四次最终实际运行及命令](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/run_status.json)。每条保留 metrics、states、actions、physics、collision_audit、源码 SHA 和双目/第三人称视频。
- [E1 第三人称视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/candidate_e1/episode_golden_e1_0001/third_person.mp4)、[E1 双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/candidate_e1/episode_golden_e1_0001/stereo_view.mp4)。
- [E2 第三人称视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/candidate_e2/episode_golden_e2_0001/third_person.mp4)、[E2 双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/candidate_e2/episode_golden_e2_0001/stereo_view.mp4)。
- [实际触发时两只眼与接受点](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2/both_eye_images_at_actual_trigger.png)。可看到机体遮挡仍在；黄色圈只是接受的深度候选，不表示真值正确。
- 原四条结果：[原导航审计](../experiments/bilateral_temporal_geometry_navigation_20261003_v1/navigation_assessment.json)；隐藏失败的第一版：[第一版导航审计](/home/an/Documents/ChatGPT/球型机器人导航/experiments/bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v1/navigation_assessment.json)。全部保留，不混入最终清洁输入结论。

## 后续独立清洁复验补充

后续工作已将未改动的旧左目连续视觉控制器在清洁输入下实际重跑 golden E0/E1/E2，均成功，动作及运动状态与原三条逐项一致。因此上文“旧成功策略尚未完成清洁复验”的当时限制，对这三条开发序列已由新证据解除。当前双目时间几何候选的拒绝、hard_0 失败及泛化限制均不改变。详见[后续遮挡与清洁视觉基线报告](benchmark_fixed_rig_self_mask_and_clean_visual_baseline_20261003.md)。
