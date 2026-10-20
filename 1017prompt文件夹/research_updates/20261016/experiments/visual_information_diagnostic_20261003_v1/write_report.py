"""Generate the deliverable from completed measurements, never planned scores."""
from pathlib import Path
import json
H=Path(__file__).resolve().parent;W=H.parents[1]
D=json.loads((H/'depth_assessment.json').read_text());A=json.loads((H/'run_assessment.json').read_text());C=json.loads((H/'confidence_assessment.json').read_text())
assert all(r['sample_pairs_completed']==66 for r in D.values()) and len(A['runs'])==8
assert not any(r['accepted'] for r in D.values()),'report decision must be revised if a model passed'
def link(p,label):return f'[{label}]({p})'
parts=[f'''# 固定视野导航第一阶段实验结果

本阶段已经完成：在同一份 66 对同步双目图像上评估 DA3Metric Large 和 UniDepthV2，再完成 8 条同控制器实际仿真。两个深度模型均未通过冻结的近障精度门槛；完整地图组完成 E0/E2，却在两个困难起点发生碰撞；可见真值深度组四条均超时。没有形成可接受的新 RGB 导航基线，也没有把停车算作导航改善。

实验开始于 2026 年 10 月 3 日，完成于 10 月 4 日。目录与协议保留开始日期。原有成功 Oracle 的 golden 3/3、hard_0 5/7 历史结论保持不变，详见{link(W/'reports/benchmark_oracle_measured_time_hybrid_navigation_20260929.md','原 Oracle 报告')}；本次独立几何控制器不替换它。

## 执行范围和保持不变的条件

遵循批准计划的第一阶段。两种感知前端各一个固定配置，不训练、不按真值拟合尺度、不继续搜模型。原计划最多 4 个起点乘 3 个信息条件；RGB 深度候选因精度 gate 失败被挡在接入前，所以实际完成 8 条，剩余 4 条 V-depth 未运行。E-C 制动补测、E-D 24 条视觉记忆消融、X-NavDP 适配和世界模型训练没有启动。这是按失败门槛结束阶段，不能把这些后续工作写成已完成。

相机保持当前实际 rig：base_link 中左右位置 (0.10, ±0.30, 0.10) m；640×480，焦距 584.889 px。左右眼各自处理当前 RGB；两个网络属于单目米制深度前端，本轮没有实现联合双目匹配网络。图像固定逆时针转 90 度使画面转正，同时转换内参；相机实体、朝向和照明均未改变。roll PID 保持 kp=0.4、ki=0.1、kd=0.7，主轴范围 ±2 rad/s，副轴协议范围 ±30°，正式 smoother 和 supervisor 不变。

所有实际运行使用 physical_contact_v2、目标半径 0.20 m 和原始 35 秒规则。到达资格沿用进入目标范围的原规则，并不新增“到达后已经停稳”的成功资格。

这条视觉研究线仍允许真实 XY、yaw、当前姿态、本体反馈和 PointGoal；不是已经完成视觉定位的纯 RGB 导航。O-visible 在线允许渲染真值深度，是感知上界诊断，也不能当成最终视觉方法。

冻结文件：{link(H/'PROTOCOL.md','感知协议')}、{link(H/'CONTROL_DIAGNOSTIC.md','同控制器对照协议')}。

## 离线深度结果

复用六条已看过的完整历史开发序列：golden E0/E1/E2、hard_0 E1 0003/0004、直接跟踪 E1。每十个动作帧取一对，并保留各序列最后一个动作帧，共 66 对、132 张图。两模型使用完全相同的输入清单。这些数据不是 fresh held-out；像素和相邻帧高度相关，不提供统计安全保证。

模型输入只有当前 RGB 和已知内参。推理结果先单独保存，再打开真值评分。DA3 使用官方 InputProcessor、官方网络核心和“处理后焦距除以 300”的米制转换；UniDepthV2 使用官方 infer、提供已知内参、固定最低分辨率档。深度是光轴 z 距离，三维点误差为绝对 z 误差乘像素光线范数。两模型严格加载官方权重。

评分范围为真实 z 深度 0.5–3 m、排除固定 CAD mask、离根中心至少 0.55 m、真实障碍高度 0.05–1.5 m。无效预测保留在分母；本次选中障碍点预测无效数均为零。门槛为每个关键 E1 子组至少 100 点、至少 90% 误差不超过 0.30 m，且 p90 不超过 0.30 m。每个关键子组都未通过，两候选均拒绝。

| 开发序列 | 障碍点数 | DA3 误差≤0.30 m | DA3 p90 / m | UniDepthV2 误差≤0.30 m | UniDepthV2 p90 / m |
|---|---:|---:|---:|---:|---:|''']
for seq,label in [('e0','E0'),('e1','E1'),('e2','E2'),('hard_0_e1_0003','E1 0003'),('hard_0_e1_0004','E1 0004'),('direct_e1','直接跟踪 E1')]:
    a=D['da3_metric']['sequences'][seq]['obstacle'];b=D['unidepth_v2']['sequences'][seq]['obstacle']
    parts.append(f"| {label} | {a['n']} | {100*a['within_0p3m']:.1f}% | {a['p90_m']:.3f} | {100*b['within_0p3m']:.1f}% | {b['p90_m']:.3f} |")
parts.append(f'''
![深度误差对照]({H/'figures/depth_errors.png'})

完整误差 JSON 还区分全部可见表面、0.5–1.5 m 和 1.5–3 m 障碍：{link(H/'depth_assessment.json','两模型测量结果')}。

| 模型 | 双目一对 p50 / s | p95 / s | 首对含预热 / s | 10 Hz p95 预算 |
|---|---:|---:|---:|---|
''')
for model,name in [('da3_metric','DA3Metric Large'),('unidepth_v2','UniDepthV2 ViT L')]:
    l=D[model]['latency'];parts.append(f"| {name} | {l['pair_p50_s']:.3f} | {l['pair_p95_s']:.3f} | {l['first_pair_s']:.3f} | 未通过 |")
parts.append(f'''
耗时在 RTX 3060 上同步测量，包含每眼预处理、推理、恢复分辨率，两眼串行；首对单列，剩余 65 对计算 p50/p95。没有与 Isaac 同时争用 GPU。这是感知前端耗时，不是已经测量的完整在线导航延迟。降频接入或异步补偿未验证，精度失败也不会因降频消失。

UniDepthV2 的 confidence 按官方说明是单张图内的相对尺度不变误差排序，不是安全概率。探索性检查中，0003 的最低预测误差四分位仍只有 {100*C['sequences']['hard_0_e1_0003']['lowest_reported_error_quartile']['within_0p3m']:.1f}% 点在 0.30 m 内，0004 为 {100*C['sequences']['hard_0_e1_0004']['lowest_reported_error_quartile']['within_0p3m']:.1f}%；这不能靠筛高置信点晋升。该补充没有修改接受门槛或原分母。未定义可接受的“高置信空闲”阈值，也未测量真实制动分支的危险事件及时检出率；这些安全指标仍未验证，不能用点误差比例替代。

![左右相机与实测预测深度]({H/'figures/depth_views_h4.png'})

图中保留完整原始图像和预测，包括机体与天空；评分时才应用规定的 mask 和深度区间。统一色标 0.5–3 m，超过范围饱和，不代表大于 3 m 的预测被真值校准。

## 同控制器的实际运行

独立共享控制器用 40×40 m、0.10 m 分辨率的局部地图，0.60 m 净空要求（0.40 m 球半径加 0.20 m 裕度），A* 路线、1 m 几何目标，每 0.5 秒重规划、10 Hz 发请求，换向前五次低速确认和反弹清零。两组只改变可用环境信息，roll PID 和执行链相同。

O-map 只在初始时接收完整地图。O-visible 只接收当前左右相机深度点、当前位姿/标定和从 t=0 积累的地图；floor hit 标为空闲，障碍高度点标为占据，未知阻塞，当前机体占地可初始化为空闲。不使用未来帧、视野外真值、Oracle 参考或盲倒车脚本。完整地图组也不读取 runtime 生成的 Oracle 路线。

这是新建的几何诊断控制器，尚未通过运动可执行性验证，不能当成已有强 Oracle 或部署候选。全地图正对照的困难场景失败说明当前局部执行器需要先过关；因此本轮不能把所有失败单独归因于视觉。保守地图还可能受到稀疏地面点、未知阻塞、净空裕度和缺少主动观测的影响，结果不证明固定相机视觉导航普遍不可能。

| 起点 | 信息条件 | 实际终止 | 时间 / s | 真实轨迹长度 / m | 终态目标距离 / m | 非零主轴请求帧 |
|---|---|---|---:|---:|---:|---:|''')
for label,name in [('e0','E0'),('e2','E2'),('h3','E1 0003'),('h4','E1 0004')]:
    for mode,mname in [('map','完整地图'),('visible','可见真值深度')]:
        r=A['runs'][mode+'_'+label]
        parts.append(f"| {name} | {mname} | "+{'success':'到达','collision':'物理碰撞','timeout':'超时'}[r['termination']]+f" | {r['elapsed_s']:.2f} | {r['path_length_m']:.3f} | {r['final_goal_distance_m']:.3f} | {r['nonzero_request_blocks']} |")
parts.append(f'''
完整地图组为这四条开发对照中的 2/4 到达，可见真值深度组 0/4。不是新正式集合的成功率，也不与旧 Oracle 的 5/7 合并。可见组虽没有非零主轴请求，机器人仍因实际物理状态变化累计移动约 0.62 m；不把零请求写成完全静止。完整地图 E0/E2 到达时真实速度约 0.73/0.72 m/s，原到达规则通过，但未验证到达后停稳。

![同条件实际轨迹]({H/'figures/trajectories.png'})

灰线为独立离线评估器冻结的初始全地图几何路线，未送给可见组；两条真实轨迹由同一投影器评估。目标距离使用 manifest 中的真实目标。没有使用候选 path_index、runtime 陈旧诊断字段或地图切换制造进度改善。

## 可见地图为何不动

四条可见组从首帧就返回 no_known_free_route，35 秒均没有非零主轴请求。距离值已经使用可见真值，网络精度不是这个组停车的原因。首米名义路径的 0.60 m 管道中，已知空闲（含机体占地）只覆盖约 20.6%–36.4%，其余为未知。这是当前固定视野加当前保守地图/动作资格的联合作用。

| 起点 | 首米管道格数 | 已知空闲 | 占据 | 未知 | 已知空闲比例 |
|---|---:|---:|---:|---:|---:|''')
for label in ('e0','e2','h3','h4'):
    r=A['initial_nominal_tube_coverage'][label];parts.append(f"| {label.upper()} | {r['tube_cells']} | {r['known_free']} | {r['occupied']} | {r['unknown']} | {100*r['known_free_fraction']:.1f}% |")
parts.append(f'''
首帧机体周围已知空闲的距离变换值均约 0.412 m，低于本控制器所需的 0.60 m；相机看到的较远地面与机体占地之间仍有未知。这个初始断连发生在 E0/E2 也有，必须先解决表示和观测策略的启动问题。首米管道仅是几何覆盖诊断，没有认证速度相关制动尾部。

![首帧可见地图]({H/'figures/initial_observability.png'})

历史记忆在实现中确实累积，但四条运行没启动有效移动，因而不能宣称已经验证“移动后建图有导航收益”。永久占据优先、地面 hit 离散化以及不清除不可见空间都是此保守基线的限制，不应笼统解释为相机完全看不到障碍。

## 完整地图失败和真实停止尾部

0003 首次实际接触在 9.46 s，整条运行在 9.50 s 终止，距目标 3.863 m。相对固定几何参考的峰值横向误差 0.199 m；这已接近所设 0.20 m 裕度。0004 在 16.80 s 实际接触，距目标 5.693 m，峰值横向误差 0.537 m，约 60.7% 动作帧副轴请求达到 ±30° 边界。几何路线没有充分表达这套执行器的转弯和制动能力，仍存在地图离散误差等其他可能原因；这些数值不是完整因果排除实验。

0004 在 16.00 s 因找不到已知可通行路线而将请求设为零，当时真实 XY 速度约 0.617 m/s，applied 主轴仍为 −1.369 rad/s。随后到接触前仍累计行驶约 0.419 m；16.80 s 的原请求是零，而 applied 主轴仍为 −0.969 rad/s。正式 smoother 保留减速时间，零请求不等于立即停下。这里直接用实际请求、applied 和物理速度核验，没有用目标斜率冒充实际停止测量。

本轮没有改 PID、阈值、速度策略、映射或碰撞判定来追求更好分数。没有充分证据把当前问题定为一个具体实现 bug，因此没有做勉强的单参数补丁。应把非零初速、实际执行状态和停止尾部纳入同一动作资格检查；不能仅在几何路径消失后才停车。

## 接口和验证证据

- 四对起点的 XY、yaw、姿态历史、本体历史、动作历史和 PointGoal 在首帧逐元素相同，最大差值均为零。policy/PID/smoother 每条完整运行重新初始化，配置和源码身份相同。
- 八条运行左右相机图像时钟都与当前 raw 物理时钟匹配，最大图像年龄绝对值为零；刷新 render 不改变物理时钟或根位姿、速度与关节状态，实际断言均通过。
- 诊断标记在每次 capture 都检查不可见。所有失败接触来自 physical_contact_v2；时间以实际物理采样为准，未把地图裕度触发当成真实碰撞。
- 22 项 CPU 测试通过：转正后像素光线/内参、深度恢复、无效输出分母、floor/body 分离、未知阻塞、历史保留、净空管道、低速换向和反弹、同地图同状态动作一致、packet 只能消费一次、可见组禁止完整地图、无关的未来临时文件创建/删除不改变当前动作。
- 两个 checkpoint 使用冻结 revision、严格权重加载。DA3 下载完整；UniDepth 下载中断后仅恢复同一文件字节，并匹配官方大小和 SHA256。未换模型、未重训。

source/checkpoint 身份和最终核验见{link(H/'final_verification.json','最终验证记录')}。{link(H/'run_assessment.json','实际运行与初始状态对齐')}、{link(H/'final_tests.txt','测试输出')}、{link(H/'progress.md','执行记录')}分别保留原始证据。推理日志、132 个每模型单眼输出对应的 66 个双目结果包、物理 50 Hz 四层执行数据、当前图像、本体状态、接触审计和相机时钟均保存在独立实验目录。

| 层次 | 本轮结论 | 不能推出什么 |
|---|---|---|
| 图像同步和几何测量接口 | 本轮检查通过 | 不代表距离模型准确或控制安全 |
| 米制深度精度与时延 | 两候选拒绝 | 不代表所有深度方法都无效 |
| 可见地图与启动资格 | 当前保守基线未通过 | 不代表所有主动观测/地图表示不可能 |
| 同控制器完整地图导航 | 简单回归通过、困难起点失败 | 不代表旧成功 Oracle 退步或视觉单独造成失败 |
| RGB 导航/世界模型效果 | 新候选未接入、未训练 | 不存在可报告的新视觉导航成功率 |

## 视频和下一项实验

每条完整运行都有第三人称与双目视频。代表性视频：

''')
for label,title in [('e2','完整地图 E2 到达'),('h3','完整地图 0003 接触'),('h4','完整地图 0004 减速后接触')]:
    r=A['runs']['map_'+label];parts.append('- '+link(r['artifacts']['third_person.mp4'],title)+'；'+link(r['artifacts']['stereo_view.mp4'],'对应双目画面'))
r=A['runs']['visible_h4'];parts.append('- '+link(r['artifacts']['third_person.mp4'],'可见真值深度 0004 超时')+'；'+link(r['artifacts']['stereo_view.mp4'],'对应双目画面'))
parts.append('''
下一项优先工作应为：让同一个局部执行器在完整地图下先通过困难起点，候选资格同时检查真实可执行转弯段与停止尾部；再处理可见地图从机体占地到远处地面的启动断连，明确哪些空闲来自测量、哪些来自可验证的几何假设。保持相机不动，不用盲倒车或“未知当空闲”绕过问题。通过全地图局部执行和可见深度资格后，再接入经单独验证的视觉距离前端。

世界模型可以随后学习这一可执行动作库的状态相关响应与观测不确定性；现在继续扩大网络或仅靠 confidence 筛选，不能修复上述实际控制和地图资格失败。本阶段的交付是把失败层用同条件数据分开，并拒绝不适合当前视野的两个冻结感知候选；尚未完成最终视觉导航目标。
''')
out=W/'reports/benchmark_visual_information_and_metric_depth_diagnostic_20261004.md';out.parent.mkdir(exist_ok=True);out.write_text('\n'.join(part.rstrip() for part in parts)+'\n')
print(out)
