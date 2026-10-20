from pathlib import Path
import json
h=Path(__file__).resolve().parent;rows=json.loads((h/'navigation_comparison.json').read_text());assert len(rows)==10
p=json.loads((h/'probe_assessment_corrected.json').read_text());selected=json.loads((h/'selected_candidate.json').read_text())
lines=['# 倾侧角 PID 闭环与导航 Benchmark 对照（2026-09-29）','',
'## 结论','',
'已完成 4 条开放平面运行（两组 B0/PID 配对）和 10 条带倾侧闭环的完整导航运行。本轮两档正向速度的开放平面保持试验中倾侧稳定性改善，但本版本没有通过导航替换验收：golden 从 3/3 降为 2/3，原来成功的 E1 变成超时。hard_0 地图碰撞由 1/7 增至 3/7。因此保留原 Oracle，不把实验候选部署为新基线。没有训练网络，也没有按导航成绩重新调参。','',
'本轮交付的是实际运行结果，不是仅离线回放或控制建议。所有新导航均使用原始 35 秒规则和 0.2 m 首次到达标准；并未证明到达后稳定停驻。','',
'## 控制方式与边界','',
'原 Oracle 只调用一次，继续使用其参考、pure-pursuit、V1 逆映射及 yaw-rate 修正。原请求先按正式协议裁到 ±30°，随后以 **期望倾侧 β*=0.75×副轴协议请求** 作为初版目标转换。这个比例只是基于既有长时响应的保守近似，不是验证过的曲率→倾侧定律。','',
'β=asin(2(wx+yz))，使用归一化 base_link 世界姿态 wxyz；含义为机体 +Y 轴相对水平面的有符号仰角，即本项目已核验的物理倾侧。它与机体绕局部 X 的倾侧相关，但不把一般三维欧拉 roll 与它无条件等同。','',
'每个 0.02 秒物理步前读取当前姿态，输出副轴**位置目标**：u=clip(原副轴请求+0.8e+I−0.35βdot_filtered, ±30°)。积分增益 0.10/s，积分输出限幅 ±5°，导数滤波时间常数 0.1s。目标变化不直接进入 D 项，避免设定值跳变冲击。角度饱和时条件积分；实际速度 supervisor 限制使用上一物理步的标志冻结积分（20ms 延迟）。','',
'PID 后继续调用原 ActionSmoother 和 supervisor，主轴加速度限幅 0.5 rad/s²、副轴目标斜率 30°/s、副轴速度 supervisor 阈值 0.30 rad/s 均保持。后者仍是超速反馈监督，不等于物理速度绝不瞬时超过阈值。每条序列单独重置，序列内部不重置状态。','',
'这不是实机 PID 的精确复刻：尚未获得实机增益及输出模式。本轮是利用现有位置执行接口验证“倾侧闭环能否改善控制”。规划、换向阈值和五次低速确认、外层增益、速度策略、参考和动作范围均未改。反馈改变运动后，上层产生不同的后续动作属于闭环响应，不代表直接改了速度策略。','',
'## 开放平面实测','',
'主轴分别为 0.5/1.5 rad/s；0–5s 回正，5–15s 目标倾侧 +15°，15–25s −15°，25–35s 回正。B0 请求副轴 ±20°。统计各保持段末尾 3 秒（12–15、22–25、32–35s），不是把瞬态当稳态。没有专门新增倒车试验，不能宣称所有速度/方向鲁棒。','',
'| 主轴命令 rad/s | B0 综合尾窗 RMS ° | PID RMS ° | 降幅 | PID 回正 RMS ° |','|---|---:|---:|---:|---:|']
for speed in [.5,1.5]:
 b=next(x for x in p['rows'] if x['main']==speed and x['mode']=='b0');c=next(x for x in p['rows'] if x['main']==speed and x['mode']=='pid')
 lines.append(f"| {speed} | {b['tail_rms_deg']:.3f} | {c['tail_rms_deg']:.3f} | {100*(1-c['tail_rms_deg']/b['tail_rms_deg']):.1f}% | {c['windows'][-1]['rms_error_deg']:.3f} |")
lines+=['','两组均通过预定有限状态、倾侧绝对值<35°、尾窗波动幅度<10°、综合 RMS 不变差和回正 RMS 不恶化超过2°的门槛。只用了首组增益，另外两组备选增益未运行。较高速负倾侧仍约有3°误差，低速负倾侧单个尾窗也并非每项都改善；结论限于综合误差与摆动，不是零误差控制。','',f'![倾侧响应]({h}/bank_probe_corrected.png)','',
f'完整逐窗实测速率、实际 XY 速度、PID 裁剪比例、supervisor 比例与有限性检查见 [校正后原始汇总]({h}/probe_assessment_corrected.json)。','',
'独立复核发现最初分析把 reset 的 t=0 样本标成 t=.02，导致窗口偏移一物理步。原分析和选参文件保留，新增分析使用日志中的 scored simulation_time_s；重算后两组门槛仍通过，参数未改变。4 runs 是两组配对，不是四组配对。','',
'## 导航实测逐条对照','',
'B0 是 2026-09-28 保存的实际运行，不是本轮重跑；本轮对照前核验其全部冻结源码与7项顶层资产/占据图哈希均未变化。每条比较还核验了相同规划参考与初始位置、姿态标量、速度、关节和执行动作。未宣称递归资产依赖或跨机器仿真完全确定性。','',
'| 场景 | B0 结果/秒 | 倾侧 PID 结果/秒 | PID 终距 m | B0/PID 实际接触采样数 |','|---|---|---|---:|---:|']
for r in rows:
 def desc(x):return ('成功' if x['success'] else ('地图碰撞' if x['collision'] else '超时'))+f" / {x['elapsed_s']:.1f}"
 lines.append(f"| {r['episode'].replace('episode_','')} | {desc(r['baseline'])} | {desc(r['candidate'])} | {r['candidate']['final_distance_m']:.3f} | {r['baseline_physical_contacts']}/{r['candidate_physical_contacts']} |")
for suite in ['golden','hard_0']:
 rr=[r for r in rows if r['suite']==suite]
 lines+=['',f"{suite}：B0 {sum(int(r['baseline']['success']) for r in rr)}/{len(rr)}，PID {sum(int(r['candidate']['success']) for r in rr)}/{len(rr)}；PID 地图碰撞 {sum(int(r['candidate']['collision']) for r in rr)}，超时 {sum(int(r['candidate']['timeout']) for r in rr)}。",'',f'![{suite} 对照轨迹]({h}/{suite}_paired_paths.png)']
lines+=['','golden E0 与 hard_0 的 cluttered-0_e0_0002 是重复物理任务，不把10次运行当10个独立泛化样本。这些都是既有开发场景，没有 fresh held-out 成功率声明。','',
'“实际接触采样数”是10Hz状态中滤掉地面后的障碍接触布尔值计数，不是独立碰撞次数；无接触采样也不证明两个采样之间绝无碰撞。正式地图碰撞与实际接触沿用旧口径分别报告，不能互相替代，未为提高成绩改动评测标准。','',
'## 导航失败的具体证据与判断','']
e=next(x for x in rows if x['episode']=='episode_golden_e1_0001')
lines += [f"golden E1：B0 在 {e['baseline']['elapsed_s']:.1f}s 到达；PID 在35s仍距目标 {e['candidate']['final_distance_m']:.3f}m。使用同一参考折线、独立于 policy path_index 的最近线段距离，横向 RMS 从 {e['baseline_geometric_cross_track_rms_m']:.3f}m 增至 {e['candidate_geometric_cross_track_rms_m']:.3f}m，PID peak {e['candidate_geometric_cross_track_peak_m']:.3f}m。此几何误差不是索引推进指标，也不能单独证明反向段进度。",'',
f"该条导航 PID 输出角度裁剪占 {100*e['pid_clipped_fraction']:.1f}% 的物理步，副轴速度 supervisor 激活占 {100*e['supervisor_fraction']:.1f}%。这是新增 PID 输出裁剪，不能混同旧 raw ±35°→协议±30°裁剪。真实反馈链见单独的 feedback/bank_feedback_*.json；旧 metrics 中辅助反馈数组不表示此次倾侧反馈。",'',
'本轮导航中变化目标的倾侧误差 RMS 约8–14°，因此不能把保持试验改善推广成导航中倾侧也已稳定。\n\n这些结果支持：开放平面保持时倾侧稳定性是可以改善的，但直接在旧映射后加入闭环，会改变转向动态，现有上层转向目标与执行响应还不匹配。不能据此认定 PID 方法不可行，也不能唯一归因于某个比例、增益或曲率极限；这些因素尚未被独立隔离。','',
'下一项具体工作应先在**本轮开放平面保持试验中改善的倾侧闭环**上测量少量“实测速度—目标/实测倾侧—实际曲率”关系，再替换路径曲率到期望倾侧的临时转换，并用同一参考验证。不要继续沿用未经核验的0.75比例并盲目加大 PID 增益，也不要同时改 planner 和限速规则。本轮不宣称完成该标定。','',
'## 验证与复现','',
'- 控制单元测试先失败后实现：5项通过，包含反馈符号、物理倾侧定义、限幅、积分与reset、设定值不引起导数冲击。浮点32输出限幅断言采用1e-7数值容差。',
'- 针对既有 policy/runtime/坐标/执行链和新PID的测试合计172项通过，3条依赖弃用警告。',
'- 额外2项真实 smoother 接口检查通过：零反馈逐步输出与B0完全相同；非零反馈主轴输出相同；后缀状态改变不影响前缀输出。没有用这些CPU测试代替实际仿真。',
'- 四次开放平面与两套导航父进程退出码均0；每条新导航有完整指标、状态、动作和视频。Isaac日志仍有已有渲染primvar警告/错误消息，不等于导航 runtime_error；视频首尾解码单独核验。',
'- 20个导航视频均有正帧数/帧率，首尾可解码；并非逐帧质量审查。源码冻结期间未变化。',
'- 独立只读审查未发现阻塞控制缺陷，指出的时间轴与证据缺项已在独立校正产物中修正，旧证据未覆盖。',
'- 所有实现保存在本实验目录，原仓库控制器源码未修改，未提交、推送或提升为默认策略。','',
f'实验目录：`{h}`','',
'同一安装环境验证与复跑（输出目录必须不存在；Isaac执行需相同GPU环境）：','',
'```bash',f'/home/an/miniconda3/envs/ball_wm/bin/python "{h}/reproduce.py" --check-only',f'/home/an/miniconda3/envs/ball_wm/bin/python "{h}/reproduce.py" --suite golden --run-root /tmp/roll_pid_golden_fresh_run','```','',
'## 视频与数据','']
for r in rows:
 ep=h/('navigation_'+r['suite'])/r['episode'];name=r['episode'].replace('episode_','')
 lines.append(f'- {name}：[第三人称视频]({ep}/third_person.mp4) · [双目视频]({ep}/stereo_view.mp4) · [轨迹]({ep}/trajectory_overlay.png)')
report=h.parents[1]/'reports/benchmark_oracle_roll_pid_navigation_20260929.md';report.write_text('\n'.join(lines)+'\n');print(report)
