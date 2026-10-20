from pathlib import Path
import json,numpy as np
h=Path(__file__).resolve().parent;rows=json.loads((h/'navigation_comparison.json').read_text());assert len(rows)==10
cal=json.loads((h/'calibration_assessment.json').read_text());val=json.loads((h/'validation_assessment.json').read_text());trans=json.loads((h/'transient_assessment.json').read_text())
golden=[r for r in rows if r['suite']=='golden'];hard=[r for r in rows if r['suite']=='hard_0'];e=next(r for r in rows if 'golden_e1' in r['episode'])
lines=['# 实测曲率—倾侧映射修正与导航对照（2026-09-29，V2）','',
'## 结论','',
f"完成4条100秒标定、2条65秒独立完整验证，以及10条原规则导航。稳态映射通过验证，8个保持窗口的曲率绝对误差为 {min(r['error'] for r in val['rows']):.3f}–{max(r['error'] for r in val['rows']):.3f}/m。它们来自两条序列，不是8个独立序列。",'',
f"导航仍未达到替换原基线的条件：golden {sum(int(r['candidate']['success']) for r in golden)}/3，hard_0 {sum(int(r['candidate']['success']) for r in hard)}/7。golden E1终距从上一版PID的 {e['previous_pid']['final_distance_m']:.3f}m 降至 {e['candidate']['final_distance_m']:.3f}m，但35秒内仍未到达；原B0在32.7秒已成功。更近不算成功。",'',
f"hard_0 本版地图碰撞 {sum(int(r['candidate']['collision']) for r in hard)}/7，原B0为1/7，上一版PID为3/7。保留B0，不提升本候选为默认导航控制器。",'',
'新增证据把问题进一步分开：稳态“给多少倾侧才产生所需曲率”已在中间速度上验证；转向请求切换后的实际曲率仍过冲和摆动，持续进入允许误差带需要约4–6秒。后者是尚未解决的动态问题，不能靠稳态查表通过来宣称解决。它是重要瓶颈证据，但不证明所有导航失败仅由它造成。','',
'## 本轮唯一控制改动','',
'在原 Oracle 的 tracking 请求出口，仅替换旧 V1 的曲率逆映射：当前实测 XY 速度、当前请求前进/倒车方向与本次几何曲率 → 实测表插值 → 期望倾侧 → 原冻结PID执行。仍保留原 outer yaw-rate 修正，且保持主轴请求、planner、参考、增益、动作范围、smoother、速度supervisor和换向五次停稳规则。', '',
'PID与上一版文件逐字节相同：Kp=.8、Ki=.1、Kd=.35，50Hz测量物理倾侧，输出副轴位置目标。内部仍以 auxiliary=bank/.75 传递请求，这里的 .75 是冻结接口编码；目标 bank 来自实测逆表，不再由旧 V1 转向角直接乘 .75 得到。外层 yaw 修正仍在这个编码后按旧单位加入，因此当前候选没有宣称所有上层控制都已按倾侧重新设计。', '',
'映射只描述固定PID下“请求倾侧→稳定运动曲率”的经验响应，不是瞬态预测网络，也没有拟合新的世界模型。正反向分别拟合，查询只有当前信息；测量速度、曲率越界则夹到已测支持范围，不外推更强转弯能力。不能把请求22.5°等同实际已达到22.5°。','',
'## 实测设计与稳定性检查','',
'四条序列主轴为 +.5、+1.5、−.5、−1.5 rad/s，每条开头完整reset，内部不重置：回正5s，+15°20s，+22.5°20s，−15°20s，−22.5°20s，回正15s。共20个预先指定窗口，全部通过预定有效速度、曲率IQR、倾侧稳定性和实际运动方向检查。没有增加参数扫描。','',
'几何曲率由世界XY轨迹计算；body-forward曲率=sign(main)×几何曲率，并以速度在body heading上的投影核验实际正反向。倾侧 β=asin(2(wx+yz))，使用归一化base_link世界四元数。窗口为每段末尾5s剔除最后.5s；31点三阶中心滤波仅用于离线测量，并保存21/41点敏感性。中心滤波不是在线控制输入。','',
'| 主轴 rad/s | 请求倾侧 ° | 实际倾侧 ° | 实际速度 m/s | body曲率 /m | 曲率IQR |','|---:|---:|---:|---:|---:|---:|']
for r in cal:lines.append(f"| {r['main']:+.1f} | {r['requested_bank_deg']:+.1f} | {r['actual_bank_deg']:+.2f} | {r['actual_speed_m_s']:.3f} | {r['curvature_body_1_m']:+.3f} | {r['curvature_iqr']:.3f} |")
lines+=['',f'![实测映射]({h}/calibration_map.png)','',
'同一主轴速度下实际速度也随倾侧变化；表的低/高速度节点取各档稳态实测速率的中位数。因此这是有限数据的近似插值，不是独立操纵速度和姿态后的完整因果定律。','',
'## 独立序列验证','',
'冻结表后，用未参与拟合的主轴 +1.0/−1.0 rad/s 两条完整序列验证。回正5s后依次请求曲率 +.4、−.4、+.7、−.7/m，各15s。没有根据验证结果重新拟合或改PID。验收看每段最后4.5s的稳态曲率：方向正确，绝对误差≤max(.20,.30×|请求|)，有效几何采样≥90%。','',
'| 主轴 rad/s | 请求曲率 /m | 实际曲率 /m | 绝对误差 /m |','|---:|---:|---:|---:|']
for r in val['rows']:lines.append(f"| {r['main']:+.1f} | {r['requested_curvature']:+.2f} | {r['actual_curvature_body']:+.3f} | {r['error']:.3f} |")
lines+=['','八个窗口均通过，但这只是稳态插值验证。独立检查实际运动方向、固定PID配置与全部有限状态；当前映射复算1300条已保存查询一致，固定PID逐步重建6500条输出一致。相关哈希是运行后保存，不能冒充运行前签名；完整输入输出一致性与配置共同提供溯源证据。','',
f'![完整验证曲率，含瞬态]({h}/validation_curvature.png)','',
'转向切换并不平滑。以下是事后诊断，不改变原验收门槛：以请求值±原允许误差为带，统计最后一次离开误差带后直到段末−.5s均留在带内的时间。离线中心滤波时间支撑约.6s，所以时刻不应解释成精确在线触发时间。','',
'| 主轴 | 请求曲率 | 持续进入误差带约需 s | body曲率反号累计 s |','|---:|---:|---:|---:|']
for r in trans['rows']:lines.append(f"| {r['main']:+.1f} | {r['requested_curvature']:+.2f} | {r['last_exit_then_stays_in_band_s']:.2f} | {r['wrong_curvature_sign_duration_s']:.2f} |")
lines+=['','反号指经行进方向换算后的body曲率与body请求反号，不代表机器人发生前后换向；倒车时原始几何曲率符号本来就与body曲率相反。启动时低速曲率尖峰保留在图中，没有用于稳态拟合。','',
'## 导航结果：相同35秒标准','',
'| 场景 | 原B0 结果/秒 | 上一版PID 结果/秒 | 新映射PID 结果/秒 | 新终距 m | B0/新实际接触采样数 |','|---|---|---|---|---:|---:|']
def desc(x):return ('成功' if x['success'] else ('地图碰撞' if x['collision'] else '超时'))+f"/{x['elapsed_s']:.1f}"
for r in rows:lines.append(f"| {r['episode'].replace('episode_','')} | {desc(r['baseline'])} | {desc(r['previous_pid'])} | {desc(r['candidate'])} | {r['candidate']['final_distance_m']:.3f} | {r['baseline_physical_contacts']}/{r['candidate_physical_contacts']} |")
lines+=['','成功是首次进入0.2m范围，不等于停稳。地图碰撞与10Hz障碍接触采样分别报告：零接触采样不否定地图安全边界触发，也不能排除采样间接触。golden E0和hard_0的cluttered-0_e0_0002是同一物理任务，不把10次算成10个独立泛化样本。旧B0和PID结果为保存的实际运行，本轮没有重跑两整套对照。','',
'所有十条均核对了相同的完整规划参考及初始实测位置、姿态标量、线/角速度、关节和applied动作。基础源码、7项顶层场景/机器人/占据图资产哈希一致，不声称完整递归资产包跨机器可复现。评测使用既有dev/golden，无新的held-out导航成功率声明。','',
f"golden E1独立最近参考线段横向RMS：B0 {e['baseline_geometric_cross_track_rms_m']:.3f}m，上一版PID {e['previous_pid_cross_track_rms_m']:.3f}m，新版 {e['candidate_geometric_cross_track_rms_m']:.3f}m。这个指标独立于policy path_index，但不能单凭它证明方向段进度；正式到达和碰撞结果仍优先。",'',
f"golden E1新版PID输出角度裁剪占 {100*e['pid_clipped_fraction']:.1f}% 物理步，速度supervisor占 {100*e['supervisor_fraction']:.1f}%；映射请求曲率超表范围的tracking块占 {100*e['mapping_curvature_clamp_fraction']:.1f}%，实测速度夹到表端点的tracking块占 {100*e['mapping_speed_clamp_fraction']:.1f}%。这几种裁剪统计不是同一层含义。",'',
f'![golden三方案轨迹]({h}/golden_paired_paths.png)','',f'![hard_0三方案轨迹]({h}/hard_0_paired_paths.png)','',
'## 日志问题、修正与验证','',
'首次导航封装原计划在_run_one外层finally保存mapping_records。但Isaac原生close会结束worker，外层保存没有执行；动作、状态、metrics、视频及close前的50Hz PID日志均已保存。这是诊断持久化缺陷，不是控制器未运行。','',
'原批次文件保留不覆盖。新增reconstruct_mapping.py只用已保存的每步当前状态、当步曲率及动作，复算映射和旧yaw修正，并逐块核对实际aux请求，最大误差要求<1e-6 rad。产物明确命名mapping_reconstructed，属于离线复核，不能冒充原始在线记录。','',
'另新增关闭前保存的logging封装，先用回归测试复现缺失再修复，并单独重跑golden E1验证日志和动作一致性。该补跑不计为新场景或额外成功样本。日志修复不改控制、参数或评测。实际在线记录位于logging_verification/feedback，修复后的复跑入口保留。','',
'继承的model_proxy_curvature_*仍是替换前旧V1辅助目标的计算值，不能当作候选预测。分析采用实际保存的动作、50Hz PID记录和上述明确标记的映射复核。','',
'## 测试、归档与下一项控制工作','',
'- 新逆映射与实际Oracle接口测试先失败后实现；速度插值、范围限制、非单调拒绝、正反向实接口、缺失方向回退和early return均有覆盖。161项相关回归通过。额外关闭前保存日志测试通过。',
'- 4条标定、2条验证、10条导航均实际执行；状态、退出码、源文件快照和视频核验保留。复跑材料依赖同一Isaac/GPU安装环境。',
f'- 标定与验证的完整执行统计见 [饱和汇总]({h}/calibration_execution_summary.json) 和 [验证溯源]({h}/validation_provenance.json)，包含PID裁剪、速度夹取、supervisor及倾侧误差。',
'- 导航期间源文件未变，映射/PID冻结；旧基线源码与旧结果未覆盖。没有提交或推送。',
'- 本候选稳态映射层接受，整套导航控制候选拒绝替换B0。不得把离目标更近、碰撞更晚或静态验证通过写成导航成功。','',
'下一项应针对已量化的转向切换过程：在相同开放平面参考上，验证有限前视的倾侧目标是否能提前消化4–6秒的曲率过渡，同时保持原速度/安全边界。不能把4–6秒直接设成统一提前量，更不能强制提前前后换向。先用真实连续轨迹检验动态收益，再进入相同35秒导航；本轮没有实现这一新控制项。','',
f'实验目录：`{h}`','',
'```bash',f'/home/an/miniconda3/envs/ball_wm/bin/python "{h}/reproduce.py" --check-only',f'/home/an/miniconda3/envs/ball_wm/bin/python "{h}/reproduce.py" --suite golden --run-root /tmp/bank_mapping_golden_fresh','```','',
'## 实际视频','']
for r in rows:
 ep=h/('navigation_'+r['suite'])/r['episode'];lines.append(f"- {r['episode'].replace('episode_','')}：[第三人称]({ep}/third_person.mp4) · [双目]({ep}/stereo_view.mp4) · [轨迹]({ep}/trajectory_overlay.png)")
report=h.parents[1]/'reports/benchmark_oracle_bank_mapping_navigation_20260929.md';report.write_text('\n'.join(lines)+'\n');print(report)
