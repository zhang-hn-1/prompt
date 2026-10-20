from pathlib import Path
import json,numpy as np
h=Path(__file__).resolve().parent;x=json.loads((h/'tuning_assessment.json').read_text());sel=json.loads((h/'selected_candidate.json').read_text());nav=json.loads((h/'navigation_comparison.json').read_text());best=next(r for r in x['candidates'] if r['tag']==sel['tag']);diag=x['supervisor_sensitivity'];baseforward=next(r for r in x['baseline'] if r['main']==1)
def stats(rr):
 return (float(np.mean([r['mean_settling_s'] for r in rr])),float(np.sqrt(np.mean([w['transient_bank_rate_rms_deg_s']**2 for r in rr for w in r['windows']]))),float(np.mean([w['tail_bank_span_deg'] for r in rr for w in r['windows']])))
bs=stats(x['baseline']);cs=stats(best['rows']);gold=[r for r in nav if r['suite']=='golden'];e=next(r for r in gold if 'e1' in r['episode'])
lines=['# PID动态整定、限速敏感性与导航诊断（2026-09-29，V3）','',
'## 结论','',
'本轮golden从调参前的2/3恢复到3/3，出现实际导航改善。整定了PID的P、D参数，没有再次把转向映射修改当作PID调参。测试4组预先指定增益，每组前进/倒车各一条完整65s序列；另做1条副轴速度supervisor敏感性对照。积分增益固定，没有声称找到全局最优PID。','',
f"最好得分为 Kp={sel['gains']['kp']}、Ki={sel['gains']['ki']}、Kd={sel['gains']['kd']}。相较原(.8,.1,.35)，曲率过渡前8秒RMS从{x['baseline_curvature_rms']:.3f}降至{best['curvature_rms']:.3f}/m（{100*best['improvement_fraction']:.1f}%）；平均持续进入误差带时间从{bs[0]:.2f}秒缩短至{cs[0]:.2f}秒。",'',
f"改善有限：倾侧目标跟踪RMS从{x['baseline_bank_rms_deg']:.2f}°变为{best['bank_rms_deg']:.2f}°，没有明显改善。所有候选均未达到预定的15%瞬态曲率RMS改善门槛。保留该拒绝结论，未触发fresh验证及hard_0整套导航；只执行预先允许的最佳得分候选golden诊断。",'',
f"本轮golden诊断为{sum(int(r['candidate']['success']) for r in gold)}/{len(gold)}成功。golden E1在{e['candidate']['elapsed_s']:.1f}秒时终距{e['candidate']['final_distance_m']:.3f}m，结果为{e['candidate']['termination_reason']}。不能把小幅平滑、较晚失败或少跑困难场景写成新baseline成功。未替换原B0。",'',
f"把副轴速度监督阈值从.30放宽至.60rad/s（仅开放平面诊断）后，同一前进序列的曲率瞬态RMS从{baseforward['curvature_rms']:.3f}增至{diag['curvature_rms']:.3f}/m，增加{100*(diag['curvature_rms']/baseforward['curvature_rms']-1):.1f}%。放松限制不是有效改进，本轮导航始终保持.30。",'',
'## 实验边界与筛选','',
'冻结上一轮经验证的曲率→期望倾侧映射、原planner/参考、主轴速度策略、外层yaw反馈、动作范围与smoother。PID执行仍是50Hz、输出副轴位置目标的外环；滤波时间常数.1s、积分输出±5°、原抗饱和逻辑不变。主轴加速度.5rad/s²、副轴目标斜率30°/s不变。','',
'整定序列：主轴+1.0或−1.0rad/s，开头回正5s，依次请求body曲率+.4、−.4、+.7、−.7/m，各15s。每条初始reset，内部不重置。基线复用上一轮同条件真实序列；核对代码、PID实现和映射一致。4组新参数=8条运行；限速诊断另1条，合计9条。','',
'选择标准在运行前写入PROTOCOL：转向切换后前8s曲率误差RMS至少降15%；同窗倾侧跟踪误差不得恶化超过10%；所有原稳态尾窗曲率门槛通过；状态有限且倾侧绝对值<35°。只从合格者选择最低曲率RMS；若无人合格，最低得分者仅进入golden诊断。未根据导航成绩追加或修改增益。','',
'原稳态门槛：每段最后5s剔除末.5s，曲率误差≤max(.20,.30×|请求|)，符号正确，几何可估计≥90%，实际运动方向匹配≥90%。前8s统计中，本轮所有已比较窗口几何有效覆盖均100%，不存在低速样本被排除而虚假提高成绩。','',
'## 四组参数的实测结果','',
'| 参数组 (P,I,D) | 曲率瞬态RMS /m | 改善 | 倾侧跟踪RMS ° | 平均入带时间 s | 倾侧角速度RMS °/s | 尾窗倾侧95–5跨度 ° | 筛选 |','|---|---:|---:|---:|---:|---:|---:|---|',
f"| 原(.8,.1,.35) | {x['baseline_curvature_rms']:.3f} | — | {x['baseline_bank_rms_deg']:.2f} | {bs[0]:.2f} | {bs[1]:.2f} | {bs[2]:.2f} | 对照 |"]
for r in x['candidates']:
 st=stats(r['rows']);lines.append(f"| ({r['gains']}) | {r['curvature_rms']:.3f} | {100*r['improvement_fraction']:.1f}% | {r['bank_rms_deg']:.2f} | {st[0]:.2f} | {st[1]:.2f} | {st[2]:.2f} | {'通过' if r['eligible'] else '未通过'} |")
lines+=['','四组稳态尾窗、有限状态、倾侧边界及倾侧误差不恶化门槛均通过，未通过项都是15%瞬态曲率改善。P升高并未更好；较强D缩短曲率持续进入误差带的时间，但并未显著减小整个过渡过程的倾侧跟踪误差。物理角速度RMS和尾窗角跨度用于补充区分运动快慢、摆动和静差，不能单独当作更安全的导航证据。','',
f'![PID动态对照]({h}/tuning_transients.png)','',
'平均入带时间定义为：转向开始后，最后一次离开请求曲率±允许误差带，再直到该段末−.5s都保持在带内的时间。曲率为离线31点中心滤波轨迹导数，时间支撑约.6s，不是在线精确停止事件。body曲率=sign(main)×世界XY几何曲率，倒车符号已核验。','',
'独立复核指出首次初步分析把反馈[i−1]配到了物理状态[i]；最终选择前已改用反馈[i]与同一时刻物理状态对齐，旧初步结果保留。曲率评分未变，倾侧误差均按同一修正重算，未修改任何门槛。','',
'## 副轴速度限制的独立敏感性实验','',
f"只比较同一前进+1.0rad/s序列、原PID(.8,.1,.35)：监督阈值.30时曲率RMS {baseforward['curvature_rms']:.3f}/m，.60时{diag['curvature_rms']:.3f}/m；平均入带时间分别{baseforward['mean_settling_s']:.2f}s和{diag['mean_settling_s']:.2f}s。",'',
'原限制下实际副轴速度p95约.237rad/s、峰值.317rad/s；放宽后p95约.342rad/s、峰值.540rad/s。该supervisor是反馈式限速，不能把.30阈值说成从不超越的硬物理上限。','',
'原supervisor只约0.6%的物理步介入，但放宽后的效果明显恶化。由此可见，触发频率低不代表影响小；本轮数据不支持通过放宽限制来改善转向。该结论只来自这一条受控敏感性序列，不推广成所有速度的普遍因果定律。','',
'## 导航诊断（原35秒规则）','',
'| 场景 | 原B0 结果/秒 | 调参前映射PID 结果/秒 | 本轮PID 结果/秒 | 本轮终距 m | 实际接触采样数 |','|---|---|---|---|---:|---:|']
def desc(m):return ('成功' if m['success'] else ('地图碰撞' if m['collision'] else '超时'))+f"/{m['elapsed_s']:.1f}"
for r in nav:lines.append(f"| {r['episode'].replace('episode_','')} | {desc(r['baseline'])} | {desc(r['previous_pid'])} | {desc(r['candidate'])} | {r['candidate']['final_distance_m']:.3f} | {r['candidate_physical_contacts']} |")
lines+=['','本轮未运行hard_0，不能把前轮3/7当作本轮结果。也未运行额外fresh序列，因为整定门槛先失败；没有声称参数获得独立泛化验证。golden是既有场景，最佳参数经过开发集选择，其导航只能视为诊断。','',
'到达仍为0.2m首次进入，不是稳定停驻；地图碰撞与10Hz物理障碍接触分别记录。零接触采样不排除采样间接触。基础源码和7项顶层资产哈希一致，保存的参考和初始实测状态完全一致；B0和调参前PID使用历史实测结果，不声称本轮重新跑了两整套对照。','',
f'![golden轨迹对照]({h}/golden_paired_paths.png)','',
'## 验证、交付与判断','',
'- 9条整定/限速序列和golden实际仿真均有状态、50Hz执行记录、配置与进程退出证据；没有读取未来状态控制。',
'- 导航mapping与PID记录在Isaac原生close前保存，本轮不是离线重构伪装在线记录。',
'- 不变控制器实现及实际Oracle接口的166项回归测试通过；新参数效果依靠上述实际物理试验评估。',
'- 参数选择后源码和配置冻结；原生产策略没有修改或覆盖。视频首尾帧、同初态/参考和复跑哈希已核验。',
'- 原增益在本轮已测参数组中并非曲率瞬态评分最好，适当降低P、增加D缩短了曲率持续进入误差带的时间；但在当前位置执行接口与速度限制下，本轮四组整定尚未证明能解决困难场景导航。不能据此断言所有PID都无效，也不继续无边界扫参。','',
'这组参数值得进入下一轮单独冻结的独立验证与困难场景评测，但不能因golden成功而追改本轮15%筛选门槛。同时应对齐实机PID输出接口：本实验PID输出副轴位置目标，如果实机输出副轴速度或力矩，控制对象就不同。在这一点明确前，不应把仿真这四组增益直接移植实机，也不应仅凭本轮结果认定规划必须重写。现有数据可以用于进一步设计过渡控制，但本轮没有宣称已经解决。','',
f'实验目录：`{h}`','',
'```bash',f'/home/an/miniconda3/envs/ball_wm/bin/python "{h}/reproduce.py" --check-only',f'/home/an/miniconda3/envs/ball_wm/bin/python "{h}/reproduce.py" --suite golden --run-root /tmp/pid_tuning_golden_fresh','```','',
'## 视频','']
for r in nav:
 ep=h/('navigation_'+r['suite'])/r['episode'];lines.append(f"- {r['episode'].replace('episode_','')}：[第三人称]({ep}/third_person.mp4) · [双目]({ep}/stereo_view.mp4)")
report=h.parents[1]/'reports/benchmark_oracle_pid_tuning_navigation_20260929.md';report.write_text('\n'.join(lines)+'\n');print(report)
