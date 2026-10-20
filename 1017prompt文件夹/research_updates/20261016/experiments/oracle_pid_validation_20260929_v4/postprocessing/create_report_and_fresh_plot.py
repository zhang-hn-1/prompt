from pathlib import Path
import json,hashlib,numpy as np,sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_pid_validation_20260929_v4');sys.path.insert(0,str(h));from measurement import geometry,bank
f=json.loads((h/'fresh_assessment.json').read_text());nav=json.loads((h/'navigation_comparison.json').read_text());v3=h.parent/'oracle_pid_tuning_20260929_v3';gold=json.loads((v3/'navigation_comparison.json').read_text())
fig,axs=plt.subplots(2,2,figsize=(12,7));checks=[]
for col,m in enumerate([.5,-1.5]):
 zs=[]
 for tag,label in [('fresh_base','Original PID'),('fresh_selected','Tuned PID')]:
  p=h/f'{tag}_{m}';z=np.load(p/'physics.npz');zs.append(z);t=z['simulation_time_s'];_,k=geometry(z['position_world_xy']);axs[0,col].plot(t,k*np.sign(m),label=label);axs[1,col].plot(t,np.degrees(bank(z['root_quaternion_world_wxyz'])),label=label)
 keys=['position_world_xy','root_quaternion_world_wxyz','joint_position_rad','joint_velocity_rad_s']
 initial={k:bool(np.array_equal(zs[0][k][0],zs[1][k][0])) for k in keys if k in zs[0]};assert all(initial.values());checks.append(dict(main=m,initial=initial))
 axs[0,col].step([0,5,17,29,41,53],[0,.3,-.6,.5,-.3,-.3],where='post',color='k',linestyle='--',label='Requested curvature');axs[0,col].set_title(f'Main request {m} rad/s');axs[0,col].set_ylabel('Body curvature [1/m]');axs[1,col].set_ylabel('Physical bank [deg]')
 for ax in axs[:,col]:ax.grid();ax.legend();ax.set_xlabel('Simulation time [s]')
fig.tight_layout();fig.savefig(h/'fresh_comparison.png',dpi=150);plt.close(fig);(h/'fresh_initial_check.json').write_text(json.dumps(checks,indent=2))
status=lambda r:'成功' if r['success'] else ('碰撞' if r['collision'] else ('超时' if r['timeout'] else r['termination_reason']))
lines=['# Oracle PID 独立动态与 hard_0 完整验证（2026-09-29）','', '## 结论','',f"冻结 Kp=0.4、Ki=0.1、Kd=0.7，完成四条新动态对照和全部七条 hard_0。独立动态曲率瞬态 RMS {f['baseline_curvature_rms']:.4f} → {f['candidate_curvature_rms']:.4f} 1/m，改善 {100*f['improvement_fraction']:.2f}%；预先约定验证门槛：{'通过' if f['all_pass'] else '未通过'}。",f"hard_0 成功 {sum(r['candidate']['success'] for r in nav)}/7；B0 {sum(r['baseline']['success'] for r in nav)}/7；调参前映射 PID {sum(r['previous_pid']['success'] for r in nav)}/7。",'', 'V3 的 ≥15% 调参门槛仍未通过（仅改善9.29%），tuning_eligible=false 不变。本次是用户明确要求的剩余测试，不能倒改此前拒绝结论，不能据此宣布基线完成。','', '## 冻结设计与口径','','正式规则35秒，成功为首次进入0.2m目标范围，不等于稳定停车。保持参考、规划、速度策略、动作范围及副轴 .30rad/s supervisor 不变。新旧控制仅 PID 增益不同，映射使用同一 V2 冻结模型。该 supervisor 为反馈限制，不能保证实际速度绝无超调。','四条完整53s序列：主轴请求 +0.5、−1.5rad/s 各对照原 PID .8/.1/.35 与候选 .4/.1/.7；前5s直行，随后请求曲率 +.3、−.6、+.5、−.3 各12s。每条完整重置，内部不重置。数据未用于本轮拟合或选参；不是四个独立任务，而是两个动作序列的成对实验，无重复随机种子统计。','曲率来自 XY 轨迹31点居中 Savitzky–Golay 导数，仅用于离线评估，非模型/控制输入；低于 .08m/s 不估计曲率。每次切换后前8秒统计瞬态误差，末5秒至末0.5秒统计稳定尾部。正反向采用原冻结 body-curvature 符号口径。settling 是最后离开曲率误差带的时间，不能解释为 roll 振荡消失时间。','', '## 独立动态结果','','|主轴|控制器|曲率RMS 1/m|bank误差RMS °|尾部通过|峰值bank °|','|---|---|---|---|---|---|']
for key in ['baseline','candidate']:
 for r in f[key]:lines.append(f"|{r['main']}|{key}|{r['curvature_rms']:.4f}|{r['bank_rms_deg']:.3f}|{sum(w['tail_gate'] for w in r['windows'])}/4|{r['max_bank_deg']:.2f}|")
lines+=['','预先冻结门槛：`'+json.dumps(f['checks'],ensure_ascii=False)+'`。','', '|主轴|请求曲率|原尾部误差|新尾部误差|原/新尾部门槛|原/新有效比例|','|---|---|---|---|---|---|']
for b,c in zip(f['baseline'],f['candidate']):
 for x,y in zip(b['windows'],c['windows']):lines.append(f"|{b['main']}|{x['target']}|{x['tail_error']:.4f}|{y['tail_error']:.4f}|{x['tail_gate']}/{y['tail_gate']}|{x['transient_valid_fraction']:.3f}/{y['transient_valid_fraction']:.3f}|")
lines+=['','![独立动态曲率和倾侧角]('+str(h/'fresh_comparison.png')+')','','## hard_0 完整导航','','|案例|B0|调参前PID|本次PID|时间s|终点距离m|地图碰撞|物理接触采样数|','|---|---|---|---|---|---|---|---|']
for r in nav:
 c=r['candidate'];lines.append(f"|{r['episode'].replace('episode_','')}|{status(r['baseline'])}|{status(r['previous_pid'])}|{status(c)}|{c['elapsed_s']:.1f}|{c['final_distance_m']:.3f}|{c['collision']}|{r['candidate_physical_contacts']}|")
lines+=['','![hard_0 轨迹对照]('+str(h/'hard_0_paired_paths.png')+')','','全七条已验证参考与初始位置、姿态、速度、关节及 applied action 相同。曲线误差使用共同参考线段最近距离计算，独立于 policy path_index；该距离不是有方向的真实沿程进度，不能单独作为成功判定。地图碰撞与物理接触分别报告，不能互相替代。','', '## golden 历史结果的复用','','本次未重跑 golden，复用同一冻结 PID 的 V3 实测3/3：E0 20.0s、E1 33.1s、E2 17.6s。来源、源码与结果散列见 golden_reuse_provenance.json。golden E0 与 hard_0 的 cluttered-0_e0_0002 是同一物理任务，不能把两套结果合并成十个独立任务成功率。','', '## 验证与复现','','新动态与导航退出码、原始日志、50Hz反馈、映射记录、physics/states、指标和视频均保留。execution_audit.json 对每个导航 tick 用冻结增益重建 PID 输出；video_verification.json 核查14个新视频首尾可读；navigation_source_identity.json / identity_verified.json 冻结源码身份；asset_verification.json 固定场景、占据图和机器人。fresh_execution_audit.json 另外核验动态冻结源码未变、四条序列共10600个tick的实际PID输出重建完全一致；动态所有瞬态窗口有效率100%，原/新各8个稳态尾部均通过。此前166项控制回归测试属V3历史证据，本轮不冒充重跑。','运行 `reproduce.py --check-only` 可核验源文件/资产；`reproduce.py --suite hard_0 --run-root 新目录` 可复现全套导航。动态由 run_fresh.py 记录命令，需在新实验目录执行，避免覆盖现有数据。所有运行均为真值位置/地图 Oracle，不是视觉基线。','', '## 后续控制工作的边界','','本轮只做冻结控制器验证，不再调参。若困难场景仍失败，优先在失败轨迹上区分目标曲率过大、实际转弯滞后和参考路径不可执行；沿参考推进与碰撞必须共同检查。不能仅靠继续增加D增益或减速宣布解决导航问题。']
report=Path('/home/an/Documents/ChatGPT/球型机器人导航/reports/benchmark_oracle_pid_independent_and_hard0_validation_20260929.md');report.write_text('\n'.join(lines)+'\n');print(report)
