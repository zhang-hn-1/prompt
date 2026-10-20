from pathlib import Path
import json,hashlib,datetime
W=Path('/home/an/Documents/ChatGPT/球型机器人导航');H=W/'experiments/stereo_memory_navigation_20261004_v1';now=datetime.datetime.now(datetime.timezone.utc).isoformat();rows=[]
for n in ('e0','e1','e2'):
 D=H/(f'sgbm_baseline35_{n}_coverage_run_v1' if n!='e2' else 'sgbm_baseline35_e2_run_v1');r=json.loads((D/'sgbm_composition_assessment_v1.json').read_text());rows.append({k:r[k] for k in ('condition','outcome','goal_remaining_m','along_start_goal_axis_m','handoff_time_s','tracking_request_count','cycle_wall_s','videos') } if 'condition' in r else dict(condition=n,**{k:r[k] for k in ('outcome','goal_remaining_m','along_start_goal_axis_m','handoff_time_s','tracking_request_count','cycle_wall_s','videos')}))
(H/'sgbm_baseline35_three_condition_results_v1.json').write_text(json.dumps(dict(utc=now,actual_results=rows,arrivals=0,total=3,scope='same frozen SGBM conditional knownpose TaskH; original35 time rule retained but not original strictRGB benchmark qualification',old_rejections_preserved=True),indent=2)+'\n')
text='\n\n## '+now+'：固定传统基线覆盖完成，接续估计位姿闭环\n\n'
text+='SGBM 同一配置的 E0/E1/E2 均正常完成实际仿真及视频导出，35秒实际到达为0/3（有平坦地面/安全起步区及已知位姿的 TaskH；不是原严格RGB导航成功率）。E0 剩余目标距离3.53285米、起终点轴进度5.30922米；E1 剩余5.89099米、轴进度0.71352米；E2 剩余8.11806米、轴进度0.78560米。均没有真实障碍接触，但E1/E2出现回退，因此没有碰撞不等于可靠导航。作者实际查看 E0/E1/E2 轨迹图；评估脚本解码完整左右拼接及第三视角视频，作者没有声称逐帧看完全部视频。原始来源、控制、正式视觉准入拒绝均保留。\n\n'
text+='已直接启动唯一 E0 位姿来源替换实验（session16721），注册及导航共同消费当前 native IMU/encoder 估计，初始 GT gauge、平坦高度先验和低层PID理想IMU明确保留。SGBM、相机、光照、地图、参考机制、增益及速度不改；最多1条、35秒、无重试。未来/过期/无效时间及未可用估计都拒绝，无真值回退。TDD四项缺失失败→通过，再独立审查发现NaN时钟漏洞，新增测试失败后修复；最终针对性27项通过。全实验目录裸pytest尝试有26项收集错误，主要第三方隔离包缺依赖、历史重复模块及pxr动态库；完整名称和原因保存于 native_pose_baseline_full_cpu_suite_v1.log，不声称全套通过。独立审查最终Critical0/Important0。\n\n'
text+='下一接续：检查 native_pose_baseline35_e0_status_v1.json/进程，不重复启动；正常关闭后执行 assess_native_pose_baseline35_e0_closed_v1.py --condition e0，核验原始封存、真实到达/接触、轨迹、两视角视频、来源/注册/导航身份和位姿误差。若未到达保留失败；不把小位姿误差作为导航成功，不要求旧95%来源准确率准入。本轮仍未完成学习记忆增量及原严格RGB/原35秒正式整体交付。\n'
files=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md',W/'reports/benchmark_baseline35_navigation_priority_20261009.md',W/'资料_球型机器人导航_2026-09-30/2026-10-09_研究进展更新/传统视觉基线导航评测优先.md'];receipts=[]
for p in files:
 old=p.read_bytes();p.write_bytes(old+text.encode());new=p.read_bytes();assert new[:len(old)]==old;receipts.append(dict(path=str(p),old_bytes=len(old),old_sha256=hashlib.sha256(old).hexdigest(),new_sha256=hashlib.sha256(new).hexdigest(),old_prefix_preserved=True))
(H/'baseline35_native_pose_started_append_receipts_v1.json').write_text(json.dumps(receipts,indent=2)+'\n')
(H/'baseline35_native_pose_continuation_started_v1.json').write_text(json.dumps(dict(utc=now,session_id=16721,status='native_pose_E0_actual_running',next='check native status/process; afterclose run assessor once; then decide next incomplete actual layer from measured results',no_user_blocker=True,research_complete=False,old_results_immutable=True),indent=2)+'\n')
print('saved three-condition results, exact-prefix receipts and running continuation')
