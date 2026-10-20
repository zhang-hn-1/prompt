from pathlib import Path
import json,hashlib,datetime,shutil,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;W=D.parents[1];O=W/'experiments/fixed_source_parallel_compute_20261017_v1';E=W/'experiments/navigation_execution_20261011_v1';Q=W/'experiments/persistent_entry_abstention_contract_20261017_v1'
load=lambda p:json.loads(Path(p).read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def main():
 now=datetime.datetime.now(datetime.timezone.utc).isoformat();sr=load(O/'status.json');cr=load(D/'status.json');r=load(O/'result.json');c=load(D/'result.json')
 assert sr['status']==cr['status']=='finished' and sr['child_returncode']==cr['child_returncode']==0 and not sr['remaining_live_group_pids'] and not cr['remaining_live_group_pids']
 assert r['output_arrays_byte_exact']==7520 and c['mask_arrays_byte_exact']==1880 and c['full_source_invocations']==8
 for root in [O,D]:
  cfg=load(root/'protocol.json')
  for k in ['code_sha256','input_sha256']:
   for p,h in cfg[k].items():assert sha(p)==h,p
  ledger=load(root/'ledger.json');ledger.update(status='closed_qualification_passed_no_online_claim',close_utc=now,unused_recovery_closed=True,active_runtime=False);put(root/'ledger.json',ledger)
 oldrows=load(W/'experiments/persistent_entry_data_qualification_20261017_v1/feature_seal.json')['rows'];expected={(v['sequence'],v['frame']):v for v in oldrows};assert len(expected)==470
 for row in r['rows']:
  key=(row['sequence'],row['frame']);ref=expected[key];assert row['raw_sha256']==ref['input_sha256'] and row['reference_sha256']==ref['prediction_sha256'] and row['arrays_exact']==16
  assert sha(ref['input_rgb_frame'])==ref['input_sha256'] and sha(ref['prediction_path'])==ref['prediction_sha256']
 for row in c['rows']:assert row['masks_byte_exact']==4 and (row['sequence'],row['frame']) in expected
 lb=load(O/'latency_result.json');lc=load(D/'latency_result.json');ratio1=float(np.median([x['ratio'] for x in lb['rows']]));ratio2=float(np.median([x['ratio'] for x in lc['rows']]))
 # Offline entry export only. Archived controllers/physics/source roots remain untouched.
 entrymodule=D/'source_entry.py';entrymodule.write_text('"""Offline-qualified same-output frontend; no automatic controller integration."""\nfrom cached_source import CachedBaseParallelSGBM as FourSGBM, legacy\npoints=legacy.points\nNAMES=legacy.NAMES\nRAYS=legacy.RAYS\nAX=legacy.AX\nF=legacy.F\nB=legacy.B\nLOCAL=legacy.LOCAL\nCONFIG=legacy.CONFIG\n')
 compile(entrymodule.read_text(),str(entrymodule),'exec')
 import importlib.util
 sys.path.insert(0,str(D));spec=importlib.util.spec_from_file_location('qualified_source_entry',entrymodule);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 assert module.FourSGBM.__name__=='CachedBaseParallelSGBM' and callable(module.points) and abs(module.F-294.77087816197377)<1e-12 and abs(module.B-.65012)<1e-12
 put(D/'entry_interface_check.json',dict(complete=True,entry_import_pass=True,source_class_andpoints_export_pass=True,unchanged_F_B=True,source_invocations=0,model_construction=0,physics=0))
 put(D/'source_entry_admission.json',dict(utc=now,source_entry=str(entrymodule),source_entry_sha256=sha(entrymodule),qualification_scope='Offline frontend on470fixeddevelopmentinputs; physics/onlineintegration not run',front_end_arrays='Parallel stage470x16 runtimebyteasserts with references to original outputs, cache stage470x4mask runtimebyteasserts plus8fullforwards at4frames. No claim cache full470fullforward empiricallyrerun.',original_baseline_unchanged=True,known_CAD_only=True,GT_online=False,learning_gain=False,navigation_gain=False,full_chain_latency_verified=False))
 fig,ax=plt.subplots(1,2,figsize=(11,4.1));x=np.arange(4);width=.36
 for a,data,left,right,title in [(ax[0],lb['rows'],'serial','parallel','Independent compute scheduling trial'),(ax[1],lc['rows'],'uncached','cached','Independent base-CAD cache trial')]:
  a.bar(x-width/2,[v[left] for v in data],width,label=left,color='#999999');a.bar(x+width/2,[v[right] for v in data],width,label=right,color='#4b8c70');a.set(xticks=x,xticklabels=[f'{v["sequence"]}\nframe{v["frame"]}' for v in data],ylabel='Seconds, source only',title=title);a.legend()
 fig.suptitle('Fixed known frames; identical output bytes — NOT end-to-end/navigation latency');fig.tight_layout();fig.savefig(D/'same_output_source_timing.png',dpi=160);plt.close(fig)
 report=W/'reports/fixed_four_camera_same_output_acceleration_20261017.md'
 def rng(rows,key):return f'{min(v[key] for v in rows):.3f}–{max(v[key] for v in rows):.3f}'
 report.write_text(f'''# 固定四路测距源：保持输出不变的实现提速（v113）

## 完成事项

两个独立单因素工程对照均通过冻结门槛。只改变计算调度和已知CAD中静态部分的重复工作；匹配器、阈值、输入、相机、语义/高度代理、位姿、记忆、规划和控制未改。没有新采集、神经训练、神经推理或物理导航。

| 对照 | 固定四帧原实现 | 固定四帧工作版 | 成对时间比中位数 | 完整输出资格 |
|---|---:|---:|---:|---|
| 四个独立SGBM调用并行 | {rng(lb['rows'],'serial')}s | {rng(lb['rows'],'parallel')}s | {ratio1:.4f} | 470曝光×16数组=7520逐字节exact |
| 固定四路静态base CAD缓存 | {rng(lc['rows'],'uncached')}s | {rng(lc['rows'],'cached')}s | {ratio2:.4f} | 470曝光×4CAD=1880逐字节exact；4帧两版本共8次完整源调用exact |

固定帧预先选择E0 0/55、E1B 0、E1A 90；各一次，串/并或未缓存/缓存顺序交替。两组是分别实施的实验，不能把不同组时间相减当成同次直接测得总收益。每组中位成对比≤.8、各帧比≤1.1，全部满足。缓存初始化另测：未缓存并行{lc['uncached_initialization_s']:.3f}s、缓存{lc['cached_initialization_s']:.3f}s；表中steady调用不含该一次成本。

![两个独立同输出计时对照]({D/'same_output_source_timing.png'})

最终源四个开发样本仍约1秒，高于旧.5秒要求；新弱baseline未据此宣称旧严格门槛通过。没有运行新的完整控制/渲染链时延或导航，不能把源耗时当端到端延迟、到达/碰撞或学习收益。测距错误与覆盖没有改善，原错误风险保留结论不变。

## 实现与证据范围

第一项：四个独立native matcher分别执行原四次compute；每个CV仍单线程，输入数组由Future持有，输出全部返回后才结束predict。动态CAD、正负视差换算和后处理沿用原代码。四离线进程仅加快完整资格吞吐；该批处理下的逐帧时间不当作在线性能。完整470域为E0 110、E1B 180、E1A 180，都是已见开发数据，非fresh。

第二项：复用此前已存在的静态base缓存思想，适配用户固定四路。base_link相对机体变换恒为I、相机随同机体固定；仅缓存此部分。三动态link每次仍使用实际关节。原逐三角形布尔并集可拆为静/动态OR，浮点边界由完整1880掩码exact检验。不是新测量/学习机制，也没有使用GT场景。原两路缓存的光学/膨胀/frozen mask未移入四路。

第一项真实重算478次原SGBM（4串行＋4并行开发计时＋470并行全域），运行时逐一比较dtype/shape/原始bytes；相同内容直接引用旧预测归档，未另存重复候选数组。独立审查核对源码、执行记录与全部引用，未重推理。第二项真实完整源8次，另470次仅mask检查；没有把后者说成470次完整源重算。结合已通过的下游相同运算，给出冻结输入域的组合等价依据，不冒称实测了缓存版470份完整新预测。

工作版入口 {entrymodule}，提供FourSGBM及原points/常量；只有离线前端资格。未改已封存运行源码、当前默认在线入口或原项目，未自动Git提交/上传。源码、配置、协议/输入/旧数组SHA及114份原身份快照在两个实验根，跨机器实际运行未做。

## 监督、预算与审查

并行资格{sr['start_utc']}–{sr['end_utc']}正常0，parent{sr['parent_pid']}/child{sr['child_pid']}，四worker全部结束；dispatch1/1、源478/478、恢复0/1余1关闭。缓存资格{cr['start_utc']}–{cr['end_utc']}正常0，parent{cr['parent_pid']}/child{cr['child_pid']}；dispatch1/1、完整源8/8、完整mask470/470，恢复0/1余1关闭。两项fit/神经推理/物理/导航均0，旧预算未重开。

派发前审查发现原直接子进程监督无法充分清理四个spawn后代，已仅在当前工作版修为独立PGID、TERM/KILL整组、退出确认；旧共享helper不改。正常/带孙进程超时两个接口fixture通过，强制KILL分支仅源码检查未实跑。实际两组结束均无存活或僵尸组成员。冻结前修订字节另存，不把派发前工程修复算作科学恢复。

## 与本轮学习结论及下一动作

v112已证明：当前/近期风险必须全保留时，E0错误格秒最多下降15.95%，达不到本长期入库方案冻结的20%。停止该方案补采/拟合；不改旧标签带、门槛或删E0救分。该结论只限既定消费者，不是所有学习不可能。报告 {W/'reports/persistent_entry_limit_and_compute_results_20261017.md'}。

这次实现提速没有解决源误报或提供新学习增量；当前没有准入的学习/记忆干预。后续先复用v109已封存的错误出生分解，有限检查“近期错误的可区分输入与完整监督目标”是否能形成实质不同的消费者，尤其区分无近处表面支持、底层无效和保守UNKNOWN；不把无GT hit当在线自由，也不重做出生评分。最多核对旧point-height风险头和旧static-cell风险头两项直接合同；若只是旧视觉过滤/网络换名，则不立拟合预算并明确学习停滞。暂不实际重跑A/B、不新采集、不修改原控制/近期风险或继续提速扫参。可靠导航、完整三臂与学习增量仍未完成。
''')
 # Seal completed implementation roots after all final reports/checks are in place.
 for root in [O,D]:
  m=root/'artifact_manifest_post_close.json';assert not m.exists();put(m,dict(utc=now,artifact_sha256={str(p):sha(p) for p in sorted(root.rglob('*')) if p.is_file() and p!=m},purpose='Sameoutput implementation qualification only',whole_research_complete=False,formal_triggered=False,fit=0,physics=0))
 nextaction='Source acceleration qualified offline; no current actualrun or navadmission. Longterm-only learningproposal closed by E0recenterrorfloor. Next only bounded NEW target/consumer prerequisite study anchored to existing v109 error-birth evidence, not rerun: can currentRGB/disparity/provenance distinguish unsupportednear-surface risk from invalid/unknown without treatingnoGThit asfree? Compare at most oldpoint-height andstatic-cell riskheads directly; if merelyrenamedfilter or no completecausallabel/input contract, recordlearningbottleneck and do notfit/collect. Keepsource/pose/planner/controller andoldbudgets; no actualA/Brerun, TTL/threshold/worker sweep.'
 entry=dict(version=113,utc=now,status='same_output_source_engineering_complete;learning_route_unqualified',prior_entry=str(E/'execution_next_entry_v112.json'),roots=[str(O),str(D)],report=str(report),parallel_full_exposures=470,parallel_output_arrays_exact=7520,cache_full_exposures=470,cache_mask_arrays_exact=1880,parallel_paired_ratio_median=ratio1,cache_paired_ratio_median=ratio2,cache_source_4frame_range_s=[min(v['cached'] for v in lc['rows']),max(v['cached'] for v in lc['rows'])],source_total_this_heartbeat=488,neural_fit=0,neural_inference=0,physics=0,navigation=0,runtime=None,online_default_changed=False,whole_research_complete=False,reliable_navigation=False,learning_increment=False,full3arm_complete=False,manifests={str(root/'artifact_manifest_post_close.json'):sha(root/'artifact_manifest_post_close.json') for root in [O,D]},source_entry=str(entrymodule),next_direct_action=nextaction,automation=dict(id='automation',status='ACTIVE',cadence='hourly'));put(E/'execution_next_entry_v113.json',entry)
 top='> **最新完成v113：** 原四路SGBM同输出工程资格完成：并行compute全470曝光7520数组exact，固定四帧时间比中位.563；静态base缓存全470曝光1880掩码exact、四帧完整源输出exact，时间比中位约.705，最终源约1.05–1.07秒。非完整在线链/导航或学习收益，原源错误不变。v112长期-only方案因近期错误上限关闭，仍无准入学习候选。全部进程结束；下一仅有限核对一个针对近期误报的新监督目标/消费者的输入与旧头区别，不重做出生诊断/旧过滤或重跑A/B，不采集/训练。整体未完成，automation每小时ACTIVE。\n\n'
 plan=W/'docs/superpowers/plans/2026-10-17-navigation-memory-task-list.md';s=plan.read_text();pos=s.index('\n\n')+2;s=s[:pos]+top+s[pos:];s=s.replace('- [ ] 接续已启动同输出四compute并行工程资格：','- [x] 已完成同输出四compute并行工程资格：');s+='''\n## v113：同输出实现提速完成，学习仍无准入\n\n- [x] 四compute并行单因素，470曝光7520数组exact；四固定帧成对比中位.5631。\n- [x] 固定四路静态base缓存单因素，470掩码1880数组exact＋四固定帧完整源exact，最终调用约1.05–1.07s；不冒称全链时延/导航改善。\n- [x] 源码/114快照/完整调用/PGID清理与资料同步；原实验和项目不改。\n- [ ] 仅有限核对一个针对近期误报的完整监督目标/消费者，复用既有出生分解，与旧两类风险头直接区别；没有实质区别/数据资格则明确停滞，不拟合。\n\n当前 experiments/navigation_execution_20261011_v1/execution_next_entry_v113.json。不得继续v111补数据、v112长期-only拟合或默认重跑A/B；本版本没有在线默认切换。\n''';plan.write_text(s)
 mission=W/'research_mission.md';mission.write_text(top+mission.read_text());statepath=E/'automation_execution_state.json';state=load(statepath);state.update(updated_at=now,current_research_state='sameoutput_source_optimization_complete_learning_unqualified',latest_scientific_entry=str(E/'execution_next_entry_v113.json'),latest_scientific_result_unchanged=False,last_operation='470privilegedconsumer/1910timesteps rejectedlongterm-only byrecentfloor; sameoutputparallel470sourceandstaticcache470mask qualifications finished0',cause='Immediate/recent false-risk floor, not missing trainingexamples alone; sourcecompute slow but sameoutputengineering reduces sampledtime. Accuracy and learningunresolved.',runtime=None,runtime_service=None,runtime_status_path=str(D/'status.json'),latest_research_round_completed=True,new_scientific_budget_opened=False,new_scientific_candidate=None,next_action=nextaction,proposed_learning_target=None);state['recent_runs']=(state.get('recent_runs',[])+[dict(utc=now,work='Perfectteacher longterm-entry necessarycheck rejected; two singlefactor sameoutput sourcecompute/cache implementationsqualified',evidence=[str(Q/'result.json'),str(Q/'cause_assessment.json'),str(O/'result.json'),str(D/'result.json')],result='E0 gainceiling15.95%<20; full470output andmaskidentity maintained;4frame sourceapprox1.06s, noML/navgain',runtime='none',next_action=nextaction)])[-10:];put(statepath,state)
 pkg=W/'资料_球型机器人导航_2026-09-30/2026-10-17_四相机弱基线与记忆检查'
 for f in [report,D/'same_output_source_timing.png',E/'execution_next_entry_v113.json',plan]:shutil.copy2(f,pkg/f.name)
 for root,name in [(O,'同输出并行匹配资格'),(D,'同输出静态机体缓存资格')]:
  dest=pkg/name;assert not dest.exists();shutil.copytree(root,dest)
 index=pkg/'源码与原始数据索引.md';index.write_text(index.read_text()+f'\n## v113\n\n- 同输出四路compute资格根：{O}\n- 同输出静态base缓存资格根：{D}\n- 完整结果/代码/114快照/引用原数组SHA已复制到本包两新目录；旧完整源数组继续canonical单份引用。无新物理/GT预测/ML。\n- 最新接续 {E/"execution_next_entry_v113.json"}，全链时延/可靠导航/学习收益未成立。\n')
 sums=pkg/'SHA256SUMS.txt';sums.write_text(''.join(f'{sha(p)}  {p.relative_to(pkg)}\n' for p in sorted(pkg.rglob('*')) if p.is_file() and p!=sums))
 print('v113complete',entry['cache_source_4frame_range_s'],'paired ratios',ratio1,ratio2,'roots sealed and package copied')
if __name__=='__main__':main()
