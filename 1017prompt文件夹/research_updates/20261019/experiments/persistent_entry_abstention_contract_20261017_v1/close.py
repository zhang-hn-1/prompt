from replay import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import shutil

def main():
 now=datetime.datetime.now(datetime.timezone.utc).isoformat();r=load(D/'result.json');cause=load(D/'cause_assessment.json');timing=load(D/'source_timing/result.json');assert not r['necessary_passed']
 put(D/'review_receipt.json',dict(utc=now,reviewer='/root/persistent_entry_qualification_review',readonly=True,scope='470exposurestatearrays,1910policycounts,1010Blabelarrays,allteacherblinddenominators,recentfloorproof andsource-timer code',decision='Necessary longterm-only contract fails: E0 exactfixedinput ceiling15.9505%<20%. B/O sameblindmissing allsequences. No new collection/fit.',bounds='These frozeninputs/staticteacher/.15m scoring/originaltimeweights/currentrecentlayeronly. Not general source/otherconsumer/navigation impossibility.',timing='Original source timer wrappers,2descriptive frames;16outputarrays perframe byteexact. No actual speedup/deployment claim.'))
 names=['E0','E1B','E1A'];fig,ax=plt.subplots(1,3,figsize=(14,4.4))
 for i,n in enumerate(names):
  a=cause['attribution'][n];b=r['sequences'][n]['metrics']['B']['false_cell_seconds'];recent=a['mandatory_recent_false_cell_seconds']/b;long=a['B_additional_persistent_false_cell_seconds']/b
  ax[0].bar(i,recent,color='#aa6954',label='Required recent error floor' if i==0 else None);ax[0].bar(i,long,bottom=recent,color='#999999',label='Removable persistent-only' if i==0 else None);ax[0].text(i,1.03,f'{100*long:.2f}% max',ha='center',fontsize=9)
 ax[0].set(xticks=range(3),xticklabels=names,ylim=(0,1.15),ylabel='Fraction of B false cell-seconds',title='Fixed-input longterm-only ceiling');ax[0].legend(fontsize=8,loc='upper center',bbox_to_anchor=(.5,-.12))
 vals=[100*r['sequences'][n]['blind_missing_increase'] for n in names];ax[1].bar(names,vals,color='#518b70');ax[1].axhline(5,color='black',ls='--',label='Maximum allowed +5pp');ax[1].set(ylim=(-.5,6),ylabel='O minus B (percentage points)',title='Same complete blind denominator');ax[1].legend(fontsize=8)
 for i,a in enumerate(timing['rows']):
  ax[2].bar(i,a['dynamic_CAD_masks_s'],color='#c08d45',label='Original dynamic CAD' if i==0 else None);ax[2].bar(i,a['four_stereo_compute_s'],bottom=a['dynamic_CAD_masks_s'],color='#648fb2',label='Four serial SGBM calls' if i==0 else None);ax[2].bar(i,a['other_predict_s'],bottom=a['dynamic_CAD_masks_s']+a['four_stereo_compute_s'],color='#999999')
 ax[2].set(xticks=[0,1],xticklabels=['E1B frame0','E1B frame90'],ylabel='Seconds, source only',title='Unchanged original-source timing');ax[2].legend(fontsize=8,loc='upper center',bbox_to_anchor=(.5,-.12));fig.suptitle('Privileged consumer diagnostic + separate exact-output timing; NO learning/navigation gain');fig.tight_layout();fig.savefig(D/'consumer_limit_and_timing.png',dpi=150);plt.close(fig)
 report=W/'reports/persistent_entry_limit_and_compute_results_20261017.md'
 report.write_text(f'''# 长期入库的可改善上限与独立实现耗时（v112）

## 本轮结论

停止本固定长期入库/边界弃权方案的新采集与拟合。即使已有教师完全正确，只控制长期层、保护全部近期风险的方案，在E0也最多减少15.9505%错误风险格×秒，低于事前20%。因此v111后不能只补训练负例就宣称问题解决。旧二分类数据资格仍拒绝、标签带/完整目标分母/旧结果与预算不变。

| 冻结输入轨迹 | B错误格×秒 | 不可绕过的近期错误格×秒 | 可移除的长期独有错误格×秒 | 理想教师降幅 | B/O盲区漏保留 |
|---|---:|---:|---:|---:|---:|
| E0 |1768.60|1486.50|282.10|15.9505%（失败）|2161/18868，两者相同|
| E1B |14501.30|7332.20|7169.10|49.4376%|634/75700，两者相同|
| E1A输入上的反事实B |25688.50|11140.60|14547.90|56.6320%|708/66215，两者相同|

E1A这一行是同A封存输入的B/O回放，不是实际B导航轨迹。E1A/B仍为同一个开发场景组。E0按原110曝光间隔计时、末曝光dt0；两条E1各900策略点×.1秒，分母不跨轨迹合并。

## 做了什么、为何可以归因

同470个封存活动曝光，原三态标签保持：距离静态教师占用格≤.10m为正，≥.20m为负，中间/域外UNKNOWN。O仅允许正标签在原两曝光条件后长期晋升，负/UNKNOWN分别保存且都不晋升；当前和近期10秒风险全部保留，缺失/未知不生成观测自由。这里直接使用GT标签，是强特权必要消费者诊断，**没有真实RGB学习预测或在线可部署性能**。

先seal470组B/O地图状态，再按110+900+900=1910个原时刻评分；B标签与1010份原B地图逐数组exact。全部当前/近期风险丢失0，三轨迹漏保留差0pp。E1A完整分母66215独立复核。

一次只读归因将每时刻错误拆为“近期已包含”和“仅长期额外包含”。O额外长期错误三轨迹全部0。因此在这些固定输入、静态教师、.15m评分、原时间权重下，任何包含全部近期风险格的输出，其错误格秒至少为1486.50（E0）。最大降幅为1−1486.50/1768.60=15.9505%。这不是所有测距、学习或导航不可能证明；它不约束改变源、近期层或其他场景的结果。不能事后删掉E0救本次全序列门槛。

![本轮完整分母与耗时]({D/'consumer_limit_and_timing.png'})

## 独立工程瓶颈：输出不变的计时

另外登记独立极小工程预算，固定已知E1B frame0/90，各调用原SGBM源一次；只包装CAD与四次compute的计时，参数、数据与运算不改。两帧各4眼×深度/valid/视差/CAD=16数组，dtype/shape/全部原字节与旧封存exact。原始源耗时约3.216/3.122秒，其中动态CAD1.424/1.354秒，四次串行compute1.762/1.739秒。两点只是描述性归因，不是稳定P90、提速证明或完整在线链时延。

基于四次compute相互独立，已另冻结当前工作版仅并行调度四个独立matcher的工程资格；CAD和全部数学保持。先四个固定帧交替串/并行，要求所有输出字节exact、中位时间比≤.8且每帧≤1.1；通过才继续全部470曝光7520数组exact。新预算最多478次原算法调用、fit/物理/导航0，不是换模型/阈值、拒绝源重跑救分。真实状态在 `experiments/fixed_source_parallel_compute_20261017_v1/status.json`；先监督，勿重复派发。批处理进程并行仅加快等价检查，不能拿批处理耗时冒充在线时延。

## 运行与预算

必要诊断09:48:41–09:48:57 UTC，durable parent3907288/child3907304正常0，replay/score子PID在各process.json；dispatch1/1关闭，恢复0/1余1关闭，fit/源重生成/物理/导航0。归因CPU检查正常0，工具session9655，短命辅助PID未单独采到，未冒称该session是宿主PID。

独立原源计时09:54:22–09:54:30 UTC，parent3913204/child3913207正常0；dispatch1/1、源调用2/2关闭，恢复0/1关闭，ML拟合/物理/导航0。两服务inactive、所有相关主要进程结束。独立数组复核与数学上限复核无重要遗留问题。四路新语义、真实学习深度/误差、联合/长期学习BEV与导航N/A；本轮不重复旧RGB/GT图。

## 后续决定

长期-only学习分支关闭，无新增合格记忆/学习候选；不再补采扩库或拟合来尝试打破这个近期层上限。当前推进的是独立的同输出实现耗时资格，先接续它的真实进程和完整数组结果。通过也只证明实现等价和有限开发计时收益；实际四路链、导航安全/到达与学习增量仍需各自证据。任何新的近期风险干预必须有独立机制/完整域/预算，不能恢复旧patch置信过滤、缩TTL或更改旧门槛。
''')
 for f in [D/'ledger.json',D/'source_timing/ledger.json']:
  a=load(f);a.update(status='closed',close_utc=now,unused_recovery_closed=True,active_runtime=False);put(f,a)
 put(D/'cause_audit_receipt.json',dict(complete=True,returncode=0,tool_exec_session_id=9655,host_pid_not_separately_captured=True,source_regeneration=0,fit=0,physics=0))
 manifest=D/'artifact_manifest_post_close.json';put(manifest,dict(utc=now,artifact_sha256={str(f):sha(f) for f in sorted(D.rglob('*')) if f.is_file() and f!=manifest},whole_research_complete=False,formal_triggered=False,fit=0,physics=0))
 entrydir=W/'experiments/navigation_execution_20261011_v1';opt=W/'experiments/fixed_source_parallel_compute_20261017_v1';runtime=load(opt/'status.json')
 entry=dict(version=112,utc=now,status='longterm_only_learning_necessary_contract_closed_recent_floor;independent_sameoutput_compute_qualification_started',prior_entry=str(entrydir/'execution_next_entry_v111.json'),root=str(D),report=str(report),decision=r['decision'],fixed_input_bound=cause['attribution'],oracle_not_online=True,fit=0,physics=0,navigation=0,original_source_timing_invocations=2,old_budgets_reopened=False,artifact_manifest=str(manifest),artifact_manifest_sha256=sha(manifest),active_engineering_root=str(opt),runtime=runtime,next_direct_action='Supervise only alreadyrunning fixed_source_parallel_compute qualification via status/log/workerstatus/PGID; never restart. Fourfixed latency gates mustpass before470 byteequivalence. Any failure closes, no workers/threshold sweep. Original longterm-only learningproposal isclosed; no newdata/fitting, unknown/margins stay. No deploy/physics in optimization budget.',whole_research_complete=False,reliable_navigation=False,learning_increment=False,full3arm_complete=False,automation=dict(id='automation',status='ACTIVE',cadence='hourly'));put(entrydir/'execution_next_entry_v112.json',entry)
 top='> **最新实质接续v112：** 470曝光/1910时刻理想标签长期入库诊断拒绝：E0错误降幅上限15.9505%<20%，84.05%错误属于必须保留的近期层；两E1为49.44%/56.63%，完整盲区漏保留均不变。该长期-only学习分支停止补数据/训练。另原源纯计时2帧16数组/帧字节exact，CAD约1.4s、串行四compute约1.75s；已独立启动同输出并行compute资格，读取v112和真实status监督，勿重复派发。无新增物理/学习拟合，可靠导航/学习增量/整体未完成。\n\n'
 plan=W/'docs/superpowers/plans/2026-10-17-navigation-memory-task-list.md';text=plan.read_text();pos=text.index('\n\n')+2;plan.write_text(text[:pos]+top+text[pos:]+'''\n## v112：先检查可改善上限，再决定数据与学习\n\n- [x] 完整470曝光/1910时刻理想教师长期入库必要检查与一次近期/长期归因；E0上限15.95%不足，停止本分支数据/拟合，原v111不改。\n- [x] 原四路源输出不变计时两帧，各16数组字节exact，定位CAD与四次串行compute成本。\n- [ ] 接续已启动同输出四compute并行工程资格：四帧固定计时门槛后全470输出等价，不修改匹配器/阈值/相机/源语义；无物理/导航准入。\n\n当前v112；学习数据方案原待办现被近期层上限阻断，勿机械补数据。\n''')
 mission=W/'research_mission.md';mission.write_text(top+mission.read_text())
 statepath=entrydir/'automation_execution_state.json';s=load(statepath);s.update(updated_at=now,latest_scientific_entry=str(entrydir/'execution_next_entry_v112.json'),latest_scientific_result_unchanged=False,current_failure_analysis='Longterm-only gain bound15.95% under20pct E0 gate; more data/fitter cannot change protectedrecentfloor. Learningbranchclosed. Independent sameoutputCPUoptimization remains underqualification.',proposed_learning_target=None,runtime=runtime);put(statepath,s)
 pkg=W/'资料_球型机器人导航_2026-09-30/2026-10-17_四相机弱基线与记忆检查'
 for f in [report,plan,D/'consumer_limit_and_timing.png',entrydir/'execution_next_entry_v112.json']:shutil.copy2(f,pkg/f.name)
 dest=pkg/'长期入库上限与原源计时';shutil.copytree(D,dest)
 index=pkg/'源码与原始数据索引.md';index.write_text(index.read_text()+f'\n## v112\n\n- 必要消费者/近期错误上限、原源计时：{D}\n- 已启动同输出工程资格：{opt}，按status监督，不重复启动。\n- 本包新增“长期入库上限与原源计时”完整复制该诊断根；GToracle不是在线模型。\n')
 sums=pkg/'SHA256SUMS.txt';sums.write_text(''.join(f'{sha(f)}  {f.relative_to(pkg)}\n' for f in sorted(pkg.rglob('*')) if f.is_file() and f!=sums));print('v112diagnosticsealed',len(load(manifest)['artifact_sha256']),sha(manifest),'optimizerstatus',runtime['status'])
if __name__=='__main__':main()
