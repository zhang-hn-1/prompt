"""Archive the completed fit0 qualification, without changing sealed source or labels."""
from common import *
from datetime import datetime,timezone
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    now=datetime.now(timezone.utc).isoformat();r=load(D/'qualification_result.json');audit=load(D/'author_audit.json');repair=load(D/'interface_repair_result.json');assert not r['qualified']
    review=dict(utc=now,reviewer='/root/persistent_entry_qualification_review',readonly=True,scope='All470feature/labelSHA, finite24arrays, exact290Bmaps, observedtimestamporder, original/repairedsingleedit, teachernearestKDtree and classes/groups.',conclusion='NOT_QUALIFIED_BEFORE_FIT remains. No important issue after isolated repair. All470nearestteacher distances independently recomputed exactly.',original_interface_issue='Newframe/sametime could count twice; observed strict sequence prevented occurrence. Original code/result preserved. Repaired workversion rejects before payload/state.',prediction_regeneration=0,fit=0,physics=0)
    write(D/'independent_review.json',review)
    original=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
    teacher_sources=[original/'scripts/generate_benchmark_tasks.py',original/'scripts/bev_ground_truth.py']
    write(D/'teacher_provenance_followup.json',dict(utc=now,reads={str(p):sha(p) for p in teacher_sources},scope='Only canonical teacher metadata/export/rasterization direct chain, no scene launch or new labels.',evidence='bodyband is vertical maximum of analytic collider Cylinder/Sphere footprints over robot height, rasterized by cell centers at.1m. Existing distances already recomputed exactly.',cause='All eligible UNKNOWN is intentionally .10-.20m label margin, not missing image, teacher-domain hole or failed nearest-neighbor query. Reading more geometry cannot change this frozen distance-band definition.',decision='No margin narrowing, class relabeling or role swapping; current data protocol remains failed. Need a separately frozen data/teacher design before any fit. Coarse2D labels are not exact render meshes or collision safety.',new_labels=0,fit=0,physics=0))
    fig,ax=plt.subplots(1,2,figsize=(10,4.2))
    for i,(g,n) in enumerate([('E0',1556),('E1',35708)]):
        q=r['groups'][g];vals=[q['eligible_positive'],q['eligible_negative'],q['eligible_unknown']];ax[0].bar(i,vals[0]/n,color='#498f52',label='Positive' if i==0 else None);ax[0].bar(i,vals[1]/n,bottom=vals[0]/n,color='#ba654e',label='Negative' if i==0 else None);ax[0].bar(i,vals[2]/n,bottom=sum(vals[:2])/n,color='#999999',label='Unknown margin' if i==0 else None)
        ax[0].text(i,1.025,f'{sum(vals[:2])/n:.2%} known\nn={n}',ha='center',fontsize=9)
    ax[0].set(xticks=[0,1],xticklabels=['E0','E1 A+B (one group)'],ylim=(0,1.18),ylabel='Fraction of persistent-entry candidates',title='All eligible instances retained');ax[0].legend(loc='upper center',bbox_to_anchor=(.5,-.1),ncol=3,fontsize=8)
    ax[1].bar([0,1],[18,16],color='#ba654e');ax[1].scatter([0,1],[50,20],marker='_',s=500,c='black',label='Frozen minimum');ax[1].set(xticks=[0,1],xticklabels=['Negative instances','Distinct negative cells'],ylim=(0,58),title='E0 decision-domain shortage');ax[1].legend();fig.suptitle('DATA QUALIFICATION ONLY — no fitted model or navigation');fig.tight_layout();fig.savefig(D/'qualification_coverage.png',dpi=160);plt.close(fig)
    report=W/'reports/persistent_entry_data_qualification_results_20261017.md'
    report.write_text(f'''# 历史风险持久入库：数据资格结果（v111）

## 结果与问题解释

本阶段已完成，结论 **NOT_QUALIFIED_BEFORE_FIT**。不是训练后模型失败：本轮拟合、学习推理、源预测重生成、物理、导航均0。完整三臂实际导航、学习收益与可靠导航仍未完成。

| 整序列组 | 全部当前风险格实例 | 达到持久入库条件 | 正例 | 负例 | UNKNOWN | 已知标签比例 |
|---|---:|---:|---:|---:|---:|---:|
| E0（110曝光） |2038|1556|1466|18|72|95.37%|
| E1 A+B（360曝光，共一个场景组） |38014|35708|25402|4220|6086|82.96%|

表中正/负/UNKNOWN均以“达到持久入库条件”为分母；重复时刻不是独立样本。E0要求负例≥50、不同负例物理格≥20，实际18和16；全部156个单帧负例中138个尚未达到原两曝光入库条件，不能挪入学习目标凑数。E1要求已知标签≥90%，实际82.956%。6086个UNKNOWN全部来自10–20厘米教师留白带，域外0；不是缺RGB或执行故障。原冻结正例≤.10m、负例≥.20m不调整，原导航评价.15m不变。

![新数据资格分母]({D/'qualification_coverage.png'})

## 实际消费者与历史区别

只拟让未来冻结的学习分数控制“两曝光之后进入长期层”，原当前及近期10秒风险全部保留；未知分数不晋升，不产生观测自由。全接纳/全拒绝/NaN只是接口fixture，没有训练过的预测。预先声明.95为未来晋升阈值，不是已校准可信度。

旧静态格风险学习、RGB/历史特征都已存在；此处结构区别只在长期入库这一干预位置。旧learned-cell head、按原像素GT高度标注的point logistic、被拒绝的当前点patch过滤和视点支持C均未加载/重跑。不是首次发明风险学习，也没有复活旧模型。

## 输入与教师隔离

完整470曝光：E0 110，E1B 180，E1A 180；两条E1末帧无实际策略决策，不进入本学习消费者域。原始源点2985832、出400×400消费者栅格0。所有40052当前风险格实例构造24个有限特征，37264实例达到入库条件。RGB、视差、估计高度、双目像素残差、CAD边缘距离及当前/过去来源/统计均来自原封存输入；世界位置/场景ID不是模型特征。近期统计窗10秒，生命周期曝光计数仍保留更早信息，不称严格10秒记忆或独立确认概率。

特征470档先封存，再由独立进程读原完整静态bodyband栅格生成470标签档。标签判断估计地图位置相对静态障碍格，不用“原像素是障碍”替代定位正确。无GT深度无命中像素不被当作在线自由。教师为.1m解析几何高度带投影，非精确渲染曲面/三维安全证书。原teacher最近距离全部独立复算，最大差0m。

E0/E1都是反复查看的开发数据；E1A/B同一场景组，不是相互独立留出。不随机拆相邻帧，没有新留出、校准、泛化或学习导航收益。

## 接口检查与一次修复

原全接纳mock与290份封存B地图逐格exact（E0 110、E1B 180），只是工程等价。独立审查发现单独接口允许新frame复用同timestamp，可能把同一曝光计数两次；数据读取层已要求严格递增，因此本次470曝光及原等价结果不受影响。

保留原consumer.py，单独consumer_repaired.py仅补“新frame必须有更大timestamp”，在访问payload及状态修改前拒绝。一次恢复已复现旧漏洞、验证新接口拒绝且状态exact，以及当前/近期保护、重复曝光、未来输入、未知不自由fixture。原特征/标签/结论未重生成。独立修复复核无重要遗留问题；未把接口通过当模型性能。

## 运行、预算与证据

主资格2026-10-17 08:45:23–08:46:28 UTC正常0，durable parent3842152/child3842171均退出；service inactive/MainPID0/ExecMainStatus0。dispatch1/1、恢复1/1关闭；fit0/0、源重生成0、物理0、导航0。恢复仅CPU接口fixture；原结果正常0不是运行失败。

原协议/源码/输入/旧参考SHA与470对输出均复核。author_audit.json及independent_review.json保留详细计数和边界。当前四RGB新预测、深度误差、每眼/联合/长期BEV、导航：N/A（本轮没有拟合模型，不重复旧图）。

## 关闭决定与下一动作

本固定数据资格分支在拟合前关闭；不删UNKNOWN、放宽标签带、把一次负例变成持久负例、交换旧角色或给同协议追加拟合。本结果不证明该学习思想不可行。

下一先冻结一份**新的整序列数据与教师资格方案**，明确如何获得跨场景持久负例、怎样保留边界UNKNOWN，以及训练/开发/真正留出的完整分组；现有E0/E1继续保留为已见诊断/压力域，不能冒称新留出或悄悄改写本次角色。须说明新证据为何解决上述两种缺口，并给出完整域验收和总采集/拟合预算；若不能成立，明确学习部分停滞，不采集、不训练、不恢复旧C/源模型。当前未登记新采集或拟合许可，不声称正在运行。
''')
    ledger=load(D/'ledger.json');ledger.update(status='closed_before_fit_data_not_qualified',recovery_used=1,recovery_scope='Single isolated capture-clock interface repair fixtures; original immutable outputs unchanged',unused_recovery_closed=True,child_pid=3842171,active_runtime=False,close_utc=now,author_checks_returncode=0);write(D/'ledger.json',ledger)
    entrydir=W/'experiments/navigation_execution_20261011_v1';next_action='First design and freeze a NEW whole-sequence data/teacher qualification contract: persistent-negative support across scenes, explicit boundary UNKNOWN and genuine train/dev/fresh-heldout groups. Existing E0/E1 remain seen stress diagnostics and this failed protocol is immutable. Explain how new evidence addresses E0 eligible negatives18/distinct16 and E1 deliberate marginUNKNOWN17.04%; do not narrow/relabel/drop margins, swap oldroles, oversample first-hit negatives or call duplicates independent. Freeze total collection/fit resources and complete-domain gates before dispatch. No new acquisition/fit yet. If no justified design, record learning-data bottleneck rather than retrying oldclosedC/source/control or literaturechain.'
    entry=dict(version=111,utc=now,status='persistent_entry_data_qualification_complete_rejected_before_fit',prior_entry=str(entrydir/'execution_next_entry_v110.json'),root=str(D),report=str(report),result=str(D/'qualification_result.json'),result_sha256=sha(D/'qualification_result.json'),groups=r['groups'],sequences=r['sequences'],original_source_points=2985832,feature_archives=470,label_archives=470,all_admit_B_maps_exact=290,original_interface_issue_preserved=True,repaired_work_version=str(D/'consumer_repaired.py'),teacher_followup=str(D/'teacher_provenance_followup.json'),review=str(D/'independent_review.json'),budget=ledger,runtime=None,fit=0,learned_inference=0,source_regeneration=0,physics=0,navigation=0,full3arm_complete=False,reliable_navigation=False,learning_increment=False,whole_research_complete=False,next_direct_action=next_action,automation=dict(id='automation',status='ACTIVE',cadence='hourly'),old_seals_unchanged=True)
    write(entrydir/'execution_next_entry_v111.json',entry)
    top='> **最新完成v111：** 历史风险持久入库的数据资格完成但未通过：470曝光/2985832源点，E0持久负例18、不同格16不足；E1已知标签82.956%不足，全部6086未知来自固定10–20cm留白。拟合/源重生成/物理/导航0。独立接口重复timestamp计数漏洞单独修复，原470数据不变。当前无运行；下一先冻结新的整序列数据/教师资格设计，未解决缺口不采集/拟合，不改标签带或恢复旧C。详见v111；学习/可靠导航/整体未完成。\n\n'
    plan=W/'docs/superpowers/plans/2026-10-17-navigation-memory-task-list.md';text=plan.read_text();lines=text.splitlines(True);lines.insert(2,top);text=''.join(lines).replace('- [ ] 下一资格工作：一个监督历史风险持久入库可信度目标；','- [x] 已完成且拟合前拒绝的数据资格：一个监督历史风险持久入库可信度目标；')
    text+='\n## v111：持久入库数据资格关闭，未拟合\n\n- [x] 470完整曝光特征先seal，再完整教师标签；290全接纳B地图exact。E1A/B为同一开发组，无fresh留出。\n- [x] 一次有界归因：E0 138/156负例尚未达到入库条件；E1全部6086eligible UNKNOWN来自固定标签留白，非缺图。原合同不调。\n- [x] 同时间重编号曝光接口漏洞仅单独工作版本修复；470输出/旧源/原结果不变。\n- [ ] 设计新的整序列数据/教师资格方案，先说明跨场景持久负例与边界unknown怎样处理、总预算和停止门槛；现有数据不冒称新留出。尚未准入新采集/拟合。\n\n当前接续 experiments/navigation_execution_20261011_v1/execution_next_entry_v111.json；报告 reports/persistent_entry_data_qualification_results_20261017.md。P4学习收益框仍未勾选。\n'
    text=text.replace('**下一具体任务：读取v110。**','**历史下一任务（现由v111完成覆盖）：读取v110。**');plan.write_text(text)
    mission=W/'research_mission.md';mission.write_text(top+mission.read_text())
    statepath=entrydir/'automation_execution_state.json';s=load(statepath);s.update(updated_at=now,current_research_state='persistent_entry_data_qualification_closed_before_fit',latest_scientific_entry=str(entrydir/'execution_next_entry_v111.json'),latest_scientific_result_unchanged=False,last_operation='470 causal feature/teacher qualification plus bounded cause and isolated timestamp interface repair; no fit',next_action=next_action,new_scientific_candidate=None,new_scientific_budget_opened=False,latest_research_round_completed=True,runtime=None,runtime_service=None,runtime_status_path=str(D/'status.json'),pending_scope_question=None,consecutive_status_only_heartbeats=0,current_failure_analysis='Data qualification only: E0 eligible-negative shortage; E1 intentional teacher-margin UNKNOWN. Not trained-performance or runtime failure.')
    s['recent_runs']=(s.get('recent_runs',[])+[dict(utc=now,work='Completed470feature/label qualification, independent exact290Bmap/teacher audits; isolatedduplicateclock repair.',evidence=[str(D/'qualification_result.json'),str(D/'author_audit.json'),str(D/'independent_review.json')],result='NOT_QUALIFIED_BEFORE_FIT; E0negative18/16cells,E1known82.956%; no learningrun',runtime='none',next_action=next_action)])[-10:];write(statepath,s)
    package=W/'资料_球型机器人导航_2026-09-30/2026-10-17_四相机弱基线与记忆检查'
    for src in [report,plan,D/'qualification_coverage.png',entrydir/'execution_next_entry_v111.json'] :shutil.copy2(src,package/src.name)
    index=package/'源码与原始数据索引.md';index.write_text(index.read_text()+f'\n## v111：持久入库学习数据资格（未拟合）\n\n- 全部470特征/470教师标签、源码、协议、来源链接、恢复与审查：{D}\n- 本包新子目录“持久入库数据资格”同步该21MB完整资格根；大原始RGB/视差继续由不可变canonical路径及SHA链接。\n- 结果NOT_QUALIFIED_BEFORE_FIT；没有训练/新预测/导航，不是跨机器实跑或自动Git上传。\n- 最新接续：{entrydir/"execution_next_entry_v111.json"}\n')
    print('Stage documents reconciled; need root seal/package copy/final verification.')
if __name__=='__main__':main()
