from pathlib import Path
import json,hashlib,datetime,re
W=Path('/home/an/Documents/ChatGPT/球型机器人导航');H=W/'experiments/stereo_memory_navigation_20261004_v1';sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();report=W/'reports/benchmark_perfect_visible_depth_and_startup_prior_20261006.md';review=json.loads((H/'startup_prior_review.json').read_text());archive=H/'startup_prior_reviewed_body.md';assert not archive.exists();archive.write_bytes(report.read_bytes())
report.write_bytes(archive.read_bytes()+'''\n\n## 一次独立终审与作者收尾

ONE fresh readonly终审 C0/I0/M0。个人7fixture/.26s、270SHA、65行已存支持统计、40端点实际两眼pose投影抽样；prior185固定、不写观测/hazard优先与futuretrajectory后置边界确认。65packet/map完整重建、3future替换/截断和361suite为作者证据，审查者未重复。没有任何物理/GPU/模型/采集/新scientificreplay。

原审正文精确保存在H/startup_prior_reviewed_body.md。本附录、任务条件总结、scheduler和master行政追加为作者后续执行，不冒称review覆盖。对所有declined范围作者裁决：没有全量重新生成来源/地图或重跑因果trap、没有新采集/physics/GPU/fit/完整scientificreplay、未证明RGB/estimatedpose/online/完整3D/safeaction/fresh/nav/整任务完成，均保留未验证/未执行；若外推会产生身份或准入误报。无二审/无源码修改/无删除。

条件诊断已经关闭，两臂sweepREJECT。原固定位姿/深度/mapper gate不改；H/task_conditions_pending_20261006.json记录尚需选择的新任务条件。用户原要求rig/验收冻结，故不能自动把unknown探索任务当获准原严格导航。异步选择未答时不默认推荐项通过；选择新条件后再冻结独立实验。所有原始数据保留。

自动任务automation经purpose-tool update暂停，原名称/prompt/每小时规则/目标chat保持；原因是等待任务条件答复，不是宣称整研究完成或隐藏正在运行实验。当前科学/GPU/采集进程已结束；整体仍NOTcomplete。收到明确条件后接续新任务，不重跑本阶段矩阵。\n'''.encode())
masters=[W/'research_mission.md',W/'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md',H/'progress.md'];prefix={}
for i,f in enumerate(masters):
 dest=H/('startup_prior_preclose_'+['mission','plan','progress'][i]+'.md');assert not dest.exists();dest.write_bytes(f.read_bytes());prefix[str(f)]=dict(archive=str(dest),bytes=dest.stat().st_size,sha256=sha(dest))
 with f.open('a') as stream:stream.write('''\n\n## 2026-10-06 独立完美可见来源/起步prior拒绝，等待新任务条件

最新reports/benchmark_perfect_visible_depth_and_startup_prior_20261006.md与research_status_and_next_task_conditions_20261006.md。ONE既有65帧privileged当前双目可见renderer端点消费21:03:12.829→21:03:41.200UTCexit0；110186records/225floor/18peakfloorcells。immutable t6起点185格只task_assumption、neverobserved/neverfollowsrobot、hazard优先；whole498sweep313outsideprior最少299unknown，strict0/65、prior0/53，prior currentdisk只1/53。futureactualtrajectory后manifest才offline评，不是已知在线plan；没有新GPU/采集/fit/action/nav/rates。4prior+3投影literalRED→GREEN、361suite10.81s、270SHA/65source65map重建/3actualfuturetrap作者证据；ONEfreshreviewC0I0M0/7fixture.26/270SHA/65savedcounts/40projection抽样。原审正文与master前缀精确保留，行政finalcheck另署。

不是所有RGB/地图表示不可能的证明；仅关闭当前bounded candidates/端点支持条件。所有旧source/pose/35snav失败、M2/学习前置门槛保留，原Oracle30.7s历史knownmap到达不是RGB成功。源模型拟合与原生IMU分支不得重开扫参数。

当前无science进程运行；独立step真实结束后已接续prior而非仅stage-only停止。现等待人类选择新任务条件：1保持安装，另列地面先验+受控未知探索任务，原严格benchmark保留；2保持严格观测要求，补近场信息/可验证初始localmap。Question在本chat异步提出，未回复不等于同意；冻结rig/验收条件下不能自行把风险探索等同原strictpass。H/task_conditions_pending_20261006.json为具体接续入口；下一步先读该记录+人类答复，冻结独立新任务再做一个实际闭环候选。不继续重复已有矩阵/审计，不删除证据。

依最新heartbeat“必须长期等待用户时明确原因后停止续做”，purpose-tool把原automation状态PAUSED；名称/prompt/小时rrule/target保持。不是completed，whole研究仍NOTcomplete。收到条件后恢复接续；不要仅因新heartbeat重做原实验。最新pending记录和finalclosure清单为权威状态，旧ACTIVE段为历史，不倒改字节。\n''')
with (H/'startup_prior_ledger.md').open('a') as stream:stream.write('\nFinal ONEreview C0I0M0; all declined scopes ruled unvalidated/execution-inherited in report. Task3 closed conditionREJECT, wholeNOTcomplete. Science jobs allfinished; taskconditionchoice pending, async answer notdefault. automation PAUSED viapurpose-tool perheartbeatlongwait instruction; frozenprompt/name/rrule/target unchanged. No further network/matrix/fit/control at currentgate. Raw evidence retained.\n')
pending=H/'task_conditions_pending_20261006.json';state=json.loads(pending.read_text());state.update(current_pending_work='human task-condition choice only; readonly final review and administrative closure complete',automation_status='PAUSED',last_utc=now(),latest_report=str(report));pending.write_text(json.dumps(state,indent=2,ensure_ascii=False)+'\n')
sumreport=W/'reports/research_status_and_next_task_conditions_20261006.md'
with sumreport.open('a') as stream:stream.write('\n\n作者收尾：上述两阶段均已完成一次独立终审，最新prior审C0/I0/M0；正文与原数据未追改。当前没有science进程；等待本chat已提出的任务条件选择。自动续做按此前“长期等待用户则停止”要求暂停，原prompt/规则/目标保持。整体未完成；回答后接续新条件任务，不重做原矩阵。\n')
checks=0;p=json.loads((H/'startup_prior_protocol.json').read_text())
for kind in ('source_sha256','input_sha256'):
 for f,d in p[kind].items():assert sha(Path(f))==d,f;checks+=1
for n,q in prefix.items():assert Path(n).read_bytes()[:q['bytes']]==Path(q['archive']).read_bytes();assert sha(Path(q['archive']))==q['sha256']
auto=Path('/home/an/.codex/automations/automation/automation.toml').read_text();saved=json.loads(Path('/tmp/spherical_automation_saved.json').read_text());fields={k:json.loads(re.search('^'+k+' = (".*")$',auto,re.M).group(1)) for k in ['id','name','kind','prompt','status','rrule','target_thread_id']};assert fields['status']=='PAUSED'
for k in ['id','name','kind','prompt','rrule','target_thread_id']:assert fields[k]==saved[k],k
owned=[report,archive,sumreport,pending]+list(H.glob('startup_prior*'))+[H/'perfect_visible_points.py',H/'test_perfect_visible_points.py',H/'test_startup_prior.py',H/'run_startup_prior_diagnostic.py',H/'assess_startup_prior.py',H/'verify_startup_prior.py']+[Path(n) for n in p['source_sha256']];files=set()
for f in owned:
 if f.is_dir():files.update(x for x in f.rglob('*') if x.is_file())
 elif f.is_file():files.add(f)
files={f for f in files if f.name not in ['startup_prior_final_identity.json','startup_prior_final_check.json']}
idf=H/'startup_prior_final_identity.json';idf.write_text(json.dumps(dict(utc=now(),owned_sha256={str(f):sha(f) for f in sorted(files)},master_prefix_archives=prefix,automation=dict(status='PAUSED',preserved_name_prompt_rule_target=True),whole_research_complete=False),indent=2)+'\n')
for f,d in json.loads(idf.read_text())['owned_sha256'].items():assert sha(Path(f))==d,f
result=dict(utc=now(),administrative_pass=True,presealed_checks=checks,owned_files=len(files),scientific_condition='REJECT',review='C0I0M0',automation_status='PAUSED',awaiting_human_task_conditions=True,whole_research_complete=False)
(H/'startup_prior_final_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
