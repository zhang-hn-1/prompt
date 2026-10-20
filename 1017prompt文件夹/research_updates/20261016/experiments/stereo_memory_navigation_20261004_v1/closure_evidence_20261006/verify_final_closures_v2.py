from pathlib import Path
import json,hashlib,datetime
W=Path('/home/an/Documents/ChatGPT/球型机器人导航');H=W/'experiments/stereo_memory_navigation_20261004_v1';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
for prefix,report in [('dense_source_memory','benchmark_dense_source_memory_and_startup_support_20261006.md'),('startup_prior','benchmark_perfect_visible_depth_and_startup_prior_20261006.md')]:
 old=json.loads((H/(prefix+'_final_identity.json')).read_text());bad=[f for f,d in old['owned_sha256'].items() if sha(f)!=d];expected=str(H/(prefix+'_closure.log'));assert bad==[expected],bad
 failure=dict(utc=now(),failed_v1_owned_paths=bad,reason='v1 mistakenly hashed stdout closure.log before printing final_check; only own output log changed, all scientific source/input/artifacts unchanged',science_unchanged=True)
 scope=H/(prefix+'_closure_v1_output_scope.json');scope.write_text(json.dumps(failure,indent=2)+'\n')
 rf=W/'reports'/report
 with rf.open('a') as out:out.write('\n行政v2核验：初次最终清单把仍在写出的自身closure.log按空SHA纳入，只有此日志在打印check后发生SHA变化。原v1清单/check/log保留，独立scope记录；科学源/输入/产物全部一致。v2在日志结束后记录原日志SHA，另封本报告追加与scope，新checker输出不纳入自校验。原审/旧门槛不追改，无重跑科学/二审。权威最终行政证据为 '+prefix+'_final_check_v2.json。\n')
 owned={f:sha(f) for f in old['owned_sha256']};owned[str(scope)]=sha(scope)
 identity=H/(prefix+'_final_identity_v2.json');identity.write_text(json.dumps(dict(utc=now(),owned_sha256=owned,old_v1_identity_sha256=sha(H/(prefix+'_final_identity.json')),old_v1_finalcheck_sha256=sha(H/(prefix+'_final_check.json')),master_prefix_archives=old['master_prefix_archives'],science_unchanged=True,whole_research_complete=False),indent=2)+'\n')
 for f,d in owned.items():assert sha(f)==d,f
 for f,q in old['master_prefix_archives'].items():
  size=q['bytes'];assert Path(f).read_bytes()[:size]==Path(q['archive']).read_bytes();assert sha(q['archive'])==q.get('sha256',q.get('old_sha256'))
 result=dict(utc=now(),administrative_pass=True,owned_checks=len(owned),science_unchanged=True,scientific_rejection_preserved=True,whole_research_complete=False,automation_status='PAUSED',pending_condition_answer=True)
 (H/(prefix+'_final_check_v2.json')).write_text(json.dumps(result,indent=2)+'\n');print(prefix,json.dumps(result))
