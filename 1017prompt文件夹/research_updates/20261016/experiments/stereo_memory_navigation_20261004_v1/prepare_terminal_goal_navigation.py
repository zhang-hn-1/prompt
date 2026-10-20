from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
# Keep all old candidate artifacts frozen, including its rejected report.
oldid=json.loads((H/'nominal_cap_final_identity.json').read_text())
for p,v in oldid['owned_sha256'].items():assert sha(p)==v,p
base=H/'run_nominal_cap_navigation.py';s=base.read_text();changes=[]
def rep(a,b):
 global s
 assert s.count(a)==1,(a,s.count(a));s=s.replace(a,b);changes.append(dict(old=a,new=b))
rep('from nominal_unknown_cap import with_nominal_unknown_cap,ReadOnlyHistorySlowShadow','from nominal_unknown_cap import ReadOnlyHistorySlowShadow\nfrom terminal_goal_consumer import with_terminal_goal_consumer')
rep("H/'nominal_cap_navigation_protocol.json'","H/'terminal_goal_navigation_protocol.json'")
rep("H/'nominal_cap_history_run'","H/'terminal_goal_history_run'")
rep("episode_id='nominal_cap_e1_local2m_'+arm","episode_id='terminal_goal_e1_local2m_'+arm")
rep('policy=with_nominal_unknown_cap(PriorRGBPolicy)(protocol,memory_mode=arm)','policy=with_terminal_goal_consumer(PriorRGBPolicy)(protocol,memory_mode=arm)')
rep("'NOMINAL_CAP_LOCAL_COMPLETE'","'TERMINAL_GOAL_LOCAL_COMPLETE'")
out=H/'run_terminal_goal_navigation.py';assert not out.exists();out.write_text(s)
r=s
for c in reversed(changes):assert r.count(c['new'])==1;r=r.replace(c['new'],c['old'])
assert r==base.read_text()
(H/'terminal_goal_runner_derivation.json').write_text(json.dumps(dict(base=str(base),base_sha256=sha(base),candidate=str(out),candidate_sha256=sha(out),reverse_exact=True,changes=changes,scientific_change='one final terminal goal main cap consumer'),indent=2)+'\n')
stage=(H/'run_nominal_cap_navigation_stage.py').read_text().replace('nominal_cap_navigation','terminal_goal_navigation').replace('nominal_cap_history_run','terminal_goal_history_run');(H/'run_terminal_goal_navigation_stage.py').write_text(stage)
p=json.loads((H/'nominal_cap_navigation_protocol.json').read_text());baseline=json.loads((H/'masked_memory_history_run/metrics.json').read_text());print('baseline',baseline)
p.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),unknown_request_main_cap_rad_s=1.,terminal_main_cap_rad_s=.3,terminal_entry_distance_m=.6,terminal_latch=True,maximum_sequences=1,fullgoal_only_after_local_gate=True,local_frozen_baseline_metrics=baseline)
sources=set(p['source_sha256'])
for f in [out,H/'terminal_goal_consumer.py',H/'test_terminal_goal_consumer.py',H/'prepare_terminal_goal_navigation.py',H/'run_terminal_goal_navigation_stage.py',H/'terminal_goal_runner_derivation.json',W/'docs/superpowers/specs/2026-10-07-terminal-goal-consumer-design.md',W/'docs/superpowers/plans/2026-10-07-terminal-goal-consumer.md']:sources.add(str(f))
p['source_sha256']={f:sha(f) for f in sorted(sources)}
(H/'terminal_goal_navigation_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print('preseal',len(sources))
