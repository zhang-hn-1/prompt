"""Run only after saved, fully audited local acceptance. Never rescues failed gate."""
from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
a=json.loads((H/'terminal_goal_navigation_assessment.json').read_text());assert a['terminal_consumer_gate_pass'] and a['local_task_pass'] and a['terminal_distance_gate'] and a['elapsed_gate'] and a['movement_geometry_gate']
base=H/'run_terminal_goal_navigation.py';s=base.read_text();changes=[]
def rep(x,y):
 global s
 assert s.count(x)==1;(changes.append(dict(old=x,new=y)));s=s.replace(x,y)
rep("H/'terminal_goal_navigation_protocol.json'","H/'terminal_goal_full_navigation_protocol.json'")
rep("H/'terminal_goal_history_run'","H/'terminal_goal_full_history_run'")
old="episode=dataclasses.replace(episode,episode_id='terminal_goal_e1_local2m_'+arm,goal_position_world_m=tuple(short_goal(episode.start_position_world_m,episode.goal_position_world_m)),euclidean_distance_m=min(2.,episode.euclidean_distance_m),oracle_path_length_m=max(2.,min(episode.oracle_path_length_m,2.)))"
rep(old,"episode=dataclasses.replace(episode,episode_id='terminal_goal_e1_originalfull_'+arm)")
rep("task='explicit_flat_ground_prior_unknown_exploration_local2m'","task='explicit_flat_ground_prior_unknown_exploration_originalfull'")
rep('not_original_complete_E1=True','not_original_complete_E1=False')
rep("'TERMINAL_GOAL_LOCAL_COMPLETE'","'TERMINAL_GOAL_FULL_COMPLETE'")
out=H/'run_terminal_goal_full_navigation.py';assert not out.exists();out.write_text(s);reverse=s
for c in reversed(changes):assert reverse.count(c['new'])==1;reverse=reverse.replace(c['new'],c['old'])
assert reverse==base.read_text()
(H/'terminal_goal_full_runner_derivation.json').write_text(json.dumps(dict(base=str(base),base_sha256=sha(base),candidate=str(out),candidate_sha256=sha(out),reverse_exact=True,changes=changes,scientific_scope='SAME acceptedterminalconsumer originalfullgoal35s no newcontrol'),indent=2)+'\n')
stage=(H/'run_terminal_goal_navigation_stage.py').read_text().replace('terminal_goal_navigation','terminal_goal_full_navigation').replace('terminal_goal_history_run','terminal_goal_full_history_run');(H/'run_terminal_goal_full_navigation_stage.py').write_text(stage)
p=json.loads((H/'terminal_goal_navigation_protocol.json').read_text());p.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),local_goal_m=None,actual_original_goal=True,local_acceptance_sha256=sha(H/'terminal_goal_navigation_assessment.json'),fullgoal_gate='actualgoal<=.2 by35/contact0/noerror; independentactualprogress/lateral/cuspcompletion; notformalE0E1E2',maximum_sequences=1)
sources=set(p['source_sha256']);sources.update(str(f) for f in [out,H/'prepare_terminal_goal_full_navigation.py',H/'terminal_goal_full_runner_derivation.json',H/'run_terminal_goal_full_navigation_stage.py',H/'terminal_goal_navigation_assessment.json']);p['source_sha256']={f:sha(f) for f in sorted(sources)}
(H/'terminal_goal_full_navigation_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print('fullgoal admission',len(sources))
