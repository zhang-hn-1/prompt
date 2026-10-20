from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;src=H/'run_short_obstacle_navigation.py';s=src.read_text();edits=[
("from short_obstacle_task import short_obstacle_endpoints","from short_obstacle_task import short_obstacle_endpoints\nfrom retreat_observation_schedule import request as probe_request"),
("short_obstacle_navigation_protocol.json","retreat_observation_protocol.json"),
("out=H/('short_obstacle_'+arm+'_run')","out=H/'retreat_observation_run'"),
("action=act(observation,goal);policy_wall_s=time.perf_counter()-begin","nominal_action=act(observation,goal);policy_wall_s=time.perf_counter()-begin\n        action=probe_request(policy.records[-1]['time_s'])\n        policy.records[-1]=dict(policy.records[-1],nominal_request=nominal_action.tolist(),request=action.tolist(),execution_role='fixed_retreat_observation_probe',nominal_policy_fields_not_applied=True)"),
("config=RunnerConfig()","config=RunnerConfig(episode_budget_s=18.)"),
("local_goal_m=2.,memory_arm=arm","local_goal_m=2.6,execution_role='fixed_retreat_observation_probe',diagnostic_duration_s=18.,original_formal_budget_unchanged_s=35.,nominal_policy_outputs_not_applied=True,memory_arm=arm"),
("SHORT_OBSTACLE_DIAGNOSTIC_COMPLETE","RETREAT_OBSERVATION_DIAGNOSTIC_COMPLETE")]
for a,b in edits:assert s.count(a)==1,(a,s.count(a));s=s.replace(a,b)
check=s
for a,b in reversed(edits):check=check.replace(b,a)
assert check==src.read_text();compile(s,'derived_retreat_runner','exec');dest=H/'run_retreat_observation_probe.py';dest.write_text(s)
stage='''from pathlib import Path
import sys,json,datetime,os,subprocess,hashlib
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'retreat_observation_status.json';assert not status.exists();row=dict(start_utc=now(),status='running',maximum_sequences=1,diagnostic_duration_s=18.,not_navigation=True);status.write_text(json.dumps(row,indent=2)+'\\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_prior_rgb',TORCH_COMPILE_DISABLE='1')
with (H/'retreat_observation_run.log').open('w') as out:r=subprocess.run([sys.executable,str(H/'run_retreat_observation_probe.py'),'current'],stdout=out,stderr=subprocess.STDOUT,env=env)
row.update(status='finished' if r.returncode==0 else 'runtime_failed',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\\n')
o=H/'retreat_observation_run'
if o.exists():
 m={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in o.rglob('*') if p.is_file()};(o/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=m,scope='post child shutdown; excludes self'),indent=2)+'\\n')
print(json.dumps(row));sys.exit(r.returncode)
'''
(H/'run_retreat_observation_stage.py').write_text(stage);compile(stage,'stage','exec')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();p=json.loads((H/'short_obstacle_navigation_protocol.json').read_text());sources=dict(p['source_sha256'])
for f,h in sources.items():assert sha(Path(f))==h,f
for f in ['prepare_retreat_observation_probe.py','run_retreat_observation_probe.py','run_retreat_observation_stage.py','retreat_observation_schedule.py']:sources[str(H/f)]=sha(H/f)
p.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,arms=['current'],maximum_sequences=1,task='one18s fixed backward observation diagnostic',execution_role='scheduled0/-0.3/0 main; auxiliary zero, nominal policy read-only to execution',schedule_s=[[0,6,0],[6,12,-.3],[12,18,0]],diagnostic_duration_s=18.,not_navigation=True,formal_budget_s=35.,source_gate_unchanged=True,retreat_space_not_claimed_observed_free=True,acceptance='measure restoration of physical binocular coverage and depth accuracy separately; no new navigation gate')
(H/'retreat_observation_protocol.json').write_text(json.dumps(p,indent=2)+'\n');(H/'retreat_observation_derivation.json').write_text(json.dumps(dict(parent_sha256=sha(src),derived_sha256=sha(dest),reverse_exact=True,changes=edits,no_planner_or_PID_source_edits=True),indent=2)+'\n');print('SEALED',len(sources),'sources; one18s sensing diagnostic')
