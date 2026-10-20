from pathlib import Path
import json,hashlib,datetime
H=Path(__file__).resolve().parent;src=H/'run_retreat_observation_probe.py';s=src.read_text();edits=[
('from retreat_observation_schedule import request as probe_request','from memory_validation_schedule import request as probe_request'),
('retreat_observation_protocol.json','memory_validation_collection_protocol.json'),
("arm=sys.argv[1];assert arm in config['arms'];out=H/'retreat_observation_run'","kind=sys.argv[1];assert kind in config['kinds'];arm='current';out=H/('memory_validation_'+kind+'_run')"),
("newstart,newgoal=short_obstacle_endpoints(episode.start_position_world_m,episode.goal_position_world_m)","newstart,newgoal=short_obstacle_endpoints(episode.start_position_world_m,episode.goal_position_world_m) if kind=='near' else (np.asarray(episode.start_position_world_m),np.asarray(episode.goal_position_world_m))"),
("episode_id='short_obstacle_e1_'+arm,start_position_world_m=tuple(newstart)","episode_id='sealed_memory_validation_'+kind,start_quaternion_world_wxyz=(0.984807753012208,0.,0.,-0.17364817766693033),start_position_world_m=tuple(newstart)"),
("action=probe_request(policy.records[-1]['time_s'])","action=probe_request(policy.records[-1]['time_s'],kind)"),
("fixed_retreat_observation_probe","fixed_memory_validation_observation"),
("local_goal_m=2.6,execution_role=","local_goal_m=float(np.linalg.norm(newgoal[:2]-newstart[:2])),collection_kind=kind,heldout_for_learning=True,execution_role="),
("RETREAT_OBSERVATION_DIAGNOSTIC_COMPLETE","MEMORY_VALIDATION_SEQUENCE_COMPLETE"),
("artifact_sha256=manifest,source_model_admitted=False,rgb_control=True","artifact_sha256=manifest,source_model_admitted=False,rgb_control=False,scheduled_probe_control=True")]
for a,b in edits:assert s.count(a)>=1,(a,s.count(a));s=s.replace(a,b)
check=s
for a,b in reversed(edits):check=check.replace(b,a)
assert check==src.read_text();compile(s,'memory_validation_runner','exec');(H/'run_memory_validation_collection.py').write_text(s)
stage='''from pathlib import Path
import json,sys,subprocess,datetime,os,hashlib
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();status=H/'memory_validation_collection_status.json';assert not status.exists();row=dict(start_utc=now(),status='running',maximum_sequences=2,sequences=[],purpose='seal two entire validation sequences before fitting; do not read heldout labels/scores');status.write_text(json.dumps(row,indent=2)+'\\n')
for kind in ['far','near']:
 entry=dict(kind=kind,status='running',start_utc=now());row['sequences'].append(entry);status.write_text(json.dumps(row,indent=2)+'\\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_prior_rgb',TORCH_COMPILE_DISABLE='1')
 with (H/('memory_validation_'+kind+'_run.log')).open('w') as out:r=subprocess.run([sys.executable,str(H/'run_memory_validation_collection.py'),kind],stdout=out,stderr=subprocess.STDOUT,env=env)
 entry.update(status='finished' if r.returncode==0 else 'runtime_failed',exit_code=r.returncode,end_utc=now());status.write_text(json.dumps(row,indent=2)+'\\n');o=H/('memory_validation_'+kind+'_run')
 if o.exists():
  manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in o.rglob('*') if p.is_file()};(o/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,whole_sequence_heldout=True,label_deserialization_by_fitter_forbidden_before_fit_and_prediction_seal=True),indent=2)+'\\n')
 if r.returncode:
  row.update(status='runtime_failed',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\\n');sys.exit(r.returncode)
row.update(status='finished_both_validation_sequences_sealed',end_utc=now());status.write_text(json.dumps(row,indent=2)+'\\n');print(json.dumps(row))
'''
(H/'run_memory_validation_collection_stage.py').write_text(stage);compile(stage,'memory_validation_stage','exec');sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();p=json.loads((H/'retreat_observation_protocol.json').read_text());sources=dict(p['source_sha256'])
for f,h in sources.items():assert sha(Path(f))==h,f
for f in ['prepare_memory_validation_collection.py','run_memory_validation_collection.py','run_memory_validation_collection_stage.py','memory_validation_schedule.py']:sources[str(H/f)]=sha(H/f)
plan=H.parents[1]/'docs/superpowers/plans/2026-10-07-learned-memory-reliability.md';sources[str(plan)]=sha(plan)
p.update(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,kinds=['far','near'],maximum_sequences=2,task='two fresh whole-sequence learned-memory validation collections; no model fit/control use',accepted_controller='fixed schedule only; PriorRGBPolicy nominal diagnostics not execution',initial_yaw_deg=-20.,diagnostic_duration_s=18.,source='original unmodifiedFast eightiteration/currentCAD postfilter; zeroCAD preprocessing NOT installed',fit_datasets=['fresh_rolling_prior_shadow_current_run','occluded_obstacle_memory_navigation_history_run','short_obstacle_current_run','short_obstacle_history_run','retreat_observation_run'],fitting_not_before_both_held_sequences_sealed=True,heldout_label_scores_not_read_before_fit_and_prediction_seal=True,maximum_learning_candidates=1,whole_research_complete=False)
(H/'memory_validation_collection_protocol.json').write_text(json.dumps(p,indent=2)+'\n');(H/'memory_validation_collection_derivation.json').write_text(json.dumps(dict(parent_source_sha256=sha(src),derived_sha256=hashlib.sha256(s.encode()).hexdigest(),reverse_exact=True,edits=edits),indent=2)+'\n');print('SEALED',len(sources),'sources, exactly two fresh validation18s sequences')
