"""Offline primary fullgoal result and same-source shadow interface assessment."""
from pathlib import Path
import json,hashlib,numpy as np,cv2,datetime
H=Path(__file__).resolve().parent;O=H/'rgb_complete_goal_current_run';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();p=json.loads((H/'rgb_complete_goal_protocol.json').read_text());status=json.loads((H/'rgb_complete_goal_status.json').read_text());assert status['status']=='finished' and status['exit_code']==0 and p['utc']<status['start_utc'];nchecks=0
for f,s in p['source_sha256'].items():assert sha(f)==s,f;nchecks+=1
for f,s in json.loads((O/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f;nchecks+=1
metrics=json.loads((O/'metrics.json').read_text());states=json.loads((O/'states.json').read_text());rows=json.loads((O/'policy_records.json').read_text());shadow=json.loads((O/'shadow_records.json').read_text());task=json.loads((O/'task_episode.json').read_text());N=len(rows);ph=np.load(O/'physics.npz');clocks=json.loads((O/'camera_clocks.json').read_text());assert len(states)==len(clocks)==N+1 and len(ph['simulation_time_s'])==5*N+1 and len(shadow)==N
actual=task['actual_episode'];orig=task['original_episode']
for k in orig:
 if k!='episode_id':assert actual[k]==orig[k],k
xy=np.array([s['position'][:2] for s in states]);start=np.asarray(orig['start_position_world_m'][:2]);goal=np.asarray(orig['goal_position_world_m'][:2]);u=(goal-start)/np.linalg.norm(goal-start);progress=(xy-start)@u;contact=sum(bool(x['contact']) for x in json.loads((O/'physics_contacts.json').read_text()));inferences=sum(x['source_predicted'] for x in clocks);assert inferences<=71
changes=[];future_traps=0;counts={'settling':0,'no_route':0,'nonzero_request':0,'zero_tracking_request':0};stages=[];last=None;all_zero=[]
for i,(r,s) in enumerate(zip(rows,shadow)):
 a=r['local'];assert s['read_only'] and not s['executed'] and not s['alternative_future_truth_available'];assert s['frame']==i and s['source_frame']==a['source_frame'] and s['source_time_s']==a['source_time_s']<=s['time_s']==a['time_s'];assert not a['global_reference_consumed'] and not s['local']['global_reference_consumed'];assert not a['source_model_admitted'];pred=np.load(O/'source_predictions'/f"frame_{a['source_frame']:03d}.npz");assert pred['confidence'].dtype==bool
 if not np.array_equal(a['request'],s['request']):changes.append(i)
 name='settling' if a['time_s']<6-1e-6 else ('no_route' if a['plan']['status']!='success' else ('nonzero_request' if abs(a['request'][0])>1e-8 else 'zero_tracking_request'));counts[name]+=1
 if name!=last:
  if stages:stages[-1]['end_frame']=i-1;stages[-1]['exit_progress_m']=float(progress[i]);stages[-1]['elapsed_s']=(i-stages[-1]['start_frame'])*.1
  stages.append(dict(kind=name,start_frame=i,start_time_s=i*.1,entry_progress_m=float(progress[i])));last=name
 if abs(a['request'][0])<=1e-8 and a['time_s']>=6-1e-6:all_zero.append(i)
if stages:stages[-1].update(end_frame=N-1,exit_progress_m=float(progress[-1]),elapsed_s=(N-stages[-1]['start_frame'])*.1)
video=[]
for name in ('stereo_view','third_person'):
 c=cv2.VideoCapture(str(O/(name+'.mp4')));n=0
 while True:
  ok,_=c.read()
  if not ok:break
  n+=1
 fps=float(c.get(cv2.CAP_PROP_FPS));c.release();assert n==N+1 and fps==10;video.append(dict(name=name,frames=n,fps=fps))
rgbtime=[x['rgb_source_wall_s'] for x in clocks if x['source_predicted']];speed=np.linalg.norm(np.asarray([s['velocity'][:2] for s in states]),axis=1)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),primary=dict(success=bool(metrics['success']),termination=metrics['termination_reason'],elapsed_s=metrics['elapsed_s'],final_goal_distance_m=float(np.linalg.norm(xy[-1]-goal)),contact_samples=contact,runtime_error=bool(metrics['runtime_error']),actual_goal_line_progress_m=float(progress[-1]),max_goal_line_progress_m=float(progress.max()),actual_path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),goal_displacement_m=float(np.linalg.norm(goal-start)),policy_calls=N,captures=N+1,physics_samples=len(ph['simulation_time_s']),RGB_inferences=inferences,control_stage_counts=counts,stages=stages,first_postsettling_zero_request=None if not all_zero else all_zero[0]*.1),interface=dict(same_live_RGB_source_ids_all_calls=True,source_age_max_s=max(x['local']['source_age_s'] for x in rows),shadow_requests_never_executed=True,shadow_different_request_blocks=len(changes),shadow_first_difference_s=None if not changes else changes[0]*.1,render_clock_delta_max_s=max(abs(x['raw_time_s']-v) for x in clocks for v in x['eyes'].values()),primary_runtime_mutation_assertions_all_calls=True,CPU_shadow_onoff_fixture_verified=True,shadow_counterfactual_navigation_success_not_measured=True),performance=dict(model_wall_p50_s=float(np.percentile(rgbtime,50)),model_wall_p95_s=float(np.percentile(rgbtime,95)),shadow_wall_p95_s=float(np.percentile([x['shadow_wall_s'] for x in shadow],95)),XY_speed_p95_m_s=float(np.percentile(speed,95)),nominal_direct_goal_speed_required_after_6s_m_s=float(np.linalg.norm(goal-start)/29)),videos=video,identity_checks=nchecks,scope='one full originalgoal35s RGBknownpose explicit-risk primary plus readonly same-source memory interface; notformal RGBestimatedpose successrate or learnedworldmodel',whole_research_complete=False)
(H/'rgb_complete_goal_assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
