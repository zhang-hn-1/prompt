"""Offline full-result assessment; never consumed by current/historical policy."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();p=json.loads((H/'prior_rgb_navigation_protocol.json').read_text());status=json.loads((H/'prior_rgb_navigation_status.json').read_text());assert status['status']=='finished';assert all(x['exit_code']==0 for x in status['arms']);assert p['utc']<status['start_utc'];checks=0
for f,d in p['source_sha256'].items():assert sha(f)==d,f;checks+=1
arms={};series={};physics={};snapshots={};episodearrays={}
for arm in p['arms']:
 O=H/('prior_rgb_'+arm+'_run');m=json.loads((O/'artifact_manifest_post_close.json').read_text())
 for f,d in m['artifact_sha256'].items():assert sha(f)==d,f;checks+=1
 metrics=json.loads((O/'metrics.json').read_text());r=json.loads((O/'policy_records.json').read_text());clocks=json.loads((O/'camera_clocks.json').read_text());states=json.loads((O/'states.json').read_text());ph=np.load(O/'physics.npz');N=len(r);a=np.load(O/'episode_arrays.npz');identity=json.loads((O/'identity.json').read_text());task=json.loads((O/'task_episode.json').read_text());goal=np.array(task['actual_episode']['goal_position_world_m'][:2]);start=np.array(task['actual_episode']['start_position_world_m'][:2]);xy=np.array([s['position'][:2] for s in states]);direction=(goal-start)/np.linalg.norm(goal-start);progress=(xy-start)@direction;cross=(xy-start)@np.array([-direction[1],direction[0]]);dgoal=np.linalg.norm(xy-goal,axis=1)
 assert len(clocks)==len(states)==N+1 and len(ph['simulation_time_s'])==5*N+1;assert all(not x['local']['global_reference_consumed'] and not x['local']['source_model_admitted'] for x in r)
 source_count=len(list((O/'source_predictions').glob('*.npz')));assert source_count<=71;prior=None;maxhistorical=0;source_ages=[];mapcounts=[]
 for i,row in enumerate(r):
  mp=np.load(O/'maps'/f'frame_{i:03d}.npz');pred=np.load(O/'source_predictions'/f"frame_{row['local']['source_frame']:03d}.npz");source_ages.append(row['local']['source_age_s']);assert pred['confidence'].dtype==bool;assert float(pred['time_s'])<=row['local']['time_s'];assert not mp['task_prior_is_observation'];assert mp['task_ground_prior'].all()
  if prior is None:prior=mp['safe_start_prior'].copy()
  else:assert np.array_equal(prior,mp['safe_start_prior'])
  codes=mp['provenance'];floor=mp['labels']==0;haz=mp['labels']==1;assert np.isfinite(mp['floor_time'][floor]).all() and np.isfinite(mp['hazard_time'][haz]).all();assert (mp['floor_time'][floor]<=float(mp['time_s'])+1e-6).all() and (mp['hazard_time'][haz]<=float(mp['time_s'])+1e-6).all();maxhistorical=max(maxhistorical,int((codes==4).sum()));mapcounts.append(dict(frame=i,time_s=row['local']['time_s'],observed_floor=int(floor.sum()),observed_hazard=int(haz.sum()),history_hazard=int((codes==4).sum()),unknown=int((codes==0).sum()),source_frame=row['local']['source_frame'],source_age_s=row['local']['source_age_s']))
 contacts=json.loads((O/'physics_contacts.json').read_text());contact_count=sum(bool(x['contact']) for x in contacts);videos=[]
 for name in ['stereo_view','third_person']:
  cap=cv2.VideoCapture(str(O/(name+'.mp4')));n=0
  while True:
   ok,_=cap.read()
   if not ok:break
   n+=1
  fps=float(cap.get(cv2.CAP_PROP_FPS));cap.release();assert n==N+1 and fps==10.;videos.append(dict(name=name,frames=n,fps=fps))
 valid=bool(metrics['success'] and contact_count==0 and not bool(metrics['runtime_error']) and metrics['elapsed_s']<=35.00001 and dgoal[-1]<=.200001)
 per=dict(local_task_navigation_pass=valid,success=bool(metrics['success']),termination=metrics['termination_reason'],elapsed_s=metrics['elapsed_s'],final_goal_distance_m=float(dgoal[-1]),path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),actual_line_final_progress_m=float(progress[-1]),actual_line_max_progress_m=float(progress.max()),cross_track_rms_to_goal_line_m=float(np.sqrt(np.mean(cross**2))),cross_track_peak_to_goal_line_m=float(np.abs(cross).max()),contact_samples=contact_count,policy_calls=N,captures=N+1,physics_samples=len(ph['simulation_time_s']),RGB_inferences=source_count,source_age_max_s=max(source_ages),peak_history_hazard_cells=maxhistorical,videos=videos,model_latency_p50_s=float(np.percentile([x['rgb_source_wall_s'] for x in clocks if x['source_predicted']],50)),model_latency_p95_s=float(np.percentile([x['rgb_source_wall_s'] for x in clocks if x['source_predicted']],95)),policy_latency_p95_s=float(np.percentile([x['policy_wall_s'] for x in r],95)),max_render_clock_delta_s=max(abs(x['raw_time_s']-v) for x in clocks for v in x['eyes'].values()),knownpose_privileged=True,original_strict_source_admitted=False,formal_navigation_result=False,realtime_10Hz_verified=False)
 arms[arm]=per;series[arm]=dict(states=states,maps=mapcounts,task=task);physics[arm]=ph;episodearrays[arm]=a;snapshots[arm]=json.loads((O/'execution_states.json').read_text())
A=episodearrays['current']['actions'];B=episodearrays['history']['actions'];common=min(len(A),len(B));diff=np.any(A[:common]!=B[:common],axis=1);changed=np.flatnonzero(diff);branch=int(changed[0]) if len(changed) else common;fields=[]
for key in physics['current'].files:
 if key not in ['raw_simulation_time_s','raw_physics_step_index']:
  np.testing.assert_equal(physics['current'][key][:5*branch+1],physics['history'][key][:5*branch+1]);fields.append(key)
assert snapshots['current'][:branch+1]==snapshots['history'][:branch+1]
executed=np.any(physics['current']['applied_action'][1:5*common+1]!=physics['history']['applied_action'][1:5*common+1],axis=1)
comparison=dict(common_blocks=common,first_request_divergence_frame=None if not len(changed) else branch,first_request_divergence_time_s=None if not len(changed) else branch*.1,changed_request_blocks=int(diff.sum()),changed_applied_ticks=int(executed.sum()),exact_prefix_nonraw_fields=fields,exact_prefix_physics_samples=5*branch+1,complete_prefix_execution_states=branch+1)
control_changed=bool(diff.any() and executed.any());no_regression=arms['history']['contact_samples']<=arms['current']['contact_samples'] and arms['history']['final_goal_distance_m']<=arms['current']['final_goal_distance_m']+1e-6 and (not arms['current']['success'] or (arms['history']['success'] and arms['history']['elapsed_s']<=arms['current']['elapsed_s']+1e-6));accepted=bool(arms['history']['local_task_navigation_pass'] and control_changed and no_regression)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=arms,comparison=comparison,history_candidate_pass=accepted,actual_control_changed=control_changed,history_no_regression=no_regression,identity_checks=checks,RGB_source_oldgate_rejected=True,scope='two separately labelled2m RGB+knownpose prior/controlledexploration tasks, not fullE1/RGBestimatedpose/formal successrate',whole_research_complete=False)
(H/'prior_rgb_navigation_assessment.json').write_text(json.dumps(result,indent=2)+'\n');(H/'prior_rgb_navigation_assessment_series.json').write_text(json.dumps(series,indent=2)+'\n');print(json.dumps(result,indent=2))
