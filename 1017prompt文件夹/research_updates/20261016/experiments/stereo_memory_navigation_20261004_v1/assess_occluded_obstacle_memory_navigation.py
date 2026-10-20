"""Offline primary fullgoal result and same-source shadow interface assessment."""
from pathlib import Path
import json,hashlib,numpy as np,cv2,datetime
H=Path(__file__).resolve().parent;O=H/'occluded_obstacle_memory_navigation_history_run';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();p=json.loads((H/'occluded_obstacle_memory_navigation_protocol.json').read_text());status=json.loads((H/'occluded_obstacle_memory_navigation_status.json').read_text());assert status['status']=='finished' and status['exit_code']==0 and p['utc']<status['start_utc'];nchecks=0
for f,s in p['source_sha256'].items():assert sha(f)==s,f;nchecks+=1
for f,s in json.loads((O/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f;nchecks+=1
metrics=json.loads((O/'metrics.json').read_text());states=json.loads((O/'states.json').read_text());rows=json.loads((O/'policy_records.json').read_text());shadow=json.loads((O/'shadow_records.json').read_text());task=json.loads((O/'task_episode.json').read_text());N=len(rows);ph=np.load(O/'physics.npz');clocks=json.loads((O/'camera_clocks.json').read_text());assert len(states)==len(clocks)==N+1 and len(ph['simulation_time_s'])==5*N+1 and len(shadow)==N
actual=task['actual_episode'];orig=task['original_episode']
for k in orig:
 if k not in ('episode_id','start_quaternion_world_wxyz'):assert actual[k]==orig[k],k
assert np.allclose(actual['start_quaternion_world_wxyz'],[np.cos(np.deg2rad(20)/2),0,0,np.sin(np.deg2rad(20)/2)])
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

fullresult=r
from dynamic_self_mask import RobotSelfMask
from masked_source_pair import paired_sources
from stereo_geometry import restore_native
from verification_grid import VerificationGrid as PriorRGBGrid
cad=RobotSelfMask();packets={};oldpackets={};frames=[];total=dict(old=0,candidate=0,removed=0,old_correct=0,candidate_correct=0,removed_correct=0,removed_wrong=0,finite_removed=0);exact=0
for f in sorted((O/'source_predictions').glob('*.npz')):
 d=np.load(f);i=int(d['frame']);t=float(d['time_s']);pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz');history=np.load(O/'history'/f'frame_{i:03d}.npz');rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');assert set(rgb.files)=={'left_rgb','right_rgb','time_s'} and set(pose.files)=={'left_camera_usd_world','right_camera_usd_world','time_s'}
 raw=dict(depth_m=d['depth_m'],confidence=d['confidence'],disparity_native_px=restore_native(d['disparity_px']),frame=i,time_s=t);known=dict(left_camera_usd_world=pose['left_camera_usd_world'],right_camera_usd_world=pose['right_camera_usd_world'],root_position_world_m=states[i]['position'],time_s=float(pose['time_s']));old,new,mask=paired_sources(raw,known,history['proprio_history'],cad,t)
 for prefix,packet in [('old_sensor_packets',old),('sensor_packets',new)]:
  saved=np.load(O/prefix/f'source_{i:03d}.npz')
  for k,v in packet.items():np.testing.assert_equal(v,saved[k]);exact+=1
 m=np.load(O/'masks'/f'frame_{i:03d}.npz')
 for eye in ('left','right'):np.testing.assert_equal(m[eye],mask[eye]);exact+=1
 np.testing.assert_equal(m['current_joints'],history['proprio_history'][-1,9:12]);np.testing.assert_equal(m['current_joints'],states[i]['joints'])
 sel=np.isin(old['query_pixel_id'],new['query_pixel_id']);np.testing.assert_equal(old['world'][sel],new['world']);np.testing.assert_equal(old['partner_pixel_id'][sel],new['partner_pixel_id']);q=old['query_pixel_id'][~sel];s=old['partner_pixel_id'][~sel];assert np.all(mask['left'].ravel()[q]|mask['right'].ravel()[s])
 # Independent renderer depths are used ONLY after source packets are finalized.
 truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');pixels=old['query_pixel_id'];yy=pixels//640;xx=pixels%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(xx))];actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;error=np.linalg.norm(old['world']-actual,axis=1);finite=np.isfinite(error);correct=finite&(error<=.1)
 v=dict(old=len(old['world']),candidate=len(new['world']),removed=int((~sel).sum()),old_correct=int(correct.sum()),candidate_correct=int(correct[sel].sum()),removed_correct=int(correct[~sel].sum()),removed_wrong=int((finite&~correct&~sel).sum()),finite_removed=int((finite&~sel).sum()))
 for k in total:total[k]+=v[k]
 frames.append(dict(frame=i,time_s=t,**v));packets[i]=new;oldpackets[i]=old
primarygrid=None;shadowgrid=None;maparrays=0;shadowfields=0
for i,r in enumerate(rows):
 s=np.load(O/'maps'/f'frame_{i:03d}.npz');t=float(s['time_s']);root=states[i]['position'];sf=r['local']['source_frame'];assert sf==shadow[i]['source_frame'] and shadow[i]['source_time_s']==float(packets[sf]['time_s'])<=t and not shadow[i]['executed'] and shadow[i]['read_only']
 assert not s['task_prior_is_observation'] and s['task_ground_prior'].all()
 if primarygrid is None:primarygrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),'history');shadowgrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),'current')
 primarygrid.set_source(packets[sf]);primarygrid.update_points([],root,t);shadowgrid.set_source(packets[sf]);shadowgrid.update_points([],root,t)
 for k,v in dict(labels=primarygrid.labels,provenance=primarygrid.provenance(),floor_time=primarygrid.memory.floor_time,hazard_time=primarygrid.memory.hazard_time,task_ground_prior=primarygrid.task_ground_prior,safe_start_prior=primarygrid.safe_start_prior).items():np.testing.assert_equal(v,s[k]);maparrays+=1
 for k,v in shadowgrid.diagnostics().items():assert v==shadow[i]['local'][k],(i,k);shadowfields+=1
class Bomb(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future payload read')
  return super().__getitem__(k)
primarygrid.set_source(Bomb(time_s=primarygrid.time+1))
try:primarygrid.update_points([],states[-1]['position'],primarygrid.time)
except ValueError:pass
else:raise AssertionError('future source accepted')

base=H/'fresh_rolling_prior_shadow_current_run';oldrows=json.loads((base/'policy_records.json').read_text());actions=np.array([r['local']['request'] for r in rows]);oldactions=np.array([r['local']['request'] for r in oldrows]);common=min(N,len(oldrows));changed=np.flatnonzero(np.any(actions[:common]!=oldactions[:common],axis=1));branch=int(changed[0]) if len(changed) else common;oldph=np.load(base/'physics.npz');fields=[]
for k in oldph.files:
 if k not in ('raw_simulation_time_s','raw_physics_step_index'):
  np.testing.assert_equal(ph[k][:5*branch+1],oldph[k][:5*branch+1]);fields.append(k)
assert json.loads((O/'execution_states.json').read_text())[:branch+1]==json.loads((base/'execution_states.json').read_text())[:branch+1]
fullresult.update(source_filter='dynamic_current_joint_CAD',primary_memory='history',baseline_cap_rad_s=.3,rejected_cap1_fullgoal_not_run=True,paired_source_totals=total,paired_source_frames=frames,exact_source_mask_fields=exact,exact_primary_history_map_arrays=maparrays,exact_current_shadow_fields=shadowfields,future_payload_trap_pass=True,source_inferences_rebuilt=len(frames),first_actual_request_difference_frame=int(changed[0]) if len(changed) else None,actual_request_blocks_different=int(len(changed)),physical_prefix_samples=5*branch+1,physical_prefix_fields=fields,execution_prefix=branch+1,old_strict_source_pose_gates_preserved=True,RGB_reset_confound=True)
(H/'occluded_obstacle_memory_navigation_assessment.json').write_text(json.dumps(fullresult,indent=2)+'\n');print(json.dumps({k:v for k,v in fullresult.items() if k!='paired_source_frames'},indent=2))
