"""Author post-run verification only; none of these labels feeds online policy."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np,cv2
from dynamic_self_mask import RobotSelfMask
from masked_source_pair import paired_sources
from stereo_geometry import restore_native
from verification_grid import VerificationGrid as PriorRGBGrid
H=Path(__file__).resolve().parent;O=H/'terminal_goal_history_run';now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
p=json.loads((H/'terminal_goal_navigation_protocol.json').read_text());status=json.loads((H/'terminal_goal_navigation_status.json').read_text());assert status['status']=='finished' and status['arms'][0]['exit_code']==0 and p['utc']<status['start_utc'];checks=0
for manifest in (p['source_sha256'],json.loads((O/'artifact_manifest_post_close.json').read_text())['artifact_sha256']):
 for f,s in manifest.items():assert sha(f)==s,f;checks+=1
rows=json.loads((O/'policy_records.json').read_text());states=json.loads((O/'states.json').read_text());clocks=json.loads((O/'camera_clocks.json').read_text());shadow=json.loads((O/'shadow_records.json').read_text());N=len(rows);assert len(shadow)==N and len(states)==len(clocks)==N+1
ph=np.load(O/'physics.npz');assert len(ph['simulation_time_s'])==5*N+1
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
 sel=np.isin(old['query_pixel_id'],new['query_pixel_id']);np.testing.assert_allclose(old['world'][sel],new['world'],rtol=32*np.finfo(float).eps,atol=32*np.finfo(float).eps);np.testing.assert_equal(old['partner_pixel_id'][sel],new['partner_pixel_id']);q=old['query_pixel_id'][~sel];s=old['partner_pixel_id'][~sel];assert np.all(mask['left'].ravel()[q]|mask['right'].ravel()[s])
 # Independent renderer depths are used ONLY after source packets are finalized.
 truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');pixels=old['query_pixel_id'];yy=pixels//640;xx=pixels%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(xx))];actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;error=np.linalg.norm(old['world']-actual,axis=1);finite=np.isfinite(error);correct=finite&(error<=.1)
 v=dict(old=len(old['world']),candidate=len(new['world']),removed=int((~sel).sum()),old_correct=int(correct.sum()),candidate_correct=int(correct[sel].sum()),removed_correct=int(correct[~sel].sum()),removed_wrong=int((finite&~correct&~sel).sum()),finite_removed=int((finite&~sel).sum()))
 for k in total:total[k]+=v[k]
 frames.append(dict(frame=i,time_s=t,**v));packets[i]=new;oldpackets[i]=old
primarygrid=None;shadowgrid=None;maparrays=0;shadowfields=0
for i,r in enumerate(rows):
 s=np.load(O/'maps'/f'frame_{i:03d}.npz');t=float(s['time_s']);root=states[i]['position'];sf=r['local']['source_frame'];assert sf==shadow[i]['source_frame'] and shadow[i]['source_time_s']==float(packets[sf]['time_s'])<=t and not shadow[i]['executed'] and shadow[i]['read_only']
 assert not s['task_prior_is_observation'] and s['task_ground_prior'].all()
 if primarygrid is None:primarygrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),'history');shadowgrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),'history')
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
A=np.array([r['local']['request'] for r in rows]);B=np.array([s['request'] for s in shadow]);different=np.any(A!=B,axis=1);first=np.flatnonzero(different);metrics=json.loads((O/'metrics.json').read_text());task=json.loads((O/'task_episode.json').read_text());goal=np.array(task['actual_episode']['goal_position_world_m'][:2]);start=np.array(task['actual_episode']['start_position_world_m'][:2]);xy=np.array([s['position'][:2] for s in states]);direction=(goal-start)/np.linalg.norm(goal-start);progress=(xy-start)@direction;cross=(xy-start)@np.array([-direction[1],direction[0]]);distance=float(np.linalg.norm(xy[-1]-goal));contacts=json.loads((O/'physics_contacts.json').read_text());count=sum(bool(x['contact']) for x in contacts);videos=[]
for name in ('stereo_view','third_person'):
 cap=cv2.VideoCapture(str(O/(name+'.mp4')));n=0
 while True:
  ok,_=cap.read()
  if not ok:break
  n+=1
 fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==N+1 and fps==10;videos.append(dict(name=name,frames=n,fps=fps))
result=dict(utc=now(),local_task_pass=bool(metrics['success'] and count==0 and not metrics['runtime_error'] and metrics['elapsed_s']<=35.00001 and distance<=.200001),termination=metrics['termination_reason'],elapsed_s=metrics['elapsed_s'],final_distance_m=distance,path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),actual_goal_direction_progress_m=float(progress[-1]),cross_track_RMS_m=float(np.sqrt(np.mean(cross**2))),cross_track_peak_m=float(np.abs(cross).max()),contact_samples=count,policy_calls=N,captures=N+1,physics_samples=len(ph['simulation_time_s']),source_inferences=len(frames),paired_source_totals=total,paired_source_frames=frames,exact_packet_mask_fields=exact,exact_primary_map_arrays=maparrays,exact_slow_history_shadow_fields=shadowfields,same_live_source_shadow_calls=N,shadow_raw_request_differences=int(different.sum()),first_request_difference_frame=int(first[0]) if len(first) else None,shadow_trajectory_never_executed=True,shadow_interface_same_raw_inputs=True,whole_physical_shadow_on_off_test_not_run=True,max_exposure_delta_s=max(abs(x['raw_time_s']-v) for x in clocks for v in x['eyes'].values()),videos=videos,identity_checks=checks,future_payload_trap_pass=True,source_oldgate_not_rescored=True,formal_navigation_successrate_not_run=True,knownpose_privileged=True,whole_research_complete=False)
base=H/'masked_memory_history_run';B=json.loads((base/'policy_records.json').read_text());baseline=json.loads((H/'masked_memory_navigation_assessment.json').read_text());common=min(N,len(B));request_diff=np.any(A[:common]!=np.array([r['local']['request'] for r in B[:common]]),axis=1);changed=np.flatnonzero(request_diff);branch=int(changed[0]) if len(changed) else common;oldph=np.load(base/'physics.npz');prefixfields=[]
for k in oldph.files:
 if k not in ('raw_simulation_time_s','raw_physics_step_index'):
  np.testing.assert_equal(ph[k][:5*branch+1],oldph[k][:5*branch+1]);prefixfields.append(k)
assert json.loads((O/'execution_states.json').read_text())[:branch+1]==json.loads((base/'execution_states.json').read_text())[:branch+1]
applied_changed=np.any(ph['applied_action'][1:5*common+1]!=oldph['applied_action'][1:5*common+1],axis=1);no_reg=bool(count<=baseline['contact_samples'] and distance<=baseline['final_distance_m']+1e-6 and metrics['elapsed_s']<=baseline['elapsed_s']+1e-6)
result.update(memory_strategy='history10s',baseline_history_actual='masked_memory_history_run',common_actual_blocks=common,first_actual_request_difference_frame=int(changed[0]) if len(changed) else None,actual_request_blocks_different=int(request_diff.sum()),actual_applied_ticks_different=int(applied_changed.sum()),exact_physical_prefix_fields=prefixfields,exact_physical_prefix_samples=5*branch+1,exact_execution_prefix=branch+1,nominal_cap_no_regression=no_reg,nominal_cap_increment_pass=bool(result['local_task_pass'] and no_reg and request_diff.any() and applied_changed.any()),same_source_slow_history_shadow_not_executed=True,historical_map_peak=max(r['local']['historical_occupied']+r['local']['historical_floor'] for r in rows),RGB_reset_confound_requires_separate_scope=True)

assert all(s['same_parent_source_state_verified'] for s in shadow)
nonzero=np.flatnonzero(np.any(A[:,::2]!=0,axis=1));move=int(nonzero[0]);movement_cross=cross[move:];rms=float(np.sqrt(np.mean(movement_cross**2)));peak=float(np.abs(movement_cross).max());geometry_gate=rms<=.12 and peak<=.30
result.update(first_movement_frame=move,movement_cross_RMS_m=rms,movement_cross_peak_m=peak,movement_geometry_gate=geometry_gate,same_live_parent_controller_source_reference_equal_calls=N,terminal_distance_regression_m=distance-baseline['final_distance_m'],terminal_distance_gate=distance<=baseline['final_distance_m']+1e-6,elapsed_gate=metrics['elapsed_s']<=baseline['elapsed_s']+1e-6)
result['nominal_cap_increment_pass']=bool(result['nominal_cap_increment_pass'] and geometry_gate and different.any())
result['full_goal_triggered']=False
result['frozen_rejection_reason']='terminal distance no-regression gate failed' if not result['terminal_distance_gate'] else ('other frozen gate failed' if not result['nominal_cap_increment_pass'] else None)
checkpoints=[]
for m in (.5,1.,1.5):
 hit=np.flatnonzero(progress>=m);checkpoints.append(dict(progress_m=m,first_frame=int(hit[0]) if len(hit) else None,time_s=float(states[int(hit[0])]['time_s']) if len(hit) and 'time_s' in states[int(hit[0])] else (float(hit[0])*.1 if len(hit) else None)))
result['independent_geometric_checkpoints']=checkpoints
result['goal_entry_sampling']={}
for label,ss,gg in [('candidate',states,goal),('baseline',json.loads((base/'states.json').read_text()),goal)]:
 dd=np.linalg.norm(np.array([x['position'][:2] for x in ss])-gg,axis=1);hit=np.flatnonzero(dd<=.2);j=int(hit[0]) if len(hit) else len(dd)-1;result['goal_entry_sampling'][label]=dict(first_entry_frame=j,first_entry_distance_m=float(dd[j]),previous_distance_m=float(dd[j-1]),distance_decrease_last_block_m=float(dd[j-1]-dd[j]),last_samples_m=dd[-5:].tolist())
result['goal_entry_sampling']['not_rescored_or_interpolated']=True


from nominal_unknown_cap import nominal_unknown_action
from terminal_goal_consumer import terminal_action
terminal=False;terminal_frames=[];fast_effective_diffs=[]
for i,r in enumerate(rows):
 local=r['local'];saved=np.load(O/'policy_inputs'/f'frame_{i:03d}.npz')
 d=float(np.linalg.norm(saved['pointgoal_base_xy']));pre=nominal_unknown_action(local['uncapped_request'],local['unknown_fraction']);action,terminal=terminal_action(pre,d,terminal)
 np.testing.assert_equal(action,saved['request']);np.testing.assert_equal(action,local['request']);np.testing.assert_equal(pre,local['pre_terminal_request'])
 assert terminal==local['terminal_mode'] and d==local['terminal_goal_distance_m']
 if terminal:terminal_frames.append(i)
 if np.any(action!=pre):fast_effective_diffs.append(i)
 assert np.array_equal(action[1::2],pre[1::2]) and np.all(np.abs(action[::2])<=np.abs(pre[::2]))
assert terminal_frames and fast_effective_diffs
# Actual frozenfast prefix independently checked, without assuming two RGB restarts bit-identical.
fastdir=H/'nominal_cap_history_run';fastrows=json.loads((fastdir/'policy_records.json').read_text());fastA=np.array([r['local']['request'] for r in fastrows]);c=min(N,len(fastA));fd=np.flatnonzero(np.any(A[:c]!=fastA[:c],axis=1));branchfast=int(fd[0]) if len(fd) else c;fastph=np.load(fastdir/'physics.npz')
for k in prefixfields:np.testing.assert_equal(ph[k][:5*branchfast+1],fastph[k][:5*branchfast+1])
assert json.loads((O/'execution_states.json').read_text())[:branchfast+1]==json.loads((fastdir/'execution_states.json').read_text())[:branchfast+1]
result.update(terminal_first_frame=terminal_frames[0],terminal_first_distance_m=rows[terminal_frames[0]]['local']['terminal_goal_distance_m'],terminal_effective_request_blocks=len(fast_effective_diffs),frozenfast_actual_first_request_difference_frame=branchfast,exact_frozenfast_physical_prefix_samples=5*branchfast+1,exact_frozenfast_execution_prefix=branchfast+1,terminal_action_chain_rebuilt_calls=N)
result['terminal_consumer_gate_pass']=bool(result['nominal_cap_increment_pass'] and fast_effective_diffs)
result['old_nominal_cap_rejection_preserved']=True
result['scope']='one terminal main consumer localknownpose; no formalnav/fullgoal until accepted'
(H/'terminal_goal_navigation_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='paired_source_frames'},indent=2))
