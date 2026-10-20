"""Author post-run verification only; none of these labels feeds online policy."""
from pathlib import Path
import json,hashlib,datetime,sys
import numpy as np,cv2
from dynamic_self_mask import RobotSelfMask
from masked_source_pair import paired_sources
from stereo_geometry import restore_native
from verification_grid import VerificationGrid as PriorRGBGrid
H=Path(__file__).resolve().parent;variant=sys.argv[1];assert variant in ('B0','observe');arm='current';O=H/('observation_handoff_'+variant+'_run');now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
p=json.loads((H/'observation_handoff_navigation_protocol.json').read_text());status=json.loads((H/'observation_handoff_navigation_status.json').read_text());entry=next(x for x in status['arms'] if x['arm']==variant);assert entry['status']=='finished' and entry['exit_code']==0 and p['utc']<entry['start_utc'];checks=0
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
 if primarygrid is None:primarygrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),arm);shadowgrid=PriorRGBGrid(s['origin'],s['labels'].shape,float(s['resolution']),'history' if arm=='current' else 'current')
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

from short_obstacle_task import short_obstacle_endpoints
original=task['original_episode'];newstart,newgoal=short_obstacle_endpoints(original['start_position_world_m'],original['goal_position_world_m']);np.testing.assert_equal(newstart,task['actual_episode']['start_position_world_m']);np.testing.assert_equal(newgoal,task['actual_episode']['goal_position_world_m']);assert p['unknown_request_main_cap_rad_s']==.3 and not p['terminal_candidate_used'];assert not any('terminal_mode' in r['local'] for r in rows)
for i,r in enumerate(rows):
 np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],np.asarray(r['local']['request']).reshape(5,2))
 assert r['local']['memory_mode']==arm
 assert shadow[i]['memory_role']==('history' if arm=='current' else 'current')
mid=float(np.linalg.norm(goal-start)/2);crossed=np.flatnonzero(progress>=mid);checkpoints=[]
for metres in [.5,mid,2.3]:
 hit=np.flatnonzero(progress>=metres);checkpoints.append(dict(goal_line_progress_m=metres,first_frame=int(hit[0]) if len(hit) else None,time_s=float(states[int(hit[0])]['time_s']) if len(hit) else None))
nonzero=np.flatnonzero(np.any(A[:,::2]!=0,axis=1));move=int(nonzero[0]) if len(nonzero) else N;crossmove=cross[move:]
result.update(task='new shortened obstacle knownpose diagnosis',arm=arm,shadow_memory='history' if arm=='current' else 'current',goal_span_m=float(np.linalg.norm(goal-start)),actual_crossed_midplane=bool(len(crossed)),independent_checkpoints=checkpoints,movement_cross_RMS_m=float(np.sqrt(np.mean(crossmove**2))),movement_cross_peak_m=float(np.abs(crossmove).max()),old_6p6m_failures_preserved=True,strict_nominal_and_terminal_rejects_preserved=True,SPL_legacy_metadata_not_used=True,original_35s_budget_unchanged=True,online_inputs_current_or_past_knownpose_RGB_proprio=True,posthoc_grid_equivalence_not_online_optimization=True)
result['short_obstacle_task_pass']=bool(result.pop('local_task_pass') and len(crossed))
result['comparative_memory_benefit_not_yet_adjudicated']=True
# Replay only interface counter/schedule using actual current prefix. Never policy.act.
from observation_handoff import ObservationHandoff
gate=ObservationHandoff()
for i,r in enumerate(rows):
 t=float(r['local']['time_s']);sf=r['local']['source_frame'];nominal=np.asarray(r['local']['nominal_request']);speed=float(np.linalg.norm(np.load(O/'policy_inputs'/f'frame_{i:03d}.npz')['proprio_history'][-1,:2]));expected=gate.choose(t,speed,packets[sf],nominal) if variant=='observe' else nominal
 np.testing.assert_equal(expected,r['local']['request']);np.testing.assert_equal(expected,np.load(O/'policy_inputs'/f'frame_{i:03d}.npz')['request']);assert bool(r['local']['nominal_policy_fields_not_applied'])==bool(np.any(expected!=nominal));assert r['local']['observation_handed_off']==gate.handed_off and r['local']['observation_low_count']==gate.low_count
result.update(control_variant=variant,actual_handoff_time_s=gate.handoff_time_s,interface_current_or_past_exact=True,nominal_tracker_diagnostics_not_executed_while_override=True,shadow_differences_include_observation_override_not_memory_causal_effect=True,learned_model_not_used=True,reset_count=1)
# Same fixed old .6m clearance point-grid polyline for both arms; independent path_index.
reference=np.array(json.loads((H/'short_task_route_length_diagnostic.json').read_text())['path_xy']);vectors=np.diff(reference,axis=0);lengths=np.linalg.norm(vectors,axis=1);cumulative=np.r_[0.,np.cumsum(lengths)];errors=[];arcs=[]
for q in xy:
 u=np.clip(np.sum((q-reference[:-1])*vectors,axis=1)/np.maximum(lengths**2,1e-30),0,1);projection=reference[:-1]+u[:,None]*vectors;distance=np.linalg.norm(q-projection,axis=1);j=int(np.argmin(distance));errors.append(float(distance[j]));arcs.append(float(cumulative[j]+u[j]*lengths[j]))
result.update(frozen_reference_sha256=sha(H/'short_task_route_length_diagnostic.json'),frozen_reference_evaluator='same allsegment nearest projection, actualXY only, no controllerindex',frozen_reference_progress_end_m=arcs[-1],frozen_reference_progress_peak_m=max(arcs),frozen_reference_lateral_RMS_m=float(np.sqrt(np.mean(np.array(errors)**2))),frozen_reference_lateral_peak_m=max(errors),actual_midplane_crossed=bool(len(crossed)),frozen_reference_not_Dubins_or_formalSPL=True)

(H/('observation_handoff_'+variant+'_assessment.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='paired_source_frames'},indent=2))
