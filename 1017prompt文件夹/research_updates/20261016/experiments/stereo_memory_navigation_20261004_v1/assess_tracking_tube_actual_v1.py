"""Closed actual C2, independent geometric progress and typed-memory evidence.
Historical C1 is design comparator, never fresh validation or another plan's label.
"""
from pathlib import Path
import json,hashlib,datetime
import numpy as np,cv2
from typed_hypothesis_grid import TypedHypothesisGrid
from route_geometry_audit import hazard_clearance
from raster_consistent_bridge import bridge_screen
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((H/'tracking_tube_stage_status_v1.json').read_text());assert status['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION' and status['returncode']==0
D=H/'tracking_tube_D1_run_v1';seal=D/'artifact_manifest_post_close.json';assert sha(seal)==status['post_close_manifest_sha256'];manifest=json.loads(seal.read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in manifest.items())
old=H/'typed_hypothesis_memory_C1_run_v1';ref=np.load(H/'typed_hypothesis_memory_frozen_evaluator_reference_v3.npz')['xy']
def project(xy,path):
 a=path[:-1];v=np.diff(path,axis=0);ll=(v*v).sum(1);arc=np.r_[0,np.cumsum(np.sqrt(ll))];errors=[];progress=[]
 for x in xy:
  u=np.clip(((x-a)*v).sum(1)/np.maximum(ll,1e-15),0,1);dist=np.linalg.norm(x-(a+u[:,None]*v),axis=1);j=int(dist.argmin());errors.append(dist[j]);progress.append(arc[j]+u[j]*np.sqrt(ll[j]))
 return np.asarray(errors),np.asarray(progress)
def load(d):
 states=json.loads((d/'states.json').read_text());rows=json.loads((d/'policy_records.json').read_text());xy=np.asarray([s['position'][:2] for s in states]);seed=dict(np.load(d/'immutable_possible_set_birth.npz'));return states,rows,xy,seed
states,rows,xy,seed=load(D);os,orr,oxy,oseed=load(old);g=TypedHypothesisGrid(xy[0]-20,retention_s=60);fields=0;aliases=[];guard_calls=0;zero_frames=[]
for i,r in enumerate(rows):
 l=r['local'];t=float(l['time_s']);source=dict(np.load(D/'sensor_packets'/f"source_{l['source_frame']:03d}.npz"));source.update(world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int));g.set_source(source)
 if g.birth is None and float(seed['time_s'])<=t:g.seed(seed,t)
 g.update_points(np.empty((0,3)),np.asarray(states[i]['position']),t);saved=np.load(D/'maps'/f'frame_{i:03d}.npz')
 for k,v in dict(labels=g.labels,provenance=g.provenance(),floor_time=g.memory.floor_time,hazard_time=g.memory.hazard_time,hypothesis_birth_time=g.hypothesis_birth_time,task_ground_prior=g.task_ground_prior,safe_start_prior=g.safe_start_prior).items():np.testing.assert_equal(v,saved[k]);fields+=1
 assert not np.isfinite(saved['floor_time']).any() and not np.isfinite(saved['hazard_time']).any() and not l['moving_geometry_consumed']
 if np.all(np.asarray(l['request'])==0):zero_frames.append(i)
 decision=l['reference_decision'];guard=None if decision is None else decision.get('guard')
 if l['guard_measured_this_call'] and guard is not None:
  assert abs(decision['measured_time_s']-t)<1e-5;guard_calls+=1;screen=bridge_screen(dict(time_s=t,guard=guard),t,float(saved['resolution']))
  hazards=saved['origin']+np.argwhere(saved['labels']==1)*float(saved['resolution'])
  for key in ['suffix','bridge']:
   measured=hazard_clearance(np.asarray(guard[key+'_xy']),hazards);np.testing.assert_allclose(measured,guard[key+'_clearance_m'],atol=1e-9)
  assert (decision['decision']=='retain')==guard['retention_necessary_screen_pass']
 if l['local_plan_created'] and l['plan'].get('status')=='success':
  savedplan=np.load(D/'local_plans'/f'frame_{i:03d}.npz');hazards=saved['origin']+np.argwhere(saved['labels']==1)*float(saved['resolution']);clearance=hazard_clearance(savedplan['pose'][:,:2],hazards);assert l['plan']['planning_clearance_m']==.8707106781186548;aliases.append(dict(frame=i,time_s=t,continuous_nominal_clearance_m=clearance,empirical_not_certified=True))
metrics=json.loads((D/'metrics.json').read_text());contacts=json.loads((D/'contacts.json').read_text());identity=json.loads((D/'identity.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text());err,progress=project(xy,ref);videos={}
for name in ['stereo_view','third_person']:
 cap=cv2.VideoCapture(str(D/(name+'.mp4')));n=0
 while True:
  ok,im=cap.read()
  if not ok:break
  n+=1
 cap.release();assert n==len(clocks);videos[name]=n
n=min(len(rows),len(orr));equal=[]
for i in range(n):
 same=all(np.array_equal(states[i][k],os[i][k]) for k in ['position','quaternion','velocity','angular_velocity','joints','joint_velocity']) and np.array_equal(rows[i]['local']['request'],orr[i]['local']['request']);equal.append(same)
first_difference=next((i for i,v in enumerate(equal) if not v),None);prefix=n if first_difference is None else first_difference
seed_equal=set(seed)==set(oseed) and all(np.array_equal(seed[k],oseed[k]) for k in seed)
requests=np.asarray([r['local']['request'] for r in rows[:n]]);oldrequests=np.asarray([r['local']['request'] for r in orr[:n]])
ph=np.load(D/'physics.npz');oph=np.load(old/'physics.npz');nt=min(len(ph['applied_action']),len(oph['applied_action']));applied_diff=np.any(ph['applied_action'][:nt]!=oph['applied_action'][:nt],axis=1);latency=np.asarray([c['capture_partial_wall_s'] for c in clocks]);source_latency=np.asarray([c['rgb_source_wall_s'] for c in clocks if c['source_predicted']]);skew=max(abs(c['eyes']['left']-c['eyes']['right']) for c in clocks)
actual=dict(termination_reason=metrics['termination_reason'],actual_success=bool(metrics['success']),elapsed_s=metrics['elapsed_s'],actual_final_distance_m=float(np.linalg.norm(xy[-1]-[2.2,0.])),actual_goal_axis_progress_m=float(xy[-1,0]-xy[0,0]),actual_reference_progress_m=float(progress[-1]),frozen_reference_lateral_RMS_m=float(np.sqrt((err*err).mean())),frozen_reference_lateral_peak_m=float(err.max()),actual_path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),actual_obstacle_segment_passed=bool(xy[:,0].max()>.75),contact_frames=sum(c['contact'] for c in contacts),collision_frames=sum(c['collision'] for c in contacts),continuous_true_body_clearance_m=float(hazard_clearance(xy,np.array([[0.,0.]]))-.75),actual_handoff_time_s=identity['observation_handoff_time_s'],typed_map_fields_exact=fields,measured_free_and_occupied_cells=0,decoded_video_frames=videos,camera_eye_time_skew_s=skew,capture_partial_wall_p50_s=float(np.median(latency)),capture_partial_wall_p95_s=float(np.quantile(latency,.95)),source_wall_p50_s=float(np.median(source_latency)),source_wall_p95_s=float(np.quantile(source_latency,.95)),real_time_replay_equivalent=False,source_updates=len(list((D/'source_predictions').glob('frame_*.npz'))),knownpose_privileged=True,task_budget_s=60.,not_original_E1_or_formal35=True)
checks=dict(goal_reached=actual['actual_success'] and actual['actual_final_distance_m']<=.4,goal_axis_progress_ge4=actual['actual_goal_axis_progress_m']>=4.,actual_obstacle_segment_passed=actual['actual_obstacle_segment_passed'],no_contact=actual['contact_frames']==0,continuous_body_proxy_clearance_ge0=actual['continuous_true_body_clearance_m']>=0.,candidate_exercised=bool(aliases),all_installed_nominal_references_clear_point8=bool(aliases) and all(e['continuous_nominal_clearance_m']>=.8-1e-8 for e in aliases),effective_request_change=bool(np.any(requests!=oldrequests)))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,6));ax.plot(ref[:,0],ref[:,1],'k--',label='frozen complete B0 evaluator reference');theta=np.linspace(0,2*np.pi,100);ax.fill(.35*np.cos(theta),.35*np.sin(theta),color='gray',alpha=.5,label='post-close physical cylinder');ax.plot(oxy[:,0],oxy[:,1],label='historical C1 actual');ax.plot(xy[:,0],xy[:,1],label='fresh D1 actual');ax.scatter(2.2,0,marker='*',s=100,color='green');ax.axis('equal');ax.grid();ax.legend();ax.set_title('One empirical tracking-tube planner / 60s local task / known pose');fig.savefig(H/'tracking_tube_actual_v1.png',dpi=160);plt.close(fig)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_LOCAL_EMPIRICAL_TRACKING_TUBE_ONLY' if all(checks.values()) else 'REJECT_LOCAL_EMPIRICAL_TRACKING_TUBE',actual=actual,checks=checks,new_reference_installs=aliases,original_guard_calls_checked=guard_calls,manifest_checks=len(manifest),historical_C1_not_fresh=True,old_strict_AB_gate_false_preserved=True,initial_model_hypothesis_packet_equal=seed_equal,physical_and_request_prefix_frames=prefix,first_physical_or_request_difference_frame=first_difference,request_different_frames=int(np.any(requests!=oldrequests,axis=1).sum()),applied_different_ticks=int(applied_diff.sum()),first_applied_difference_tick=next((i for i,v in enumerate(applied_diff) if v),None),source_matched_causal_AB_claim=False,full_source_prefix_AB_not_verified=True,post_close_GT_only_for_actual_evaluation=True,no_stateful_policy_recall=True,no_future_plan_feedback_label=True,world_model_increment_not_tested=True,estimated_pose_not_admitted=True,formal35_not_triggered=True,whole_research_complete=False,code_sha256=sha(__file__))
file=H/'tracking_tube_actual_assessment_v1.json';assert not file.exists();file.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
