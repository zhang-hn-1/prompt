"""Post-close typed maps and independent actual progress; no policy re-call."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np,cv2
from typed_hypothesis_grid import TypedHypothesisGrid
from route_geometry_audit import hazard_clearance
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();status=json.loads((H/'typed_hypothesis_memory_stage_status_v1.json').read_text());assert status['stage']=='BOTH_ACTUAL_ARMS_CLOSED_NEEDS_VERIFICATION';records={};trajectories={};actions={};seeds={};fields=0;frozen_ref=None;prefix_sources={}
def project(xy,path):
 a=path[:-1];v=np.diff(path,axis=0);len2=(v*v).sum(1);lens=np.sqrt(len2);arc=np.r_[0,np.cumsum(lens)];errors=[];progress=[]
 for x in xy:
  u=np.clip(((x-a)*v).sum(1)/np.maximum(len2,1e-15),0,1);dist=np.linalg.norm(x-(a+u[:,None]*v),axis=1);j=int(np.argmin(dist));errors.append(dist[j]);progress.append(arc[j]+u[j]*lens[j])
 return np.array(errors),np.array(progress)
for arm in ['B0','C1']:
 D=H/f'typed_hypothesis_memory_{arm}_run_v1';seal=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(f)==s for f,s in seal['artifact_sha256'].items());states=json.loads((D/'states.json').read_text());policy=json.loads((D/'policy_records.json').read_text());seed=dict(np.load(D/'immutable_possible_set_birth.npz'));seeds[arm]=seed;start=np.array(states[0]['position'])[:2];g=TypedHypothesisGrid(start-20,retention_s=10 if arm=='B0' else 60);nfields=0;prefix_sources[arm]={}
 for i,r in enumerate(policy):
  t=float(r['local']['time_s']);src=dict(np.load(D/'sensor_packets'/f"source_{r['local']['source_frame']:03d}.npz"));src.update(world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int));g.set_source(src)
  if g.birth is None and float(seed['time_s'])<=t:g.seed(seed,t)
  g.update_points(np.empty((0,3)),np.array(states[i]['position']),t);saved=np.load(D/'maps'/f'frame_{i:03d}.npz')
  expected=dict(labels=g.labels,provenance=g.provenance(),floor_time=g.memory.floor_time,hazard_time=g.memory.hazard_time,hypothesis_birth_time=g.hypothesis_birth_time,task_ground_prior=g.task_ground_prior,safe_start_prior=g.safe_start_prior)
  for k,v in expected.items():np.testing.assert_equal(v,saved[k]);nfields+=1
  assert not np.isfinite(saved['floor_time']).any() and not np.isfinite(saved['hazard_time']).any() and r['local']['moving_geometry_consumed']==False
 xy=np.array([s['position'][:2] for s in states]);traj_time=np.array([s['time_s'] for s in states]);trajectories[arm]=(traj_time,xy,states);actions[arm]=np.array([r['local']['request'] for r in policy]);metrics=json.loads((D/'metrics.json').read_text());identity=json.loads((D/'identity.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text());contacts=json.loads((D/'contacts.json').read_text())
 if frozen_ref is None:
  valid=[x for x in policy if x['local']['local_plan_created'] and float(x['local']['time_s'])>=float(seed['time_s']) and x['local']['plan'].get('route_xy')];assert valid,'no complete post-source geometric route';frozen_ref=np.asarray(valid[0]['local']['plan']['route_xy'],float)
 err,progress=project(xy,frozen_ref);clear=float(hazard_clearance(xy,np.array([[0.,0.]]))-.75);videos={}
 for name in ['stereo_view','third_person']:
  cap=cv2.VideoCapture(str(D/(name+'.mp4')));count=0
  while True:
   ok,im=cap.read()
   if not ok:break
   count+=1
  cap.release();assert count==len(clocks);videos[name]=count
 rec=dict(termination_reason=metrics['termination_reason'],actual_success=bool(metrics['success']),elapsed_s=metrics['elapsed_s'],actual_final_distance_m=float(np.linalg.norm(xy[-1]-[2.2,0.])),actual_goal_axis_progress_m=float(xy[-1,0]-start[0]),actual_reference_progress_m=float(progress[-1]),actual_reference_progress_max_m=float(progress.max()),frozen_reference_lateral_RMS_m=float(np.sqrt((err*err).mean())),frozen_reference_lateral_peak_m=float(err.max()),actual_path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),actual_obstacle_segment_passed=bool(xy[:,0].max()>.75),contact_frames=sum(c['contact'] for c in contacts),collision_frames=sum(c['collision'] for c in contacts),continuous_true_body_clearance_m=clear,actual_handoff_time_s=identity['observation_handoff_time_s'],typed_map_fields_exact=nfields,measured_free_and_occupied_cells=0,model_birth_time_s=float(seed['time_s']),decoded_video_frames=videos,source_updates=len(list((D/'source_predictions').glob('frame_*.npz'))),source_network_forwards=2*len(list((D/'source_predictions').glob('frame_*.npz')))+1,knownpose_privileged=True,task_budget_s=60.,not_original_E1_or_formal35=True)
 records[arm]=rec;fields+=nfields
np.savez_compressed(H/'typed_hypothesis_memory_frozen_evaluator_reference_v3.npz',xy=frozen_ref,from_B0_first_post_source_reference=True,independent_of_path_index=True)
limit=min(seeds['B0']['time_s'],seeds['C1']['time_s'])+10;bt,bxy,bs=trajectories['B0'];ct,cxy,cs=trajectories['C1'];n=min(int((bt<limit).sum()),int((ct<limit).sum()),len(actions['B0']),len(actions['C1']));physical_error=0.
for k in ['position','quaternion','velocity','angular_velocity','joints','joint_velocity']:
 physical_error=max(physical_error,float(np.max(np.abs(np.array([s[k] for s in bs[:n]])-np.array([s[k] for s in cs[:n]])))))
request_equal=np.array_equal(actions['B0'][:n],actions['C1'][:n]);seed_equal=set(seeds['B0'])==set(seeds['C1']) and all(np.array_equal(seeds['B0'][k],seeds['C1'][k]) for k in seeds['B0']);prefix_match=physical_error<=1e-5 and request_equal and seed_equal
b=records['B0'];c=records['C1'];checks=dict(goal_reached=c['actual_final_distance_m']<=.4 and c['actual_success'],no_contact=c['contact_frames']==0,through_obstacle_segment=c['actual_obstacle_segment_passed'],actual_goal_axis_progress_ge4=c['actual_goal_axis_progress_m']>=4,continuous_body_clearance_ge0=c['continuous_true_body_clearance_m']>=0,tracking_RMS_not_worse=c['frozen_reference_lateral_RMS_m']<=b['frozen_reference_lateral_RMS_m']+1e-9,physical_action_and_model_prefix_matched=prefix_match,improvement=not b['actual_success'] or c['actual_goal_axis_progress_m']>b['actual_goal_axis_progress_m']+.1 or c['continuous_true_body_clearance_m']>b['continuous_true_body_clearance_m']+.05)
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,6));ax.plot(frozen_ref[:,0],frozen_ref[:,1],'k--',label='common B0 initial evaluator reference');theta=np.linspace(0,2*np.pi,100);ax.fill(.35*np.cos(theta),.35*np.sin(theta),color='gray',alpha=.5,label='post-close physical cylinder')
for arm in ['B0','C1']:
 xy=trajectories[arm][1];ax.plot(xy[:,0],xy[:,1],label=arm+' actual root trajectory')
ax.scatter(2.2,0,marker='*',s=100,color='green');ax.axis('equal');ax.grid();ax.legend();ax.set_title('Typed RGB hypothesis retention / actual local task / not formal35');ax.set_xlabel('world X m');ax.set_ylabel('world Y m');fig.savefig(H/'typed_hypothesis_memory_actual_v3.png',dpi=160);plt.close(fig)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_LOCAL_IMMUTABLE_HYPOTHESIS_RETENTION_ONLY' if all(checks.values()) else 'REJECT_LOCAL_IMMUTABLE_HYPOTHESIS_RETENTION',arms=records,checks=checks,prefix_until_s=float(limit),prefix_frames=n,maximum_physical_prefix_error=physical_error,requests_prefix_exact=request_equal,initial_model_hypothesis_packet_equal=seed_equal,strict_causal_AB_claim=prefix_match,all_typed_map_fields_exact=fields,GT_only_post_close=True,moving_geometry_not_consumed=True,model_hypothesis_not_observation=True,world_model_increment_not_tested=True,estimated_pose_not_admitted=True,formal35_not_triggered=True,whole_research_complete=False,code_sha256=sha(__file__))
(H/'typed_hypothesis_memory_assessment_v3.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
