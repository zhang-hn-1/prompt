from pathlib import Path
import sys,json,hashlib,datetime,math,numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
import scripts.run_pointgoal_benchmark as runner
from benchmark.policies import PolicyObservation
from prior_rgb_navigation import PriorRGBPolicy
from tracking_tube_planner import with_tracking_tube
from command_domain_cap import with_command_domain_cap
from persistent_core_grid import PersistentCoreGrid
from observation_clock_adapter import observe_clock_without_act
D=H/'negative_ray_memory_R1_run_v2';O=H/'discrete_lookahead_chain_audit_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=json.loads((D/'policy_records.json').read_text());policy=with_command_domain_cap(with_tracking_tube(PriorRGBPolicy))(runner.load_action_protocol(P/'configs/data_contract.yaml'),memory_mode='current');policy.reset(None);init=np.load(D/'estimated_poses/frame_000.npz');policy.grid=PersistentCoreGrid(init['root_position_world_m'][:2]-20,60)
for i in range(140):
 q=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');observe_clock_without_act(policy,dict(frame=i,time_s=float(q['time_s']),points_world=np.empty((0,3)),root_xyz=q['root_position_world_m']),float(q['time_s']))
seed=dict(np.load(D/'frozen_memory_seed.npz'));policy.grid.seed(seed,float(seed['time_s']));trace=[]
for i in range(140,265):
 q=np.load(D/'policy_inputs'/f'frame_{i:03d}.npz');pose=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');source=dict(np.load(D/'sensor_packets'/f'source_{i-i%5:03d}.npz'));source.update(world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int));policy.set_source_packet(source);policy.set_current_packet(dict(frame=i,time_s=float(pose['time_s']),points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m']))
 obs=PolicyObservation(**{k:q[k] for k in ['left_rgb','right_rgb']} ) if False else PolicyObservation(stereo_left_rgb=q['left_rgb'],stereo_right_rgb=q['right_rgb'],proprio_history=q['proprio_history'],action_history=q['action_history'],position_world_xy=q['position_world_xy'],yaw_world_rad=float(q['yaw_world_rad']),quaternion_world_wxyz_history=q['quaternion_history'])
 a=policy.act(obs,q['pointgoal_base_xy']);np.testing.assert_equal(a,rows[i]['request']);diag=policy.tracker.tracking_diagnostics;rec=dict(frame=i,request=a.tolist(),diagnostics=diag,geometry_counterfactual=None)
 if diag.get('valid'):
  reference,xy=policy.tracker._require_reference();j=policy.tracker._path_index;direction=reference[j].drive_direction;end=j
  while end+1<len(reference) and reference[end].drive_direction==direction:end+=1
  start=j-1 if j>0 and reference[j-1].drive_direction==direction else j
  path=xy[start:end+1];dl=np.diff(path,axis=0);ll=np.linalg.norm(dl,axis=1)
  if len(ll) and (ll>0).all():
   ss=np.r_[0,np.cumsum(ll)];u=np.clip(((obs.position_world_xy-path[:-1])*dl).sum(1)/ll**2,0,1);pp=path[:-1]+u[:,None]*dl;idx=int(np.linalg.norm(pp-obs.position_world_xy,axis=1).argmin());s=float(ss[idx]+u[idx]*ll[idx]);target_s=min(float(ss[-1]),s+diag['lookahead_requested_m']);k=min(len(ll)-1,int(np.searchsorted(ss,target_s,side='right')-1));target=path[k]+dl[k]*((target_s-ss[k])/ll[k]);dd=target-obs.position_world_xy;cy,sy=np.cos(obs.yaw_world_rad),np.sin(obs.yaw_world_rad);body=np.array([cy*dd[0]+sy*dd[1],-sy*dd[0]+cy*dd[1]]);curve=2*body[1]/np.dot(body,body) if np.dot(body,body)>1e-9 else 0
   rec['geometry_counterfactual']=dict(exact_target_xy=target.tolist(),same_adaptive_requested_lookahead_m=diag['lookahead_requested_m'],old_discrete_accumulated_m=diag['lookahead_accumulated_m'],overshoot_m=diag['lookahead_accumulated_m']-diag['lookahead_requested_m'],target_difference_m=float(np.linalg.norm(target-[diag['lookahead_x_m'],diag['lookahead_y_m']])),geometric_curvature_1_m=float(curve),old_curvature_1_m=diag['geometric_pure_pursuit_curvature_1_m'],no_new_trajectory_label=True)
 trace.append(rec)
(O/'trace.json').write_text(json.dumps(trace,indent=2)+'\n');valid=[r for r in trace if r['geometry_counterfactual']];overshoot=np.array([r['geometry_counterfactual']['overshoot_m'] for r in valid]);diff=np.array([r['geometry_counterfactual']['target_difference_m'] for r in valid]);r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),factual_requests_exact=len(trace),valid_tracking_rows=len(valid),old_discrete_overshoot_peak_m=float(overshoot.max()),target_difference_peak_m=float(diff.max()),target_difference_median_m=float(np.median(diff)),protocol_bound_rows=sum(r['diagnostics'].get('angle_target_at_bound',False) for r in trace),not_online=True,not_new_navigation=True,no_candidate_pass_claimed=True,code_sha256=sha(__file__),input_sha256={str(D/n):sha(D/n) for n in ['policy_records.json','frozen_memory_seed.npz','artifact_manifest_post_close.json']},whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
