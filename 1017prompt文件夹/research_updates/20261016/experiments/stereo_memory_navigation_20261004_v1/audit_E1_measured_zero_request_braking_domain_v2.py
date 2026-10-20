"""One inventory of actual zero-request stopping, no fitted bound or online prediction."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from precommand_stop_qualification_v2 import qualify
H=Path(__file__).resolve().parent;O=H/'E1_measured_zero_request_braking_domain_v2';O.mkdir(exist_ok=False)
names=['original_E1_static_sparse_memory_navigation_run_v1','original_E1_route_side_navigation_resumed_run_v2','original_E1_tube_viability_navigation_run_v1']
p=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),runs=names,event='After actual nonzero main request; maximal consecutive exactly-zero two-axis requests lasting>=5 capture intervals; initial settling excluded',qualification='first five consecutive 10Hz actual worldXY speed<.05m/s; rebound resets0; event interval ends before next nonzero command, but includes its current pre-command capture; no physics executed by next nonzero command is included',no_joint_crossing_substitution=True,include_incomplete_stops=True,no_fit_or_upperbound_admission=True,all_old_data_development_not_fresh=True,no_new_physics_or_NN=True,whole_research_complete=False,inputs={})
for n in names:
 for f in ['physics.npz','requests.npz','policy_records.json']:
  path=H/n/f;p['inputs'][str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
(O/'protocol.json').write_text(json.dumps(p,indent=2)+'\n');events=[]
for n in names:
 D=H/n;z=np.load(D/'physics.npz');r=np.load(D/'requests.npz')['requests'];rows=json.loads((D/'policy_records.json').read_text());xy=z['position_world_xy'].astype(float);v=z['linear_velocity_world_m_s'].astype(float);q=z['root_quaternion_world_wxyz'];N=len(r);zero=np.all(r.reshape(N,5,2)==0,axis=(1,2));i=1
 while i<N:
  if not zero[i] or zero[i-1]:i+=1;continue
  lo=i
  while i<N and zero[i]:i+=1
  hi=i
  if hi-lo<5:continue
  info=qualify(np.linalg.norm(v[::5,:2],axis=1),lo,hi);qualified=info['qualified_frame'];rebounds=info['rebound_resets']
  endpoint=5*qualified if qualified is not None else 5*hi;start=5*lo;d=xy[start:endpoint+1]-xy[start];last=r[lo-1].reshape(5,2)[-1];enddist=np.linalg.norm(d,axis=1)
  events.append(dict(run=n,start_frame=lo,end_frame_exclusive=hi,initial_main_command=float(last[0]),initial_aux_request_rad=float(last[1]),initial_speed_m_s=float(np.linalg.norm(v[start,:2])),initial_position=xy[start].tolist(),initial_joint_rad=z['joint_position_rad'][start].astype(float).tolist(),initial_joint_velocity_rad_s=z['joint_velocity_rad_s'][start].astype(float).tolist(),initial_quaternion=q[start].astype(float).tolist(),qualified_frame=qualified,actual_qualification_delay_s=None if qualified is None else float(z['simulation_time_s'][endpoint]-z['simulation_time_s'][start]),displacement_until_qualification_or_censor_m=float(enddist[-1]),peak_displacement_until_qualification_or_censor_m=float(enddist.max()),path_until_qualification_or_censor_m=float(np.linalg.norm(np.diff(xy[start:endpoint+1],axis=0),axis=1).sum()),rebound_resets=rebounds,qualified=qualified is not None,censored_without_qualification=qualified is None,pending_direction_at_start=rows[lo]['tracker_state_after_act']['pending_direction'],plan_status_at_start=rows[lo]['plan'].get('status'),precommand_capture_boundary_inclusive=True,no_new_command_physics_in_stop_tail=True,no_future_state_used_by_actual_actor=True))
a=dict(decision='ACTUAL_STOPPING_DOMAIN_INVENTORY_ONLY_NOT_SAFE_ENVELOPE',events=len(events),qualified=sum(e['qualified'] for e in events),censored=sum(not e['qualified'] for e in events),observed_qualified_peak_displacement_m=max(e['peak_displacement_until_qualification_or_censor_m'] for e in events if e['qualified']),old_v1_excluded_precommand_fifth_capture_REJECTED_inventory_only=True,not_worstcase_safe_bound=True,no_fresh_heldout_or_recursive_prediction=True,no_controller_admission=True,no_new_matrix_or_thresholds=True,whole_research_complete=False)
(O/'events.json').write_text(json.dumps(events,indent=2)+'\n');(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
