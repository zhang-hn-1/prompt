"""Magnitude, no-route causality and presealed-gate scope; no new controls."""
import json,hashlib
from pathlib import Path
import numpy as np
from route_geometry_audit import project
H=Path(__file__).resolve().parent;read=lambda p:json.loads(p.read_text())
A=H/'online_bank_hold_navigation_run';B=H/'route_clearance_navigation_run'
ra=read(A/'vision_records.json');rb=read(B/'vision_records.json');pa=np.load(A/'physics.npz');pb=np.load(B/'physics.npz')
states=read(A/'policy_states.json');path=np.asarray(states[200]['path_xy']);start_progress=project([pa['position_world_xy'][1000]],path)[0][0]
def geometry(p):
 xy=p['position_world_xy'][1000:];time=p['simulation_time_s'][1000:]
 progress,error,_=project(xy,path)
 arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
 checkpoints=[]
 for distance in (.5,1.):
  at=start_progress+distance;seg=min(int(np.searchsorted(arc,at,side='right'))-1,len(path)-2)
  point=path[seg]+(path[seg+1]-path[seg])*(at-arc[seg])/(arc[seg+1]-arc[seg])
  actual_distance=np.linalg.norm(xy-point,axis=1);idx=int(actual_distance.argmin())
  checkpoints.append(dict(along_event_reference_m=distance,checkpoint_world_xy=point.tolist(),
      closest_actual_distance_m=float(actual_distance[idx]),closest_time_s=float(time[idx]),actual_xy=xy[idx].tolist()))
 return dict(progress_gain_peak_m=float((progress-start_progress).max()),progress_gain_final_m=float(progress[-1]-start_progress),
     first_one_metre_segment_completed=bool(np.any(progress-start_progress>=1.-1e-8)),checkpoints=checkpoints,
     scope='fixed event-time B0 path for geometry evaluation only; no path_index; nearest checkpoint distance is not a success gate')
requests_a=np.asarray([r['request'] for r in ra]);requests_b=np.asarray([r['request'] for r in rb])
before_stop=280*5+1
changed=np.flatnonzero(np.any(requests_a[200:280]!=requests_b[200:280],axis=1))+200
physics_changed=np.flatnonzero(np.any(pa['position_world_xy']!=pb['position_world_xy'],axis=1))
target_changes=[i for i in range(200,280) if ra[i]['target_base_xy']!=rb[i]['target_base_xy']]
result=dict(planning_event_frame=200,first_actual_request_change_frame=int(changed[0]),
  changed_moving_request_frames=changed.tolist(),prestop_changed_requests_only_aux=bool(np.array_equal(requests_a[200:280,::2],requests_b[200:280,::2])),
  max_raw_aux_difference_before_first_stop_rad=float(np.max(np.abs(requests_a[200:280,1::2]-requests_b[200:280,1::2]))),
  max_raw_aux_difference_before_first_stop_deg=float(np.degrees(np.max(np.abs(requests_a[200:280,1::2]-requests_b[200:280,1::2])))),
  max_applied_aux_difference_before_first_stop_rad=float(np.max(np.abs(pa['applied_action'][:before_stop,1]-pb['applied_action'][:before_stop,1]))),
  max_actual_xy_separation_before_first_stop_m=float(np.max(np.linalg.norm(pa['position_world_xy'][:before_stop]-pb['position_world_xy'][:before_stop],axis=1))),
  first_physics_xy_different_tick=int(physics_changed[0]),target_changed_moving_blocks=len(target_changes),
  no_route_count=sum(r['target_base_xy'] is None for r in rb),veto_count=sum(r['stopping_consumer']['stop_veto'] for r in rb),
  all_final70_blocks_no_route=bool(all(r['target_base_xy'] is None and not np.asarray(r['baseline_final_request'])[::2].any() for r in rb[280:])),
  no_direction_confirmation_wait=bool(all(r['low_speed_count']==0 for r in rb[280:])),
  baseline_event_reference=geometry(pa),candidate_event_reference=geometry(pb),
  frozen_gate_scope_defect=dict(local_window_s=6.,baseline_veto_blocks_20_to26_s=sum(r['stopping_consumer']['stop_veto'] for r in ra[200:260]),
      strictly_fewer_than_zero_is_possible=False,old_protocol_not_changed=True,
      consequence='presealed fewer-veto test cannot pass in this event window; retain failure, do not call it evidence of control regression. Independent meaningful .05m benefit and whole-progress gates also fail.'),
  conclusion='Most moving outputs remain saturated/equal despite changed route. Five aux requests differ minutely; main stays equal until new no-route stop28s, .4s earlier than B0 veto. Zero-veto count is upstream parking, not improved navigation.',
  next='Reject the sole candidate and do not repeat with a radius/cap/PID tweak. Return to archived aligned RGB visibility/truepartner/matching cause audit.',
  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__),A/'physics.npz',B/'physics.npz',A/'vision_records.json',B/'vision_records.json')})
(H/'route_clearance_effect_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'},indent=2))
