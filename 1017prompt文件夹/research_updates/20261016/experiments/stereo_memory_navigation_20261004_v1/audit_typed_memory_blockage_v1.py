"""Closed bounded cause audit: source discrepancy, lattice failure, inflated start."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;Ds={a:H/f'typed_hypothesis_memory_{a}_run_v1' for a in ['B0','C1']};seeds={a:dict(np.load(d/'immutable_possible_set_birth.npz')) for a,d in Ds.items()};diff={}
for k in seeds['B0']:
 a,b=seeds['B0'][k],seeds['C1'][k]
 if not np.array_equal(a,b):diff[k]=dict(B0_shape=list(np.shape(a)),C1_shape=list(np.shape(b)))
sets={a:set(map(tuple,np.round(s['possible_xy'],9))) for a,s in seeds.items()};D=Ds['C1'];rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());events=[];counts={};previous=None
for i,r in enumerate(rows):
 l=r['local'];status=l['plan'].get('status','none');counts[status]=counts.get(status,0)+1
 if status==previous:continue
 previous=status
 if status not in ['unreachable','observed_obstacle_blocks_route']:continue
 m=np.load(D/'maps'/f'frame_{i:03d}.npz');pos=np.array(states[i]['position'][:2]);origin=m['origin'];res=float(m['resolution']);cell=np.rint((pos-origin)/res).astype(int);labels=m['labels'];dist=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*res;target=np.array([2.2,0.]);gc=np.rint((target-origin)/res).astype(int)
 events.append(dict(frame=i,time_s=l['time_s'],legacy_status_name=status,legacy_observed_string_actually_model_hypothesis=True,root_xy=pos.tolist(),root_heading_rad=float(np.load(D/'policy_inputs'/f'frame_{i:03d}.npz')['yaw_world_rad']),root_grid_clearance_m=float(dist[tuple(cell)]),goal_grid_clearance_m=float(dist[tuple(gc)]),frozen_Astar_clearance_m=.6,start_meets_Astar_clearance=bool(dist[tuple(cell)]>=.6-1e-8),goal_meets_Astar_clearance=bool(dist[tuple(gc)]>=.6-1e-8),physical_circle_proxy_clearance_m=float(np.linalg.norm(pos)-.75),raw_request_zero=bool(np.all(np.array(l['request'])==0)),actual_XY_speed=float(np.linalg.norm(states[i]['velocity'][:2])),pending_direction=l['reference_decision'].get('state_after',{}).get('pending_direction'),guard_measured_current_call=l['guard_measured_this_call']))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_birth_packet_different_fields=diff,B0_only_hypothesis_XY=sorted(sets['B0']-sets['C1']),C1_only_hypothesis_XY=sorted(sets['C1']-sets['B0']),C1_plan_status_frames=counts,C1_first_status_transitions=events,no_assumption_all_stops_are_collision=True,no_stale_tracking_fields_used=True,no_model_or_controller_or_gate_change=True,no_repeat_actual_sequence=True,strict_AB_gate_failure_preserved=True,next_control_layer='First resolve measured-state feasible local connector/reference consumption when lattice rebuild fails and inflated start is blocked; preserve model/observation distinction, do not tune memory TTL to hide it.',not_navigation_or_whole_research_pass=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(H/'typed_memory_blockage_audit_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
