"""Throwaway six-query necessary geometry probe; no actual policy or actuator changes."""
from pathlib import Path
import json,hashlib,copy,time,numpy as np
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from prior_rgb_navigation import PriorRGBPolicy
from scripts.run_pointgoal_benchmark import load_action_protocol
import execution_domain_four_point_consumer_v2 as C
H=Path(__file__).resolve().parent;D=H/'original_E1_tube_viability_navigation_run_v1';O=H/'E1_same_direction_geometry_probe_v1';O.mkdir(exist_ok=False);p=json.loads((H/'E1_same_direction_geometry_probe_protocol_v1.json').read_text())
for f,s in p['sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());z=np.load(D/'physics.npz');protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));model=PriorRGBPolicy(protocol,'current').tracker.drive_model;goal=np.array([3.300000005960465,5.960465010446114e-9]);results=[];original=C.supported_requests;commands=original();forward=tuple(q for q in commands if q[0]>0);assert len(forward)==3 and len(commands)==4
for i in p['frames']:
 row=rows[i];q=np.load(D/'maps'/f'frame_{i:03d}.npz');xy=np.array(states[i]['position'][:2]);yaw=float(z['yaw_world_rad'][5*i]);assert row['tracker_state_after_act']['active_direction']=='forward';arms=[]
 for candidate in [False,True]:
  grid=RouteSideCommitmentGrid(q['origin'],10,static_scene_assumption=True);grid.labels=q['labels'].copy();grid.codes=q['codes'].copy();grid.time=float(q['time_s']);grid.commitment=copy.deepcopy(row['route_side_commitment']);grid._axis_start=np.array(grid.commitment['axis_start']);grid._axis=np.array(grid.commitment['axis']);grid._normal=np.array([-grid._axis[1],grid._axis[0]]);grid._goal=goal.copy();before=grid.labels.copy();begin=time.perf_counter()
  try:
   if candidate:C.supported_requests=lambda:forward
   plan,meta=C.local_curve_for_direction_tube(grid,xy,yaw,goal,model,initial_direction='forward')
  finally:C.supported_requests=original
  np.testing.assert_array_equal(grid.labels,before);record=dict(candidate=candidate,elapsed_wall_s=time.perf_counter()-begin,meta=meta,has_plan=plan is not None)
  if plan is not None:
   record['waypoints']=[vars(w) for w in plan.waypoints]
   if candidate:assert plan.direction_changes==0 and all(w.drive_direction=='forward' for w in plan.waypoints)
  arms.append(record)
 assert arms[0]['meta']['local_target']==arms[1]['meta']['local_target'];results.append(dict(frame=i,time_s=row['time_s'],arms=arms,current_actual_state_only=True,not_alternate_future_label=True));print(i,[(a['candidate'],a['meta']['status'],a['meta'].get('direction_changes')) for a in arms],flush=True)
first=results[0]['arms'][1];negative=results[-1]['arms'];entry=first['has_plan'] and first['meta']['direction_changes']==0 and all(not a['has_plan'] for a in negative)
a=dict(decision='PASS_NECESSARY_SAME_DIRECTION_GEOMETRY_ONLY' if entry else 'REJECT_UNIQUE_FORWARD_ONLY_GEOMETRY_ENTRY',entry_pass=entry,queries=6,protocol_sha256=hashlib.sha256((H/'E1_same_direction_geometry_probe_protocol_v1.json').read_bytes()).hexdigest(),not_physical_or_dynamic_admission=True,not_new_online=True,no_parameter_sweep=True,no_models_or_actions_changed=True,no_actual_simulation=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'queries.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(a,indent=2))
