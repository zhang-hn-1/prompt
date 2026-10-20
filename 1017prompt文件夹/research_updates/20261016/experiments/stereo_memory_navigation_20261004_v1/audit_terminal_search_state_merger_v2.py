from pathlib import Path
import json,hashlib,datetime,dataclasses,numpy as np
from persistent_core_grid import PersistentCoreGrid
from execution_domain_four_point_consumer_v2 import local_curve_for_direction_tube
from execution_domain_four_point_source_v2 import supported_curvature,supported_requests
from benchmark.policies import _default_drive_model
import execution_domain_four_point_consumer_v2 as consumer
from negative_memory_validation import true_cylinder_clearance
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'terminal_node_search_merger_v2';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
records=json.loads((D/'policy_records.json').read_text());events=[r for r in records if r.get('local',{}).get('local_plan_created') and r.get('local',{}).get('plan',{}).get('local_target') is not None and np.linalg.norm(np.asarray(r['local']['plan']['local_target'])-[2.2,.15])<1e-5];i=events[0]['frame'];assert i==480
files=[D/f'maps/frame_{i:03d}.npz',D/f'estimated_poses/frame_{i:03d}.npz',D/'estimated_poses/frame_000.npz',D/'frozen_memory_seed.npz',D/'policy_records.json',H/'execution_domain_four_point_consumer_v2.py',H/'execution_domain_four_point_source_v2.py',H/'supported_four_point_planning.py',H/'terminal_node_admission.py',H/'terminal_node_search_trace.py',H.parent.parent/'docs/superpowers/plans/2026-10-08-terminal-search-state-merger-causal-audit.md']
(O/'inputseal.json').write_text(json.dumps(dict(utc=now(),files={str(f):sha(f) for f in files},code_sha256=sha(__file__),event_frame=i,past_only_geometry_event=True,historical_seen=True,goal_tolerance_m=.2,necessary_terminal_curvature_interval=[-.827225,.796603],no_GT_planning_input=True),indent=2)+'\n')
m=np.load(files[0]);pose=np.load(files[1]);init=np.load(files[2]);seed=dict(np.load(files[3]));g=PersistentCoreGrid(m['origin'],60);g.set_source(dict(time_s=float(init['time_s'])));g.update_points(np.empty((0,3)),init['root_position_world_m'],float(init['time_s']));g.seed(seed,float(seed['time_s']));g.set_source(dict(time_s=float(pose['time_s'])));g.update_points(np.empty((0,3)),pose['root_position_world_m'],float(pose['time_s']));np.testing.assert_equal(g.labels,m['labels']);yaw=float(np.arctan2(pose['root_rotation_world'][1,0],pose['root_rotation_world'][0,0]));xy=pose['root_position_world_m'][:2];goal=np.array([2.2,.15]);baseline,meta=local_curve_for_direction_tube(g,xy,yaw,goal,_default_drive_model(),initial_direction='forward');assert baseline is not None
baseXY=np.array([[w.x_m,w.y_m] for w in baseline.waypoints]);np.testing.assert_array_equal(baseXY,events[0]['local']['installed_reference_xy_after_act'])
from terminal_node_search_trace import plan_kinodynamic_path as checked_plan,goal_approach_allowed,GENERATED_GOAL_NODES
Original=consumer.plan_kinodynamic_path
calls=[]
def checked(request,model,**kwargs):
 assert np.linalg.norm(np.array(request.goal_xy)-goal)<1e-5, 'Only final target allowed'
 def acceptance(x,y,yaw,direction):
  result=goal_approach_allowed(x,y,yaw,direction,goal)
  calls.append(dict(x=x,y=y,yaw=yaw,direction=direction,accepted=result))
  return result
 return checked_plan(request,model,goal_acceptance=acceptance,**kwargs)
consumer.plan_kinodynamic_path=checked
try:candidate,cmeta=local_curve_for_direction_tube(g,xy,yaw,goal,_default_drive_model(),initial_direction='forward')
finally:consumer.plan_kinodynamic_path=Original
(O/'goal_admission_calls_pretruth.json').write_text(json.dumps(calls,indent=2)+'\n')
(O/'generated_goal_nodes_pretruth.json').write_text(json.dumps(GENERATED_GOAL_NODES,indent=2)+'\n')
assert calls==json.loads((H/'terminal_node_admission_preflight_v1/goal_admission_calls_pretruth.json').read_text())
assert candidate is None and cmeta['expanded_nodes']==3000

C=None if candidate is None else np.array([[w.x_m,w.y_m,w.yaw_rad] for w in candidate.waypoints]);commands=None if candidate is None else np.array([[w.main_command_rad_s,w.auxiliary_angle_rad] for w in candidate.waypoints]);supported=candidate is not None and all(w.main_command_rad_s==0 or supported_curvature(w.main_command_rad_s,w.auxiliary_angle_rad) is not None for w in candidate.waypoints)
np.savez_compressed(O/'plans_pretruth.npz',baseline_xy=baseXY,candidate_pose=np.empty((0,3)) if C is None else C,candidate_commands=np.empty((0,2)) if commands is None else commands)
(O/'outputseal_pretruth.json').write_text(json.dumps(dict(utc=now(),output_sha256=sha(O/'plans_pretruth.npz'),baseline_exact=True,candidate_metadata=cmeta,goal_calls_sha256=sha(O/'goal_admission_calls_pretruth.json'),generated_sha256=sha(O/'generated_goal_nodes_pretruth.json')),indent=2)+'\n')
clear=None if C is None else true_cylinder_clearance(C[:,:2]);end=None if C is None else float(np.linalg.norm(C[-1,:2]-goal));gates=dict(success=candidate is not None,endpoint=end is not None and end<=.2+1e-8,endpoint_approach=candidate is not None and goal_approach_allowed(*C[-1],candidate.waypoints[-1].drive_direction,goal),nominal_clearance=clear is not None and clear>=.8,supported=bool(supported),no_extra_switch=candidate is not None and candidate.direction_changes<=baseline.direction_changes,reference_changed=C is not None and not np.array_equal(C[:,:2],baseXY))
r=dict(utc=now(),decision='PASS_HISTORICAL_TERMINAL_NODE_GEOMETRY_ONLY' if all(gates.values()) else 'REJECT_TERMINAL_NODE_ADMISSION_BEFORE_GPU',event_frame=i,gates=gates,baseline_metadata=meta,candidate_metadata=cmeta,baseline_end_distance_m=float(np.linalg.norm(baseXY[-1]-goal)),candidate_end_distance_m=end,true_nominal_clearance_m=clear,no_new_actual_navigation=True,not_fresh_heldout=True,not_stopping_prediction=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
