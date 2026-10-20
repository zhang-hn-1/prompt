"""Nine factual installation queries: current/past-only alternate geometric headings.
Not an actor and not alternate future physical labels.
"""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
from current_motion_heading_feasibility_v1 import derive_motion_heading
from partial_point_risk_grid import PartialPointRiskGrid
from execution_domain_four_point_consumer_v2 import local_curve_for_direction_tube
from prior_rgb_navigation import PriorRGBPolicy
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent;O=H/'E1_current_motion_heading_feasibility_v1';O.mkdir(exist_ok=False);protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));drive=PriorRGBPolicy(protocol,'current').tracker.drive_model;results=[]
for name,folder in [('B0','original_E1_direct_memory_navigation_run_v1'),('C_static','original_E1_static_sparse_memory_navigation_run_v1')]:
 D=H/folder;seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
 for f,s in seal['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
 z=np.load(D/'physics.npz');states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text())
 for i,r in enumerate(rows):
  if not r['local_plan_created']:continue
  indices=np.arange(max(0,i-4),i+1)*5;t=z['simulation_time_s'][indices];v=z['linear_velocity_world_m_s'][indices,:2];m=z['joint_velocity_rad_s'][indices,0];motion=derive_motion_heading(float(t[-1]),t,v,m);R=Rotation.from_quat(np.asarray(states[i]['quaternion'])[[1,2,3,0]]).as_matrix();body=float(np.arctan2(R[1,0],R[0,0]));saved=np.load(D/'maps'/f'frame_{i:03d}.npz');g=PartialPointRiskGrid(saved['origin'],10);g.labels=saved['labels'];xy=np.asarray(states[i]['position'])[:2];initial=r['plan']['initial_direction'];pair=[]
  for label,yaw in [('body_yaw',body)]+([] if motion is None else [('qualified_measured_motion_heading',motion)]):
   plan,meta=local_curve_for_direction_tube(g,xy,yaw,np.array([3.300000005960465,5.960465010446114e-9]),drive,initial_direction=initial)
   if label=='body_yaw':assert plan is not None;np.testing.assert_array_equal([[w.x_m,w.y_m] for w in plan.waypoints],r['installed_reference_xy_after_act'])
   reverse_length=0.
   if plan is not None:
    for a,b in zip(plan.waypoints[:-1],plan.waypoints[1:]):
     if b.drive_direction=='reverse':reverse_length+=float(np.hypot(b.x_m-a.x_m,b.y_m-a.y_m))
   pair.append(dict(name=label,yaw_rad=yaw,status=meta['status'],local_target=meta['local_target'],path_length_m=None if plan is None else plan.path_length_m,direction_changes=None if plan is None else plan.direction_changes,nominal_reverse_length_m=reverse_length,not_alternate_actual_trajectory=True))
  results.append(dict(arm=name,frame=i,time_s=float(t[-1]),current_actual_main_rad_s=float(m[-1]),current_XY_speed_m_s=float(np.linalg.norm(v[-1])),heading_qualified=motion is not None,motion_minus_body_deg=None if motion is None else float(np.degrees((motion-body+np.pi)%(2*np.pi)-np.pi)),queries=pair))
a=dict(decision='CURRENT_MOTION_HEADING_FEASIBILITY_ONLY_NO_CONTROL_ADMISSION',events=len(results),qualified_events=sum(r['heading_qualified'] for r in results),body_yaw_installed_geometry_all_exact=True,old_whole_sequences_DEVELOPMENT_not_fresh=True,known_GT_velocity_upperbound=True,nominal_heading_from_course_is_not_future_heading_dynamics=True,no_actor_changes_or_new_physics=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'installation_queries.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(a,indent=2));print(json.dumps(results,indent=2))
