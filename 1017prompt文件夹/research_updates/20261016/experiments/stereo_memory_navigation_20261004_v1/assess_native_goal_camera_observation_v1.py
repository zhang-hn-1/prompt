from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;D=H/'native_goal_camera_observation_run_v1'
assert (H/'native_goal_camera_observation_runtime_verification_v1.json').exists()
records=json.loads((D/'observer_records.json').read_text());z=np.load(D/'physics.npz');E=np.load(H/'partial_surface_frozen_extrinsics_v1.npz');R=Rotation.from_quat(z['root_quaternion_world_wxyz'][:,[1,2,3,0]]).as_matrix();axis=R@E['left_optical_to_base'][:3,2];root=z['root_position_world_m'];goal=np.array([2.2,.15]);bearing=np.arctan2(goal[1]-root[:,1],goal[0]-root[:,0]);heading=np.arctan2(axis[:,1],axis[:,0]);error=np.arctan2(np.sin(heading-bearing),np.cos(heading-bearing));displacement=np.linalg.norm(root[:,:2]-np.array([-2.2,.15]),axis=1)
turn=[i for i,r in enumerate(records) if r['phase']=='turn'];first_hold=next((i for i,r in enumerate(records) if r['phase']=='hold'),None)
crossed=bool(first_hold is not None and np.any(error[:first_hold*5+1]<=0))
vel=z['linear_velocity_world_m_s'][::5,:2] if 'linear_velocity_world_m_s' in z.files else z['velocity_world_xy'][::5]
low=0;counts=[]
for v in vel:
 low=low+1 if np.linalg.norm(v)<.05 else 0;counts.append(low)
contacts=json.loads((D/'physics_contacts.json').read_text());source=json.loads((H/'native_goal_camera_observation_source_assessment_v1/assessment.json').read_text())
checks=dict(exercised=bool(turn),within_start_prior=bool(displacement.max()<=.7),no_contact=not any(r['contact'] for r in contacts),five_low_at_end=counts[-1]>=5,actual_camera_crossed_goal_by_turn_end=crossed,source=all(source['checks'].values()))
baseline=H/'partial_surface_source_validation_run_v1';b=np.load(baseline/'physics.npz');fields=['simulation_time_s','root_position_world_m','root_quaternion_world_wxyz','joint_position_rad','joint_velocity_rad_s','applied_action','smoothed_target'];prefix={k:bool(np.array_equal(z[k][:26],b[k][:26])) for k in fields};np.savez_compressed(H/'native_goal_camera_observation_actual_trace_v1.npz',time_s=z['simulation_time_s'],camera_goal_error_rad=error,root_displacement_m=displacement,actual_root_position_world_m=root)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_BOUNDED_OBSERVATION_AND_PARTIAL_SOURCE_ONLY' if all(checks.values()) else 'REJECT_BOUNDED_NATIVE_CAMERA_GOAL_OBSERVATION',checks=checks,turn_blocks=len(turn),first_turn_frame=turn[0] if turn else None,first_hold_frame=first_hold,actual_root_max_displacement_m=float(displacement.max()),actual_camera_error_at_hold_deg=float(np.degrees(error[first_hold*5])) if first_hold is not None else None,final_consecutive_low_samples=counts[-1],source_decision=source['decision'],physical_prefix_fields_through_0p5=prefix,strict_physical_prefix=all(prefix.values()),not_navigation=True,not_original35=True,not_fresh_heldout=True,no_gain_or_threshold_scan=True,whole_research_complete=False)
(H/'native_goal_camera_observation_assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
