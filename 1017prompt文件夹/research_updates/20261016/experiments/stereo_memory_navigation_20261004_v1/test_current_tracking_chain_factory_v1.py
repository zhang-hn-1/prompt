import numpy as np
def test_genuine_full_factory_readonly_chain_single_act(monkeypatch):
 import json
 from pathlib import Path
 from scipy.spatial.transform import Rotation
 from benchmark.policies import PolicyObservation,OraclePathPolicy
 from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
 from scripts.run_pointgoal_benchmark import load_action_protocol
 from depth_local_curve import P,install_local_reference
 from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
 from current_tracking_chain_telemetry_v1 import policy_class
 h=Path(__file__).resolve().parent;d=h/'reverse_main_navigation_e1_b0_run_v1';f=60
 row=json.loads((d/'policy_records.json').read_text())[f];state=json.loads((d/'states.json').read_text())[f];plan=json.loads((d/'local_plans/frame_060.json').read_text());hist=np.load(d/'history/frame_060.npz');t=row['time_s'];xy=np.asarray(state['position'][:2]);rot=Rotation.from_quat(np.array(state['quaternion'])[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(rot[1,0],rot[0,0]));prop=hist['proprio_history'].copy();prop[-1,:2]=state['velocity'][:2]
 obs=PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),prop,hist['action_history'],xy,yaw,quaternion_world_wxyz_history=hist['quaternion_world_wxyz_history']);g=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])@(np.array([3.300000005960465,5.960465010446114e-09])-xy);protocol=load_action_protocol(P/'configs/data_contract.yaml');original=OraclePathPolicy.act;calls=[]
 def counted(self,*a,**k):calls.append(self);return original(self,*a,**k)
 monkeypatch.setattr(OraclePathPolicy,'act',counted);outputs=[];policies=[]
 for enabled in [False,True]:
  p=policy_class(enabled)(protocol,memory_mode='current');reference=KinodynamicPlanResult('success',tuple(KinodynamicWaypoint(**v) for v in plan['waypoints']),plan['expanded_nodes'],plan['path_length_m'],plan['direction_changes']);install_local_reference(p.tracker,reference);p.plan=reference;p.plan_meta=plan['plan_meta'];p.reference_decision=row['reference_decision'];p.reference_installed_time=t;p.last_plan=t;p.last_frame=f-1;p.time=t-.1;p.grid=RepeatedExposureMemoryGrid(xy-20,10,static_scene_assumption=True)
  p.tracker._active_drive_direction='forward';p.tracker._pending_drive_direction=None;p.tracker._braking_state='tracking';p.tracker._path_index=row['tracker_state_after_act']['path_index']
  p.set_source_packet(dict(frame=f,time_s=t,records=[]));p.set_current_packet(dict(frame=f,time_s=t,points_world=np.empty((0,3)),root_xyz=np.asarray(state['position'])));outputs.append(p.act(obs,g));policies.append(p)
 assert len(calls)==2 and all(len(p.records)==1 for p in policies)
 np.testing.assert_array_equal(outputs[0],outputs[1]);assert np.all(outputs[0][::2]>.5)
 assert 'current_tracking_chain' not in policies[0].records[-1]
 chain=policies[1].records[-1]['current_tracking_chain'];assert chain['fresh_tracker_record'] and chain['valid'] and chain['source']=='tracking'
 assert 'calibrated_bank_mapping' in chain and 'yaw_rate_feedback_auxiliary_rad' in chain
 assert policies[0].tracker._tracking_records==policies[1].tracker._tracking_records
 np.testing.assert_array_equal(outputs[0][1::2],outputs[1][1::2]);assert np.any(outputs[0][1::2]!=0)
 assert [c.__name__ for c in type(policies[0]).__mro__]==[c.__name__ for c in type(policies[1]).__mro__]
 assert policies[0].tracker._path_index==policies[1].tracker._path_index and policies[0].tracker._pending_drive_direction==policies[1].tracker._pending_drive_direction
 np.testing.assert_array_equal(policies[0].tracker._reference_xy,policies[1].tracker._reference_xy)
