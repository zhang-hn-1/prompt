import numpy as np
import pytest
@pytest.mark.parametrize("no_prefix,already_used",[(False,False),(True,False),(False,True)])
def test_real_factory_before_install_and_single_act(monkeypatch,no_prefix,already_used):
 import json
 from pathlib import Path
 from scipy.spatial.transform import Rotation
 from benchmark.policies import PolicyObservation,OraclePathPolicy
 from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
 from scripts.run_pointgoal_benchmark import load_action_protocol
 from depth_local_curve import P,install_local_reference
 from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
 from bounded_prefix_live_adapter_v1 import policy_class
 import bounded_prefix_live_adapter_v1 as adapter
 import continuous_cell_tube_consumer_v1 as owner
 import minimum_prefix_transition_consumer_v1 as planner_owner
 h=Path(__file__).resolve().parent;d=h/'reverse_main_navigation_e1_b0_run_v1';f=70
 row=json.loads((d/'policy_records.json').read_text())[f];state=json.loads((d/'states.json').read_text())[f];plan=json.loads((d/'local_plans/frame_060.json').read_text());hist=np.load(d/'history/frame_060.npz');t=row['time_s'];xy=np.asarray(state['position'][:2]);rot=Rotation.from_quat(np.array(state['quaternion'])[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(rot[1,0],rot[0,0]));prop=hist['proprio_history'].copy();prop[-1,:2]=state['velocity'][:2]
 obs=PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),prop,hist['action_history'],xy,yaw,quaternion_world_wxyz_history=hist['quaternion_world_wxyz_history']);g=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])@(np.array([3.300000005960465,5.960465010446114e-09])-xy);protocol=load_action_protocol(P/'configs/data_contract.yaml');original=OraclePathPolicy.act;calls=[]
 def counted(self,*a,**k):calls.append(self);return original(self,*a,**k)
 monkeypatch.setattr(OraclePathPolicy,'act',counted);outputs=[];policies=[];installed=[];original_install=owner.install_local_reference
 def install(tracker,reference):installed.append(reference);return original_install(tracker,reference)
 monkeypatch.setattr(owner,'install_local_reference',install)
 full=KinodynamicPlanResult('success',tuple(KinodynamicWaypoint(float(xy[0]-j*.3*np.cos(yaw)),float(xy[1]-j*.3*np.sin(yaw)),yaw,'reverse',-.5,0.) for j in range(3)),0,.6,0)
 meta=dict(status='success',route_xy=[xy.tolist(),[3.3,0]],local_target=[float(xy[0]-.6),float(xy[1])],local_path_length_m=.6,direction_changes=0,certified_safe=False,synthetic_planner_fixture=True)
 def local(*args,**kwargs):return full,meta
 monkeypatch.setattr(planner_owner,'local_curve_first_direction',local)
 if no_prefix:monkeypatch.setattr(adapter,'choose_prefix',lambda *args:(None,[]))
 for enabled in [False,True]:
  p=policy_class(enabled,goal_axis_world=[1,0])(protocol,memory_mode='current');reference=KinodynamicPlanResult('success',tuple(KinodynamicWaypoint(**v) for v in plan['waypoints']),plan['expanded_nodes'],plan['path_length_m'],plan['direction_changes']);install_local_reference(p.tracker,reference);p.plan=None;p.plan_meta=plan['plan_meta'];p.reference_decision=row['reference_decision'];p.reference_installed_time=t;p.last_plan=t-.5;p.last_frame=f-1;p.time=t-.1;p.grid=RepeatedExposureMemoryGrid(xy-20,10,static_scene_assumption=True)
  p.bounded_prefix_used=already_used
  p.tracker._active_drive_direction='forward';p.tracker._pending_drive_direction=None;p.tracker._braking_state='tracking';p.tracker._path_index=row['tracker_state_after_act']['path_index']
  p.set_source_packet(dict(frame=f,time_s=t,records=[]));p.set_current_packet(dict(frame=f,time_s=t,points_world=np.empty((0,3)),root_xyz=np.asarray(state['position'])));outputs.append(p.act(obs,g));policies.append(p)
 assert len(calls)==2 and len(installed)==2 and all(len(p.records)==1 for p in policies)
 assert installed[0] is full and policies[0].plan is full
 assert not policies[0].records[-1]['bounded_prefix_decision']['intervention']
 if no_prefix or already_used:
  assert installed[1] is full and policies[1].plan is full and policies[1].plan_meta is meta
  np.testing.assert_array_equal(outputs[0],outputs[1]);assert policies[1].bounded_prefix_used==already_used
 else:
  assert installed[1] is policies[1].plan and installed[1].waypoints[0].drive_direction=='forward'
  assert policies[1].bounded_prefix_used and policies[1].records[-1]['bounded_prefix_decision']['intervention']
  assert np.all(outputs[1][::2]>0) and np.all(outputs[0][::2]==0)
  assert policies[0].tracker._pending_drive_direction=='reverse' and policies[1].tracker._pending_drive_direction is None
  assert policies[1].plan_meta['partial_reference'] and not policies[1].plan_meta['old_local_target_reached']
  assert policies[1].records[-1]['bounded_prefix_decision']['parent_full_plan']==[vars(w) for w in full.waypoints]
 assert [c.__name__ for c in type(policies[0]).__mro__]==[c.__name__ for c in type(policies[1]).__mro__]

def test_static_axis_rejects_invalid_domain_and_copies_input():
 from bounded_prefix_live_adapter_v1 import policy_class
 for value in ([0,0],[float('nan'),1],[1,2,3]):
  with pytest.raises(ValueError):policy_class(True,goal_axis_world=value)
 axis=np.array([1.,0.]);cls=policy_class(True,goal_axis_world=axis);axis[:]=0
 assert cls.__name__=='BoundedForwardPrefix'
