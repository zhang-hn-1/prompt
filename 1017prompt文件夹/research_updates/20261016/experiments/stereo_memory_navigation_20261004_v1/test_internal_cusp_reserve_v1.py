import numpy as np
from types import SimpleNamespace as NS
from internal_cusp_reserve_v1 import cusp_context,with_internal_cusp_reserve

def fixture():
 tr=NS(_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking',_reference_xy=np.array([[0.,0.],[.3,0.],[.6,0.],[-.3,0.]]),_reference=tuple(NS(drive_direction=d) for d in ['forward','forward','reverse','reverse']),_path_index=1)
 p=NS(time=10.,reference_decision={'measured_time_s':10.},tracker=tr)
 return p,NS(position_world_xy=np.array([.4,.1]))

def test_internal_cusp_uses_outgoing_segment_not_local_final_endpoint():
 p,o=fixture();r=cusp_context(p,o);assert r['eligible'] and abs(r['remaining_arc_m']-.2)<1e-12 and r['cusp_index']==2
 p.tracker._pending_drive_direction='reverse';assert not cusp_context(p,o)['eligible']

def test_no_cusp_and_future_clock_do_not_read_reference():
 p,o=fixture();p.tracker._reference=tuple(NS(drive_direction='forward') for _ in range(4));assert not cusp_context(p,o)['eligible']
 class Poison:
  @property
  def tracker(self):raise AssertionError('future reference read')
 p=Poison();p.time=10.;p.reference_decision={'measured_time_s':11.};assert not cusp_context(p,o)['eligible']

def test_single_act_auxiliary_state_and_b0_unchanged():
 p,o=fixture()
 class Base:
  def act(self,o,g):
   self.calls+=1;self.records.append(dict(measured_speed_control_applied=True));return np.tile([.93,.25],5).astype(np.float32)
 for enabled in [False,True]:
  q=with_internal_cusp_reserve(Base,enabled=enabled)();q.calls=0;q.records=[];q.tracker=p.tracker;q.time=p.time;q.reference_decision=p.reference_decision
  a=q.act(o,[4,0]);assert q.calls==1;assert np.all(a[::2]==np.float32(.5 if enabled else .93));assert np.all(a[1::2]==np.float32(.25));assert q.tracker._path_index==1 and q.tracker._pending_drive_direction is None

def test_real_current_installed_reference_qualifies_without_waypoint_jump():
 import json
 from pathlib import Path
 h=Path(__file__).resolve().parent;d=h/'forward_intent_navigation_e1_candidate_run_v1'
 row=json.loads((d/'policy_records.json').read_text())[120];s=json.loads((d/'states.json').read_text())[120];plan=json.loads((d/'local_plans/frame_120.json').read_text());state=row['tracker_state_after_act'];xy=np.array(row['installed_reference_xy_after_act'])
 tr=NS(_reference_xy=xy,_reference=tuple(NS(**w) for w in plan['waypoints']),_path_index=state['path_index'],_active_drive_direction=state['active_direction'],_pending_drive_direction=state['pending_direction'],_braking_state=state['braking_state']);policy=NS(time=row['time_s'],reference_decision=row['reference_decision'],tracker=tr);r=cusp_context(policy,NS(position_world_xy=np.asarray(s['position'][:2])))
 assert r['eligible'] and .59<r['remaining_arc_m']<.61 and tr._path_index==0

def test_real_two_arm_class_chain_common_parent_and_only_one_new_wrapper():
 from internal_cusp_policy_factory_v1 import policy_class
 a,b=policy_class('b0'),policy_class('candidate')
 assert [c.__name__ for c in a.__mro__]==[c.__name__ for c in b.__mro__]
 assert sum(c.__name__=='InternalCuspReserve' for c in a.__mro__)==1
 assert sum(c.__name__=='CurrentForwardIntent' for c in a.__mro__)==1

def test_invalid_reference_preserves_parent_action_even_in_b0():
 for bad in ['index','duplicate','nan']:
  p,o=fixture()
  if bad=='index':p.tracker._path_index=99
  if bad=='duplicate':p.tracker._reference_xy[1]=p.tracker._reference_xy[0]
  if bad=='nan':p.tracker._reference_xy[1,0]=np.nan
  assert not cusp_context(p,o)['eligible']

def test_complete_runtime_factory_single_real_tracker_act(monkeypatch):
 import json
 from pathlib import Path
 from scipy.spatial.transform import Rotation
 from benchmark.policies import PolicyObservation,OraclePathPolicy
 from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
 from internal_cusp_policy_factory_v1 import policy_class
 from scripts.run_pointgoal_benchmark import load_action_protocol
 from depth_local_curve import install_local_reference,P
 from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
 h=Path(__file__).resolve().parent;d=h/'forward_intent_navigation_e1_candidate_run_v1'
 row=json.loads((d/'policy_records.json').read_text())[120];state=json.loads((d/'states.json').read_text())[120];plan=json.loads((d/'local_plans/frame_120.json').read_text());hist=np.load(d/'history/frame_120.npz');maps=np.load(d/'maps/frame_120.npz');t=row['time_s'];xy=np.array(state['position'][:2]);R=Rotation.from_quat(np.array(state['quaternion'])[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));proprio=hist['proprio_history'].copy();proprio[-1,:2]=state['velocity'][:2]
 obs=PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),proprio,hist['action_history'],xy,yaw,quaternion_world_wxyz_history=hist['quaternion_world_wxyz_history']);protocol=load_action_protocol(P/'configs/data_contract.yaml');actual=OraclePathPolicy.act;calls=[]
 def counted(self,*args,**kw):calls.append(self);return actual(self,*args,**kw)
 monkeypatch.setattr(OraclePathPolicy,'act',counted)
 actions=[];policies=[]
 for arm in ['b0','candidate']:
  p=policy_class(arm)(protocol,memory_mode='current');reference=KinodynamicPlanResult('success',tuple(KinodynamicWaypoint(**w) for w in plan['waypoints']),plan['expanded_nodes'],plan['path_length_m'],plan['direction_changes']);install_local_reference(p.tracker,reference);p.plan=reference;p.plan_meta=plan['plan_meta'];p.reference_decision=row['reference_decision'];p.reference_installed_time=t;p.last_plan=t;p.time=t-.1;p.last_frame=119;p.grid=RepeatedExposureMemoryGrid(maps['origin'],10,static_scene_assumption=True);p.set_source_packet(dict(frame=120,time_s=t,records=[]));p.set_current_packet(dict(frame=120,time_s=t,points_world=np.empty((0,3)),root_xyz=np.array(state['position'])));actions.append(p.act(obs,np.array([4.,0.])));policies.append(p)
 assert len(calls)==2 and len(policies[0].records)==len(policies[1].records)==1
 assert actions[0][0]>.5 and actions[1][0]==.5
 np.testing.assert_array_equal(actions[0][1::2],actions[1][1::2]);np.testing.assert_array_equal(policies[0].tracker._reference_xy,policies[1].tracker._reference_xy)
 assert policies[0].tracker._path_index==policies[1].tracker._path_index and policies[0].tracker._pending_drive_direction==policies[1].tracker._pending_drive_direction
