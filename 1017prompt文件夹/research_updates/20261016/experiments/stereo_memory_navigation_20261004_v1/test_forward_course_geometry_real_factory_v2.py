"""Genuine factory, synthetic empty-risk/current-state fixture; no physics replay."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
H=Path(__file__).resolve().parent

def make(enabled):
 from scripts.run_pointgoal_benchmark import load_action_protocol
 from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
 from depth_local_curve import P,install_local_reference
 from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
 from forward_course_geometry_adapter_v2 import policy_class
 p=policy_class(enabled)(load_action_protocol(P/'configs/data_contract.yaml'),memory_mode='current')
 ref=KinodynamicPlanResult('success',tuple(KinodynamicWaypoint(x,0.,0.,'forward',.5,0.) for x in [0.,.5,1.,1.5,2.]),0,2.,0)
 install_local_reference(p.tracker,ref);p.plan=ref;p.plan_meta={'unknown_fraction':0.};p.reference_installed_time=7.;p.grid=RepeatedExposureMemoryGrid(np.array([-20.,-20.]),10,static_scene_assumption=True)
 p.last_frame=69;p.time=6.9;p.last_plan=7.
 return p

def observation(v=(.2,.2),main=.5):
 from benchmark.policies import PolicyObservation
 data=np.load(H/'reverse_main_navigation_e1_b0_run_v1/history/frame_060.npz')
 prop=data['proprio_history'].copy();prop[-1,:2]=v;prop[-1,12]=main
 return PolicyObservation(np.zeros((480,640,3),np.uint8),np.zeros((480,640,3),np.uint8),prop,data['action_history'],np.zeros(2),0.,quaternion_world_wxyz_history=np.tile([1.,0.,0.,0.],(len(prop),1)))

def current(p,f,t):
 p.last_plan=t # freeze existing reference in CPU fixture, no solver
 p.set_source_packet(dict(frame=f,time_s=t,records=[]))
 p.set_current_packet(dict(frame=f,time_s=t,points_world=np.empty((0,3)),root_xyz=np.zeros(3)))

def test_genuine_factory_curvature_reaches_request_once(monkeypatch):
 from benchmark.policies import OraclePathPolicy
 old=OraclePathPolicy.act;calls=[]
 def counted(self,*a,**kw):calls.append(self);return old(self,*a,**kw)
 monkeypatch.setattr(OraclePathPolicy,'act',counted)
 out=[]
 for enabled in (False,True):
  p=make(enabled);o=observation();ref=tuple(p.tracker._reference);counts={'target':0,'compose':0}
  target=p.tracker._lookahead_target;compose=p.tracker._compose_tracking_curvature
  def target_count(*a,**kw):counts['target']+=1;return target(*a,**kw)
  def compose_count(*a,**kw):counts['compose']+=1;return compose(*a,**kw)
  p.tracker._lookahead_target=target_count;p.tracker._compose_tracking_curvature=compose_count
  for f in range(70,75):
   current(p,f,f*.1);action=p.act(o,np.array([3.,0.]))
  assert counts=={'target':5,'compose':5}
  assert p.tracker._lookahead_target is target_count and p.tracker._compose_tracking_curvature is compose_count
  assert tuple(p.tracker._reference)==ref and o.yaw_world_rad==0.
  assert p.tracker._active_drive_direction=='forward' and p.tracker._pending_drive_direction is None
  assert p.records[-1]['forward_course_geometry']['used']==enabled
  out.append(action)
 assert len(calls)==10
 assert not np.array_equal(out[0],out[1])

@pytest.mark.parametrize('mode',['low_speed','negative_main','short_history','goal','no_plan','tracking_failed'])
def test_genuine_ineligible_or_early_return_parity(mode):
 outputs=[]
 for enabled in (False,True):
  p=make(enabled);o=observation(v=(0.,0.) if mode=='low_speed' else (.2,.2),main=-.5 if mode=='negative_main' else .5)
  if mode not in ('short_history',):p._course_history=[(6.6+i*.1,np.array([.2,.2]),.5) for i in range(4)]
  if mode=='no_plan':p.plan=None
  if mode=='tracking_failed':p.tracker._failure_subreason='fixture_tracking_failure'
  goal=np.zeros(2) if mode=='goal' else np.array([3.,0.])
  current(p,70,7.);action=p.act(o,goal);outputs.append(action)
  assert not p.records[-1]['forward_course_geometry']['used']
 np.testing.assert_array_equal(*outputs)

def test_real_packet_invalid_clock_guard_before_poison_payload():
 p=make(True)
 class Poison:
  @property
  def proprio_history(self):raise AssertionError('invalid packet consumed observation')
 for frame,t in [(70,float('nan')),(70,float('inf')),(70,6.8),(71,7.)]:
  with pytest.raises(ValueError):p.set_current_packet(dict(frame=frame,time_s=t,points_world=np.empty((0,3)),root_xyz=np.zeros(3)))
 assert not p._course_history and p._packet is None

def test_original_composition_exception_restores_hooks():
 p=make(True);o=observation();target=p.tracker._lookahead_target
 def bad(*a,**kw):raise RuntimeError('original composition failed')
 p.tracker._compose_tracking_curvature=bad;current(p,70,7.)
 with pytest.raises(RuntimeError,match='original composition failed'):p.act(o,np.array([3.,0.]))
 assert p.tracker._lookahead_target==target and p.tracker._compose_tracking_curvature is bad
 p.reset(None);assert p._course_history==[]

def test_valid_future_or_broken_history_clock_rejects_before_payload():
 class Poison:
  @property
  def proprio_history(self):raise AssertionError('clock must reject before reading payload')
 for future in (True,False):
  p=make(True)
  if not future:p._course_history=[(6.5+i*.1,np.array([.2,.2]),.5) for i in range(4)]
  current(p,70,100. if future else 7.)
  with pytest.raises(ValueError):p.act(Poison(),np.array([3.,0.]))

@pytest.mark.parametrize('mode',['reverse','pending_reverse','planned_zero'])
def test_genuine_direction_or_planned_zero_parity(mode):
 from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
 from depth_local_curve import install_local_reference
 outputs=[];states=[]
 for enabled in (False,True):
  p=make(enabled);p._course_history=[(6.6+i*.1,np.array([.2,.2]),.5) for i in range(4)]
  if mode=='planned_zero':
   ref=KinodynamicPlanResult('success',tuple(KinodynamicWaypoint(x,0.,0.,'forward',0.,0.) for x in [0.,.5,1.,1.5,2.]),0,2.,0);install_local_reference(p.tracker,ref);p.plan=ref
  else:
   p.tracker._active_drive_direction='reverse'
   if mode=='pending_reverse':p.tracker._pending_drive_direction='reverse';p.tracker._braking_state='direction_change_braking'
  current(p,70,7.);outputs.append(p.act(observation(),np.array([3.,0.])))
  states.append((p.tracker._active_drive_direction,p.tracker._pending_drive_direction,p.tracker._stopped_cycles,p.tracker._braking_state))
  assert not p.records[-1]['forward_course_geometry']['used']
 np.testing.assert_array_equal(*outputs);assert states[0]==states[1]
