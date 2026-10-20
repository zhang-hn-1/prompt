"""Hand-derived geometry plus inherited real tracker; no simulator or mocks."""
import numpy as np,pytest
from velocity_pursuit_consumer import proxy_target,with_velocity_pursuit
from outgoing_lookahead import make_endpoint_oracle
from oracle_rgb_shadow import reference_from_arrays
from depth_local_curve import install_local_reference,P
from benchmark.policies import PolicyObservation
from training.action_protocol import load_action_protocol

def tracker(candidate=False,directions=None):
 p=load_action_protocol(P/'configs/data_contract.yaml');original=make_endpoint_oracle(p);t=with_velocity_pursuit(type(original))(p,lookahead_m=.5) if candidate else original
 signs=[1]*5 if directions is None else directions
 pose=[[0,0,0],[.5,0,0],[1,0,0],[1.5,0,0],[2,0,0]]
 install_local_reference(t,reference_from_arrays(pose,signs,[[.5*s,0] for s in signs[:-1]]+[[0,0]],0.,1));return t

def obs(xy=(0,0),yaw=0.,v=(.1,0)):
 rgb=np.zeros((480,640,3),np.uint8);proprio=np.zeros((2,17),np.float32);proprio[-1,:2]=v
 return PolicyObservation(rgb,rgb,proprio,np.zeros((2,10),np.float32),np.array(xy,float),yaw)

@pytest.mark.parametrize('direction,velocity,target',[('forward',[.1,0],[.5,.1339745962155614]),('reverse',[-.1,0],[-.5,.1339745962155614])])
def test_ideal_forward_reverse_circle_keeps_curvature(direction,velocity,target):
 # Break: apply reverse sign twice or use raw course rather than body convention.
 got,heading,active=proxy_target(target,[0,0],0,velocity,direction)
 np.testing.assert_array_equal(got,target);assert heading==pytest.approx(0,abs=1e-12)
 assert 2*got[1]/np.dot(got,got)==pytest.approx(1.,abs=1e-12)

@pytest.mark.parametrize('sign',[1,-1])
def test_hand_derived_course_rotation(sign):
 # Break: rotation uses opposite angle, giving wrong bank correction sign.
 got,heading,active=proxy_target([1,0],[0,0],0,[.1,sign*.1],'forward')
 np.testing.assert_allclose(got,[2**-.5,-sign*2**-.5],atol=1e-15);assert active

@pytest.mark.parametrize('velocity',[(.1,0),(.01,.01)])
def test_aligned_and_invalid_course_exact_real_requests(velocity):
 # Break: candidate changes unrelated gain/latch/mapping or low-speed fallback.
 b,c=tracker(),tracker(True)
 for xy in [(0,0),(.2,0),(.4,0)]:np.testing.assert_array_equal(b.act(obs(xy,v=velocity),[4,0]),c.act(obs(xy,v=velocity),[4,0]))
 assert c._tracking_audit['root_heading_rad']==0

def test_sideways_course_changes_actual_protocol_request_not_reference():
 # Break: new field telemetry only; applied request consumer remains body yaw.
 b,c=tracker(),tracker(True);a=b.act(obs(v=(.1,.1)),[4,0]);d=c.act(obs(v=(.1,.1)),[4,0]);assert not np.array_equal(a,d)
 np.testing.assert_array_equal(b._reference_xy,c._reference_xy)
 assert c._tracking_audit['root_heading_rad']==0 and c._tracking_audit['velocity_pursuit_active']
 assert c._tracking_audit['actual_reference_lookahead_world_xy']==[1.,0.] # frozen adaptive lookahead, unchanged by candidate

def test_direction_change_five_low_and_rebound_are_unchanged():
 # Break: heading intervention changes braking or counts stale observations.
 b,c=tracker(directions=[1,-1,-1,-1,-1]),tracker(True,[1,-1,-1,-1,-1])
 for t in (b,c):t._path_index=1
 for speed,count in [(.04,1),(.04,2),(.06,0),(.04,1),(.04,2),(.04,3),(.04,4)]:
  for t in (b,c):np.testing.assert_array_equal(t.act(obs((.5,0),v=(speed,0)),[4,0]),np.zeros(10,np.float32));assert t._stopped_cycles==count
  assert not c._tracking_audit['velocity_pursuit_evaluated']
 for t in (b,c):t.act(obs((.5,0),v=(.04,0)),[4,0]);assert t._active_drive_direction=='reverse' and t._pending_drive_direction is None and t._stopped_cycles==0
