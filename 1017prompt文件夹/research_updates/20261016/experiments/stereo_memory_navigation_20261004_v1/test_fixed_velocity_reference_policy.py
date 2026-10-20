import numpy as np,pytest
from fixed_velocity_reference_policy import FixedReferencePolicy
from test_velocity_pursuit_consumer import obs
from depth_local_curve import P
from training.action_protocol import load_action_protocol

def setup(variant):return FixedReferencePolicy(load_action_protocol(P/'configs/data_contract.yaml'),variant)
def source(i,t):
 return dict(frame=i,time_s=t,frame_width=640,world=np.tile([-0.8,0,.5],(100,1)),sigma_m=np.zeros(100)+.01,query_pixel_id=np.arange(100),partner_pixel_id=np.arange(100)+100,pose_sigma_m=0.)
def advance(p,i,t,v=(.01,0)):
 o=obs((-2.,.23),-.3385734,v);p.set_source_packet(source(i,t));p.set_current_packet(dict(frame=i,time_s=t,points_world=p._source['world'],root_xyz=np.array([-2.,.23,.4])));return p.act(o,[1.,0])
def test_same_fixed_prefix_and_no_replan_no_midsequence_reset():
 b,c=setup('B0'),setup('velocity')
 for i,t in enumerate([0,.1,6,12,17.6,17.7,17.8,17.9,18.]):
  np.testing.assert_array_equal(advance(b,i,t),advance(c,i,t));np.testing.assert_array_equal(b.tracker._reference_xy,c.tracker._reference_xy)
 assert b.gate.handed_off and c.gate.handed_off
 for p in (b,c):assert len(p.records)==9 and p.local_reference_version==1 and p.plan is not None
 assert not np.array_equal(advance(b,9,18.1,(.1,.1)),advance(c,9,18.1,(.1,.1)))
def test_future_source_rejected_before_plan_control():
 p=setup('velocity');p.set_source_packet(source(0,2));p.set_current_packet(dict(frame=0,time_s=1,points_world=np.zeros((0,3)),root_xyz=np.array([-2.,.23,.4])))
 with pytest.raises(ValueError):p.act(obs((-2.,.23)),[1.,0])
