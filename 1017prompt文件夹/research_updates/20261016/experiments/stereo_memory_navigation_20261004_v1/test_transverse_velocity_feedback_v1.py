from types import SimpleNamespace
import numpy as np
import pytest
from transverse_velocity_feedback_v1 import with_transverse_velocity_feedback
class Parent:
 def __init__(self):
  from scripts.run_pointgoal_benchmark import load_action_protocol
  from pathlib import Path
  self.protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));self.tracker=SimpleNamespace(_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking');self.records=[];self.n=0;self.vy=.1;self.fresh=True
 def act(self,o,g):
  self.n+=1;self.records.append(dict(current_tracking_chain=dict(fresh_tracker_record=self.fresh,valid=True,source='tracking',reference_drive_direction='forward',reference_motion_tangent_rad=0.,root_heading_rad=0.,velocity_world_x_m_s=.2,velocity_world_y_m_s=self.vy),request=[1.,.25]*5));return np.array([1.,.25]*5,np.float32)
@pytest.mark.parametrize('vy',[.1,-.1,0.])
def test_sign_main_single_parent_and_zero_equal(vy):
 p=with_transverse_velocity_feedback(Parent,enabled=True)();p.vy=vy;out=p.act(None,None);assert p.n==1;np.testing.assert_array_equal(out[::2],np.ones(5,np.float32));assert np.all(out[1::2]==np.float32(.25+vy))
 assert p.records[-1]['transverse_feedback_override']==(vy!=0)
def test_stale_finite_payload_not_read():
 p=with_transverse_velocity_feedback(Parent,enabled=True)();p.fresh=False;p.vy=float('nan');np.testing.assert_array_equal(p.act(None,None),np.array([1.,.25]*5,np.float32));assert not p.records[-1]['transverse_feedback_eligible']
def test_reverse_preserved():
 p=with_transverse_velocity_feedback(Parent,enabled=True)();p.tracker._active_drive_direction='reverse';np.testing.assert_array_equal(p.act(None,None),np.array([1.,.25]*5,np.float32))

def test_opposite_tangent_sign_domain_parent_preserved():
 class Opposite(Parent):
  def act(self,o,g):
   a=super().act(o,g);self.records[-1]['current_tracking_chain']['reference_motion_tangent_rad']=np.pi;return a
 p=with_transverse_velocity_feedback(Opposite,enabled=True)();np.testing.assert_array_equal(p.act(None,None),np.array([1.,.25]*5,np.float32));assert not p.records[-1]['transverse_feedback_eligible']
