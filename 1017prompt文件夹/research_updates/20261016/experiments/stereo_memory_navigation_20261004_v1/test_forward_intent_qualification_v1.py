from types import SimpleNamespace
import numpy as np
from current_forward_intent_qualification_v1 import measured_forward_intent

def observed(speed=.331160509,joint=.85031396):
 p=np.zeros((5,17));p[-1,0]=speed;p[-1,12]=joint
 return SimpleNamespace(proprio_history=p,yaw_world_rad=0.,quaternion_world_wxyz_history=np.tile([1.,0.,0.,0.],(5,1)))
def normal():return dict(active_direction='forward',pending_direction=None,braking_state='tracking')
def test_current_forward_intent_does_not_claim_empirical_coast_domain():
 assert measured_forward_intent(observed(),normal())=='forward'
 assert measured_forward_intent(observed(.2,.45),normal())=='forward'
def test_braking_pending_reverse_or_not_forward_velocity_is_ineligible():
 for change in [dict(braking_state='braking'),dict(pending_direction='reverse'),dict(active_direction='reverse')]:assert measured_forward_intent(observed(),dict(normal(),**change)) is None
 assert measured_forward_intent(observed(-.2,.8),normal()) is None
 assert measured_forward_intent(observed(.04,.8),normal()) is None
 assert measured_forward_intent(observed(.3,-.3),normal()) is None

def test_wrapper_calls_stateful_parent_once_and_leaves_action_unchanged(monkeypatch):
 import minimum_prefix_transition_consumer_v1 as original
 from current_forward_intent_qualification_v1 import with_current_forward_intent
 monkeypatch.setattr(original,'boundary_state',lambda tracker:normal())
 import current_forward_intent_qualification_v1 as candidate_module
 monkeypatch.setattr(candidate_module,'boundary_state',lambda tracker:normal())
 class Parent:
  def reset(self,context):self.tracker=SimpleNamespace(_context=object());self.records=[];self.calls=0
  def act(self,observation,goal):self.calls+=1;self.records.append({});return np.array([.8,-.2])
 policy=with_current_forward_intent(Parent)();policy.reset(None);action=policy.act(observed(),None)
 assert policy.calls==1 and policy.current_first_direction=='forward'
 np.testing.assert_array_equal(action,[.8,-.2])

def test_nonfinite_current_or_past_measurement_cannot_become_forward():
 import pytest
 obs=observed();obs.proprio_history[0,0]=np.nan
 with pytest.raises(ValueError):measured_forward_intent(obs,normal())
 obs=observed();obs.yaw_world_rad=float('nan')
 with pytest.raises(ValueError):measured_forward_intent(obs,normal())

def test_same_wrapper_b0_preserves_original_narrow_qualification(monkeypatch):
 import minimum_prefix_transition_consumer_v1 as old
 import current_forward_intent_qualification_v1 as current
 monkeypatch.setattr(old,'boundary_state',lambda tracker:normal());monkeypatch.setattr(current,'boundary_state',lambda tracker:normal())
 class Parent:
  def reset(self,context):self.tracker=SimpleNamespace(_context=object());self.records=[];self.calls=0
  def act(self,observation,goal):self.calls+=1;self.records.append({});return np.array([.8,-.2])
 a=current.with_current_forward_intent(Parent,candidate=False)();b=current.with_current_forward_intent(Parent,candidate=True)()
 a.reset(None);b.reset(None);x=a.act(observed(),None);y=b.act(observed(),None)
 assert a.calls==b.calls==1 and a.current_first_direction is None and b.current_first_direction=='forward'
 np.testing.assert_array_equal(x,y)
 assert not a.records[-1]['forward_intent_decoupling_qualified'] and b.records[-1]['forward_intent_decoupling_qualified']

def test_real_factory_has_one_direction_wrapper_and_identical_tracker_identity():
 from forward_intent_policy_factory_v1 import policy_class
 from scripts.run_pointgoal_benchmark import load_action_protocol
 from pathlib import Path
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 a=policy_class('b0')(protocol,memory_mode='current');b=policy_class('candidate')(protocol,memory_mode='current')
 assert [str(c) for c in type(a).__mro__]==[str(c) for c in type(b).__mro__]
 assert [str(c) for c in type(a.tracker).__mro__]==[str(c) for c in type(b.tracker).__mro__]
 assert sum(c.__name__=='FirstPrimitiveDirection' for c in type(a).__mro__)==1
 assert sum(c.__name__=='CurrentForwardIntent' for c in type(a).__mro__)==1
