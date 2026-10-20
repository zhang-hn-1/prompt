import numpy as np,pytest
from terminal_goal_bank_source import TerminalGoalBankSource
def block(m,a):return np.tile(np.array([m,a],np.float32),5)
def test_only_fresh_goal_braking_source_changes_bank_and_preserves_main():
 c=TerminalGoalBankSource();c.apply(block(.5,.4),'tracking')
 for source in ['direction_change','goal_reached','local_no_route']:
  x,info=c.apply(block(0,0),source);np.testing.assert_array_equal(x,block(0,0));assert not info['active']
 parent=block(0,0);x,info=c.apply(parent,'goal_braking');np.testing.assert_array_equal(x,block(0,.4));np.testing.assert_array_equal(parent,block(0,0));assert info['active']
 np.testing.assert_array_equal(x[::2],parent[::2])
def test_resume_reset_and_invalid_source_no_future():
 c=TerminalGoalBankSource();x,_=c.apply(block(0,0),'goal_braking');assert not x.any()
 c.apply(block(.5,.3),'tracking');c.apply(block(0,0),'goal_braking');x,_=c.apply(block(-.3,-.1),'tracking');np.testing.assert_array_equal(x,block(-.3,-.1))
 x,_=c.apply(block(0,0),'goal_braking');np.testing.assert_array_equal(x,block(0,-.1))
 with pytest.raises(ValueError):c.apply(block(0,0),None)
 with pytest.raises(ValueError):c.apply(np.full(10,np.nan),'goal_braking')
 fresh=TerminalGoalBankSource();x,_=fresh.apply(block(0,0),'goal_braking');assert not x.any()
def test_wrapper_calls_parent_once_and_keeps_tracker_stop_counter():
 from types import SimpleNamespace
 from terminal_goal_bank_source import with_terminal_goal_bank
 class Parent:
  def reset(self,context):self.calls=0;self.records=[];self.tracker=SimpleNamespace(request_diagnostics={},_stopped_cycles=4)
  def act(self,observation,goal):
   self.calls+=1;request,source=observation;self.tracker.request_diagnostics={'request_source':source};self.records.append({});return request
 p=with_terminal_goal_bank(Parent)();p.reset(None);p.act((block(.5,.4),'tracking'),None);before=p.tracker._stopped_cycles
 a=p.act((block(0,0),'goal_braking'),None);np.testing.assert_array_equal(a,block(0,.4));assert p.calls==2 and p.tracker._stopped_cycles==before
 assert p.records[-1]['parent_final_request']==block(0,0).tolist()
 p.reset(None);a=p.act((block(0,0),'goal_braking'),None);assert not a.any() and p.calls==1
