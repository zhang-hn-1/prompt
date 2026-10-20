import numpy as np,pytest
from terminal_goal_consumer import terminal_action,with_terminal_goal_consumer
from test_depth_local_curve import protocol,obs
from prior_rgb_navigation import PriorRGBPolicy

def test_distance_boundary_signed_main_aux_zeros_and_input_copy():
 a=np.array([1.,.2,-1.,-.3,0.,.1,.2,.4,-.2,-.5],np.float32);old=a.copy()
 np.testing.assert_equal(terminal_action(a,.600001,False)[0],a)
 b,latched=terminal_action(a,.6,False);assert latched
 np.testing.assert_allclose(b[::2],[.3,-.3,0,.2,-.2],atol=1e-8);np.testing.assert_equal(b[1::2],a[1::2]);np.testing.assert_equal(a,old)
 c,on=terminal_action(a,1.,latched);np.testing.assert_equal(c,b);assert on

def test_invalid_values_never_enter_policy():
 for d in [-1,np.nan,np.inf]:
  with pytest.raises(ValueError):terminal_action(np.zeros(10),d,False)
 for a in [np.zeros(9),np.full(10,np.nan)]:
  with pytest.raises(ValueError):terminal_action(a,1.,False)

def test_parent_act_once_latch_reset_final_diagnostic(monkeypatch):
 calls=[];raw=np.tile([1.5,.2],5).astype(np.float32)
 def base(self,observation,goal):
  calls.append(1);self.records.append(dict(uncapped_request=raw.tolist(),unknown_fraction=1.,request=raw.tolist()));return raw.copy()
 monkeypatch.setattr(PriorRGBPolicy,'act',base)
 cls=with_terminal_goal_consumer(PriorRGBPolicy);p=cls(protocol(),'history');o=obs();a=p.act(o,[.6,0]);assert len(calls)==1 and p.terminal_mode
 np.testing.assert_allclose(a[::2],.3);assert p.records[-1]['request']==a.tolist();assert p.records[-1]['terminal_mode'];assert p.records[-1]['pre_terminal_request'][0]==1.
 p.act(obs(-.5),[1.1,0]);assert len(calls)==2 and p.terminal_mode
 q=cls(protocol(),'history');assert not q.terminal_mode
 monkeypatch.setattr(PriorRGBPolicy,'reset',lambda self,c:None);p.reset(None);assert not p.terminal_mode
