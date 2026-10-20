import numpy as np,pytest
from nominal_unknown_cap import nominal_unknown_action,with_nominal_unknown_cap,ReadOnlyHistorySlowShadow
from prior_rgb_navigation import PriorRGBPolicy
from test_depth_local_curve import protocol,obs
from test_same_stream_memory_shadow import empty

def test_cap_restores_nominal_range_and_preserves_aux_zero_and_observed():
 a=np.array([1.5,.2,.5,.3,-1.5,-.4,0.,.1,1.,.25],np.float32);before=a.copy();b=nominal_unknown_action(a,1.)
 np.testing.assert_equal(b[::2],[1.,.5,-1.,0.,1.]);np.testing.assert_equal(b[1::2],a[1::2]);np.testing.assert_equal(a,before);np.testing.assert_equal(nominal_unknown_action(a,0.),a);np.testing.assert_equal(nominal_unknown_action(np.zeros(10),1.),np.zeros(10))

def test_invalid_shape_or_unknown_value_rejected():
 for x in [np.nan,-.1,1.1]:
  with pytest.raises(ValueError):nominal_unknown_action(np.zeros(10),x)
 with pytest.raises(ValueError):nominal_unknown_action(np.zeros(9),1.)

def test_wrapper_calls_inherited_act_once_and_records_final_request(monkeypatch):
 calls=[];raw=np.tile([1.5,.2],5).astype(np.float32)
 def base(self,observation,goal):
  calls.append((observation,goal));self.records.append(dict(uncapped_request=raw.tolist(),unknown_fraction=1.,request=np.tile([.3,.2],5).tolist()));return np.tile([.3,.2],5).astype(np.float32)
 monkeypatch.setattr(PriorRGBPolicy,'act',base);cls=with_nominal_unknown_cap(PriorRGBPolicy);p=cls(protocol(),'history');o=obs();goal=[2,0];a=p.act(o,goal);assert calls==[(o,goal)] and p.records[-1]['request']==a.tolist() and p.records[-1]['unknown_main_cap_rad_s']==1.;np.testing.assert_equal(a[::2],np.ones(5));np.testing.assert_equal(a[1::2],raw[1::2])

def test_slow_shadow_stays_history_at_original_cap_and_rejects_future():
 sh=ReadOnlyHistorySlowShadow(protocol());s=empty(6.,0);p=dict(frame=0,time_s=6.,points_world=s['world'],root_xyz=np.array([0.,0.,.4]));r=sh.consume(s,p,obs(),[2,0]);assert sh.policy.memory_mode=='history' and r['unknown_main_cap_rad_s']==.3 and r['source_filter']=='dynamic_current_joint_CAD' and not r['executed'] and sh.policy._source is not s
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('futurepayload')
   return super().__getitem__(k)
 with pytest.raises(ValueError):sh.consume(Bomb(time_s=7.),dict(time_s=6.),obs(),[2,0])
