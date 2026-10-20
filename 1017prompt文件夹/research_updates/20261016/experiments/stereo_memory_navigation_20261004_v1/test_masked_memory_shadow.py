import numpy as np,pytest
from masked_memory_shadow import ReadOnlyCurrentMemoryShadow
from test_same_stream_memory_shadow import empty
from test_depth_local_curve import protocol,obs

def test_current_shadow_uses_same_dynamic_source_and_own_state():
 sh=ReadOnlyCurrentMemoryShadow(protocol());s=empty(6.,0);p=dict(frame=0,time_s=6.,points_world=s['world'],root_xyz=np.array([0.,0.,.4]));r=sh.consume(s,p,obs(),[2,0]);assert sh.policy.memory_mode=='current' and r['source_filter']=='dynamic_current_joint_CAD' and r['memory_role']=='current_source_only' and r['read_only'] and not r['executed'];assert sh.policy._source is not s

def test_future_masked_source_rejected_before_payload():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('futurepayloadread')
   return super().__getitem__(k)
 with pytest.raises(ValueError):ReadOnlyCurrentMemoryShadow(protocol()).consume(Bomb(time_s=7.),dict(time_s=6.),obs(),[2,0])
