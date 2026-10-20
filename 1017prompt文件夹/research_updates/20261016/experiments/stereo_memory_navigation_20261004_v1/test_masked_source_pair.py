import copy
import numpy as np
import pytest
from masked_source_pair import paired_sources,ReadOnlyMaskShadow
from test_same_stream_memory_shadow import empty
from test_depth_local_curve import obs,protocol
from prior_rgb_navigation import PriorRGBPolicy
from guarded_reference import boundary_state

def inputs():
 z=np.ones((8,8));d=np.ones((8,8))*2;pred=dict(depth_m=z,confidence=np.ones((8,8),bool),disparity_native_px=d,frame=0,time_s=6.)
 pose=dict(left_camera_usd_world=np.eye(4),right_camera_usd_world=np.eye(4),root_position_world_m=np.zeros(3),time_s=6.)
 class CAD:
  frozen={'left':np.zeros((8,8),bool),'right':np.zeros((8,8),bool)}
  def project(self,joints):
   assert joints.tolist()==[.1,.2,.3];m={k:v.copy() for k,v in self.frozen.items()};m['left'][4,2]=True;m['right'][2,4]=True;return m
 h=np.zeros((2,17));h[-1,9:12]=[.1,.2,.3]
 return pred,pose,h,CAD()

def test_same_raw_pair_removes_only_covered_query_or_partner():
 p,k,h,c=inputs();old,new,m=paired_sources(p,k,h,c,6.);before=copy.deepcopy(p)
 assert len(old['world'])-len(new['world'])==2
 removed=set(old['query_pixel_id'])-set(new['query_pixel_id']);assert removed=={34,36}
 np.testing.assert_equal(p,before)

def test_future_rejected_without_access_to_source_payload():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload')
   return super().__getitem__(k)
 p,k,h,c=inputs()
 with pytest.raises(ValueError):paired_sources(Bomb(time_s=7.),k,h,c,6.)

def test_truth_fields_and_invalid_current_proprio_rejected():
 p,k,h,c=inputs();p['future_truth']=1
 with pytest.raises(ValueError):paired_sources(p,k,h,c,6.)
 del p['future_truth'];h[-1,10]=np.nan
 with pytest.raises(ValueError):paired_sources(p,k,h,c,6.)

def test_mask_shadow_inputs_independent_primary_same_with_on_off():
 a=PriorRGBPolicy(protocol(),'current');b=PriorRGBPolicy(protocol(),'current');sh=ReadOnlyMaskShadow(protocol())
 for i,t in enumerate([6.,6.1,6.5]):
  s=empty(t,i);p=dict(frame=i,time_s=t,points_world=s['world'],root_xyz=np.array([0.,0.,.4]));o=obs()
  for policy in (a,b):policy.set_source_packet(copy.deepcopy(s));policy.set_current_packet(copy.deepcopy(p))
  A=a.act(o,[2,0]);B=b.act(o,[2,0]);state=copy.deepcopy(boundary_state(b.tracker));r=sh.consume(s,p,o,[2,0]);np.testing.assert_equal(A,B);assert boundary_state(a.tracker)==boundary_state(b.tracker)==state;assert r['source_filter']=='frozen_base_only' and not r['executed']
  assert sh.policy is not b and sh.policy.grid is not b.grid
