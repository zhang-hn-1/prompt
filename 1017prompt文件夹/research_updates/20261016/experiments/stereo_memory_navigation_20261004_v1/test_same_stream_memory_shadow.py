import copy
import numpy as np
import pytest
from same_stream_memory_shadow import ReadOnlyMemoryShadow
from prior_rgb_navigation import PriorRGBPolicy
from test_prior_rgb_navigation import source
from test_depth_local_curve import obs,protocol
from guarded_reference import boundary_state

def empty(t,i):
 s=source(t,i);s.update(world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,int),partner_pixel_id=np.empty(0,int));return s

def test_shadow_inputs_are_independent_copies():
 sh=ReadOnlyMemoryShadow(protocol());s=source(6.,0);p=dict(frame=0,time_s=6.,points_world=s['world'],root_xyz=np.array([0.,0.,.4]));o=obs();before=s['world'].copy()
 result=sh.consume(s,p,o,[2.,0.]);sh.policy._source['world'][0]=99
 np.testing.assert_equal(s['world'],before);assert result['read_only'] and len(result['request'])==10

def test_future_source_rejected_before_payload():
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return super().__getitem__(k)
 sh=ReadOnlyMemoryShadow(protocol())
 with pytest.raises(ValueError):sh.consume(Bomb(time_s=7.),dict(time_s=6.),obs(),[2,0])

def test_primary_requests_state_same_with_shadow_on_off():
 a=PriorRGBPolicy(protocol(),'current');b=PriorRGBPolicy(protocol(),'current');sh=ReadOnlyMemoryShadow(protocol())
 for i,t in enumerate([6.,6.1,6.5]):
  s=empty(t,i);p=dict(frame=i,time_s=t,points_world=s['world'],root_xyz=np.array([0.,0.,.4]));o=obs()
  for policy in (a,b):policy.set_source_packet(copy.deepcopy(s));policy.set_current_packet(copy.deepcopy(p))
  A=a.act(o,[2,0]);B=b.act(o,[2,0]);state=copy.deepcopy(boundary_state(b.tracker));sh.consume(s,p,o,[2,0]);np.testing.assert_equal(A,B);assert boundary_state(a.tracker)==boundary_state(b.tracker)==state
