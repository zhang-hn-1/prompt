import numpy as np
import pytest
from prior_rgb_navigation import short_goal,require_boolean_confidence,PriorRGBGrid,PriorRGBPolicy
from test_depth_local_curve import obs,protocol

def source(t=0.,frame=0,point=(1.,0.,.8)):
 return dict(world=np.array([point]),sigma_m=np.array([.01]),query_pixel_id=np.array([1]),partner_pixel_id=np.array([2]),frame=frame,time_s=t,pose_sigma_m=0.,source_model_admitted=False,certified_free=False,frame_width=640)

def test_short_goal_uses_only_known_task_goal():
 assert np.allclose(short_goal([0,0,0],[3,4,0]),[1.2,1.6,0])
 with pytest.raises(ValueError):short_goal([0,0,0],[0,0,0])

def test_numeric_or_nan_confidence_not_accepted():
 for a in [np.array([np.nan]),np.array([1.])]:
  with pytest.raises(ValueError):require_boolean_confidence(a)
 assert require_boolean_confidence(np.array([True,False])).dtype==bool

def test_cached_source_age_not_refreshed_and_prior_not_observed():
 g=PriorRGBGrid([-2,-2],shape=(41,41),memory_mode='history');g.set_source(source());g.update_points([],np.array([0,0,.4]),0.)
 g.update_points([],np.array([0,0,.4]),5.);assert np.max(g.memory.hazard_time[np.isfinite(g.memory.hazard_time)])==0.
 assert g.task_ground_prior.all() and np.any(g.safe_start_prior) and np.all(g.labels[g.safe_start_prior]!=0)
 g.update_points([],np.array([0,0,.4]),10.1);assert not (g.labels==1).any()

def test_memory_changes_retained_obstacle_but_never_future():
 a=PriorRGBGrid([-2,-2],shape=(41,41),memory_mode='current');b=PriorRGBGrid([-2,-2],shape=(41,41),memory_mode='history')
 for g in [a,b]:
  g.set_source(source());g.update_points([],np.array([0,0,.4]),0.)
  g.set_source(source(1.,1,(-1,0,.8)));g.update_points([],np.array([0,0,.4]),1.)
 assert (a.labels==1).sum()==1 and (b.labels==1).sum()==2
 with pytest.raises(ValueError):a.set_source(source(2.,2));a.update_points([],np.array([0,0,.4]),1.1)

def test_both_arms_share_unknown_cap_and_real_local_request():
 for arm in ['current','history']:
  p=PriorRGBPolicy(protocol(),memory_mode=arm);p.set_source_packet(dict(source(6.),world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,int),partner_pixel_id=np.empty(0,int)))
  p.set_current_packet(dict(frame=0,time_s=6.,points_world=np.empty((0,3)),root_xyz=np.array([0,0,.4])))
  request=p.act(obs(),[2,0]);assert 0<request[0]<=.3000001
  assert not p.records[-1]['certified_safe'] and p.records[-1]['source_layer']=='RGB_depth_knownpose'
