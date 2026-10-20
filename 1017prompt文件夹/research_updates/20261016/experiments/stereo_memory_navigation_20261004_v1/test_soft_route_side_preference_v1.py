import numpy as np
import pytest
from soft_route_side_preference_v1 import weighted_astar,preference_weights
from benchmark.grid_planning import astar_path

def test_uniform_cost_exact_old_astar_and_finite_preference():
 occupied=np.zeros((9,9),np.uint8);occupied[3:6,3:6]=1
 old=astar_path(occupied,start=(4,0),goal=(4,8))
 np.testing.assert_array_equal(weighted_astar(occupied,np.ones((9,9)),start=(4,0),goal=(4,8)),old)
 w=np.ones((9,9));w[:4,:]=1.25
 path=weighted_astar(occupied,w,start=(4,0),goal=(4,8))
 assert path[:,0].max()>5 and not occupied[tuple(path.T)].any()
 # An opposite-side endpoint remains reachable; preference is not a ban.
 assert tuple(weighted_astar(occupied,w,start=(0,0),goal=(0,8))[-1])==(0,8)

def test_corner_hazard_and_nonfinite_cost_never_ignored():
 b=np.array([[0,1],[1,0]],np.uint8)
 with pytest.raises(ValueError):weighted_astar(b,np.ones((2,2)),start=(0,0),goal=(1,1))
 w=np.ones((2,2));w[0,1]=np.nan
 with pytest.raises(ValueError):weighted_astar(np.zeros((2,2)),w,start=(0,0),goal=(1,1))

def test_hypothesis_is_current_prior_and_doesnot_modify_labels():
 labels=np.full((7,7),-1);labels[3,3]=1;old=labels.copy()
 c=dict(side=-1,longitudinal_span_m=[1.,5.],axis_start=[0.,0.],axis=[1.,0.],decision_time_s=2.,observed_risk_birth_max_s=2.,planning_prior_not_observation=True)
 w=preference_weights(labels.shape,np.array([0.,-3.]),1.,c,current_time=2.)
 assert w[2,5]==1.25 and w[2,1]==1. and w[0,5]==1.;np.testing.assert_array_equal(labels,old)
 with pytest.raises(ValueError):preference_weights(labels.shape,np.array([0.,-3.]),1.,c,current_time=1.9)

def test_nonfinite_prior_clock_and_future_release_cannot_enter_geometry():
 from soft_route_side_preference_v1 import SoftRouteSidePreferenceGrid
 c=dict(side=-1,longitudinal_span_m=[1.,5.],axis_start=[0.,0.],axis=[1.,0.],decision_time_s=2.,observed_risk_birth_max_s=2.,planning_prior_not_observation=True)
 for key in ('decision_time_s','observed_risk_birth_max_s'):
  bad=dict(c,**{key:float('nan')})
  with pytest.raises(ValueError):preference_weights((7,7),np.array([0.,-3.]),1.,bad,current_time=2.)

def test_future_prior_release_cannot_enter_geometry():
 from soft_route_side_preference_v1 import SoftRouteSidePreferenceGrid
 c=dict(side=-1,longitudinal_span_m=[1.,5.],axis_start=[0.,0.],axis=[1.,0.],decision_time_s=2.,observed_risk_birth_max_s=2.,planning_prior_not_observation=True)
 # Robot beyond a claimed span must not silently release a future prior.
 g=SoftRouteSidePreferenceGrid(np.array([-10.,-10.]),10,static_scene_assumption=True);g.commitment=dict(c,decision_time_s=3.);g.commitment_released=False;g._axis_start=np.zeros(2);g._axis=np.array([1.,0.]);g._goal=np.array([8.,0.]);g.time=2.
 with pytest.raises(ValueError,match='current/past'):g.plan(np.array([6.,0.]),g._goal)
