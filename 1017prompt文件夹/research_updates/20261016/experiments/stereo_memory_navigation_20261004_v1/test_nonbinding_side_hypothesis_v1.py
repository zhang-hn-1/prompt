import importlib.util
import numpy as np
import pytest
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from test_route_side_commitment_grid_v1 import wall

def candidate():
 assert importlib.util.find_spec('nonbinding_side_hypothesis_v1') is not None, 'nonbinding side hypothesis consumer missing'
 from nonbinding_side_hypothesis_v1 import NonbindingSideHypothesisGrid
 return NonbindingSideHypothesisGrid([-20,-20],10,static_scene_assumption=True)

def test_growing_observed_barrier_can_revise_route_without_changing_observation():
 g=candidate();hard=RouteSideCommitmentGrid([-20,-20],10,static_scene_assumption=True);base=StaticSparsePointRiskGrid([-20,-20],10,static_scene_assumption=True)
 for a in [g,hard,base]:wall(a,-1.2,.2,0.)
 first,_=g.plan([-3,0],[3,0]);old,_=hard.plan([-3,0],[3,0]);np.testing.assert_array_equal(first,old);assert g.commitment['side']==1
 for a in [g,hard,base]:wall(a,-1.2,2.,1.)
 labels=g.labels.copy();birth=g.hazard_acquisition_time.copy();route,status=g.plan([-2.2,.5],[3,0]);expected,es=base.plan([-2.2,.5],[3,0]);locked,_=hard.plan([-2.2,.5],[3,0]);np.testing.assert_array_equal(route,expected);assert status==es
 assert route[np.argmin(abs(route[:,0])),1]<0 and locked[np.argmin(abs(locked[:,0])),1]>0
 np.testing.assert_array_equal(g.labels,labels);np.testing.assert_array_equal(g.hazard_acquisition_time,birth);assert g.commitment['side']==1 and not g.commitment_released
 assert g.diagnostics()['route_side_hypothesis_is_binding'] is False

def test_same_goal_validation_and_release_remain():
 g=candidate();wall(g,-1.2,.2,0.);g.plan([-3,0],[3,0])
 with pytest.raises(ValueError):g.plan([-3,0],[3,1])
 g.plan([2.,2.],[3,0]);assert g.commitment_released

def test_no_observation_no_invented_side_or_free():
 g=candidate();g.consume_records([],0.,[-3,0,.4]);g.plan([-3,0],[3,0]);assert g.commitment is None and not (g.labels==0).any()
