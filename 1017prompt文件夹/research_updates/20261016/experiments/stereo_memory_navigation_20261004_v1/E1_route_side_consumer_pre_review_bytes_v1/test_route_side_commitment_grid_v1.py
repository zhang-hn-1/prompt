import numpy as np,pytest
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid

def wall(grid,low,high,t):
 p=np.column_stack([np.zeros(len(np.arange(low,high+.05,.1))),np.arange(low,high+.05,.1),np.full(len(np.arange(low,high+.05,.1)),.4)])
 grid.consume_records([dict(time_s=t,world=p)],t,[-3,0,.4])
def test_first_route_identity_then_side_survives_barrier_growth():
 from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
 g=RouteSideCommitmentGrid([-20,-20],10,static_scene_assumption=True);b=StaticSparsePointRiskGrid([-20,-20],10,static_scene_assumption=True)
 for a in [g,b]:wall(a,-1.2,.2,0.)
 a,status=g.plan([-3,0],[3,0]);base,_=b.plan([-3,0],[3,0]);np.testing.assert_array_equal(a,base);assert g.commitment['side']==1
 for a in [g,b]:wall(a,-1.2,2.,1.)
 labels=g.labels.copy();clock=g.hazard_acquisition_time.copy();route,_=g.plan([-2.2,.5],[3,0]);free,_=b.plan([-2.2,.5],[3,0]);assert route[np.argmin(abs(route[:,0])),1]>0 and free[np.argmin(abs(free[:,0])),1]<0
 np.testing.assert_array_equal(g.labels,labels);np.testing.assert_array_equal(g.hazard_acquisition_time,clock);assert not (g.labels==0).any()
def test_no_crossing_component_no_invented_latch_and_reset():
 g=RouteSideCommitmentGrid([-20,-20],10,static_scene_assumption=True);g.consume_records([],0.,[-3,0,.4]);g.plan([-3,0],[3,0]);assert g.commitment is None
 wall(g,-1.2,.2,1.);g.plan([-3,0],[3,0]);assert g.commitment is not None
 fresh=RouteSideCommitmentGrid([-20,-20],10,static_scene_assumption=True);assert fresh.commitment is None
 with pytest.raises(ValueError):g.plan([-3,0],[3,1])
def test_release_does_not_write_observed_free():
 g=RouteSideCommitmentGrid([-20,-20],10,static_scene_assumption=True);wall(g,-1.2,.2,0.);g.plan([-3,0],[3,0]);before=g.labels.copy();g.plan([2.,2.],[3,0]);assert g.commitment_released
 np.testing.assert_array_equal(g.labels,before)
