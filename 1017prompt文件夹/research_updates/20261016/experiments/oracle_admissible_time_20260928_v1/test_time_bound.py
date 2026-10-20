import math
import numpy as np
import pytest
import reserve_planner as p
from test_reserve import MODEL

def test_euclidean_bound_not_octile_overestimate():
    f=getattr(p,'remaining_time_lower_bound_s',None)
    assert callable(f)
    h=f(0.,0.,(2.,1.),0.,.6)
    assert h==pytest.approx(math.sqrt(5)/.6)
    assert 31+h<=35 < 31+(1+math.sqrt(2))/.6

def test_overestimating_guide_cannot_prune_all_feasible_paths(monkeypatch):
    def exaggerated_guide(grid,**kwargs):return np.full(np.asarray(grid).shape,1000.)
    monkeypatch.setattr(p,'_grid_distance_to_goal',exaggerated_guide)
    req=p.KinodynamicPlanRequest(occupancy_grid=np.zeros((41,41),np.uint8),resolution_m=.1,origin_xy=(-2.,-2.),start_xy=(0.,0.),start_yaw_rad=0.,goal_xy=(.6,0.),node_budget=2000)
    result=p.plan_kinodynamic_path(req,MODEL)
    assert result.status=='success'
    assert result.estimated_time_s<=35

def test_sample_max_speed_bounds_all_primitive_execution_speeds():
    bound=max(s.speed_m_s for s in MODEL.steering_samples)
    for main in [-1.5,-1.,-.5,.5,1.,1.5]:
        for k in [0.,-.25,.25,-.5,.5]:
            aux=MODEL.auxiliary_for_curvature(main,k)
            assert .3/p.execution_time_s(main,aux,MODEL)<=bound+1e-12
