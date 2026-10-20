import math
import pytest
from test_reserve import MODEL
import reserve_planner as p

def test_cost_accounts_for_B0_curvature_speed_cap():
    f=getattr(p,'execution_time_s',None)
    assert callable(f),'missing tracker-aware time score'
    aux=MODEL.auxiliary_for_curvature(1.5,.5)
    effective_aux=MODEL.auxiliary_for_curvature(1.,MODEL.curvature(1.5,aux))
    assert f(1.5,aux,MODEL)==pytest.approx(.3/MODEL.speed(1.,effective_aux))
    assert f(1.5,aux,MODEL)>.3/MODEL.speed(1.5,aux)

def test_blocked_geometry_is_not_mislabeled_as_time_budget():
    import numpy as np
    grid=np.ones((61,61),np.uint8);grid[30,30]=0;grid[45,30]=0
    req=p.KinodynamicPlanRequest(occupancy_grid=grid,resolution_m=.1,origin_xy=(-3.,-3.),start_xy=(0.,0.),start_yaw_rad=0.,goal_xy=(1.5,0.))
    assert p.plan_kinodynamic_path(req,MODEL).status=='unreachable'
