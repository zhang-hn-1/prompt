import sys
sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
import numpy as np,pytest
from supported_forward_primitive_source import supported_curvature

def test_only_the_three_validated_forward_request_points_have_geometry():
 assert supported_curvature(.5,0)==pytest.approx(-.03296351060271263)
 assert supported_curvature(.5,-np.pi/6)==pytest.approx(.7966033220291138)
 assert supported_curvature(.5,np.pi/6)==pytest.approx(-.8272253274917603)
 for m,a in [(-.5,0),(.3,0),(1.,0),(.5,np.pi/12),(.5,np.pi/6+.00001)]:assert supported_curvature(m,a) is None
@pytest.mark.parametrize('m,a',[(np.nan,0),(.5,np.nan),(.5,np.inf)])
def test_invalid_query_rejected_before_domain_lookup(m,a):
 with pytest.raises(ValueError):supported_curvature(m,a)

def test_source_disabled_preserves_original_search_and_supported_domain_rejects_reverse_edges():
 import dataclasses
 from benchmark.drive_model import DriveModel
 from benchmark.policies import DEFAULT_DRIVE_PROFILE
 import direction_root_planning as old
 import supported_forward_point_planning as new
 grid=np.zeros((80,80),np.uint8);kw=dict(occupancy_grid=grid,resolution_m=.1,origin_xy=(-4.,-4.),start_xy=(0.,0.),start_yaw_rad=0.,goal_xy=(1.,.5),node_budget=3000);model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
 A=old.plan_kinodynamic_path(old.KinodynamicPlanRequest(**kw),model)
 B=new.plan_kinodynamic_path(new.KinodynamicPlanRequest(**kw),model)
 assert dataclasses.asdict(A)==dataclasses.asdict(B)
 C=new.plan_kinodynamic_path(new.KinodynamicPlanRequest(**kw),model,primitive_curvature_source=supported_curvature)
 assert C.status=='success' and len(C.waypoints)>1
 assert all(w.drive_direction=='forward' and (w.main_command_rad_s==0 or w.main_command_rad_s==.5 and supported_curvature(w.main_command_rad_s,w.auxiliary_angle_rad) is not None) for w in C.waypoints)
