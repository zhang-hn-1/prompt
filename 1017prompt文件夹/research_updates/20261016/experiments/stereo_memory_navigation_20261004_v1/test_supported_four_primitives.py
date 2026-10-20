import sys,dataclasses
sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
import numpy as np,pytest
from supported_four_primitive_source import supported_curvature,supported_requests,source_identity

def test_only_admitted_request_points_are_enumerated_and_queried():
 p=supported_requests();assert len(p)==4 and (-.3,0.) in p
 for main,aux in p:assert np.isfinite(supported_curvature(main,aux))
 for main,aux in [(-.5,0),(-.3,.1),(.3,0),(.5,np.pi/12),(1.,0)]:assert supported_curvature(main,aux) is None
 assert source_identity()['not_dynamic_prediction']

@pytest.mark.parametrize('main,aux',[(np.nan,0),(.5,np.inf)])
def test_invalid_point_query_is_rejected(main,aux):
 with pytest.raises(ValueError):supported_curvature(main,aux)

def test_disabled_source_exactly_preserves_original_search():
 from benchmark.drive_model import DriveModel
 from benchmark.policies import DEFAULT_DRIVE_PROFILE
 import direction_root_planning as old
 import supported_four_point_planning as new
 kw=dict(occupancy_grid=np.zeros((80,80),np.uint8),resolution_m=.1,origin_xy=(-4.,-4.),start_xy=(0.,0.),start_yaw_rad=0.,goal_xy=(1.,.5),node_budget=3000);d=DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
 assert dataclasses.asdict(old.plan_kinodynamic_path(old.KinodynamicPlanRequest(**kw),d))==dataclasses.asdict(new.plan_kinodynamic_path(new.KinodynamicPlanRequest(**kw),d))

def test_supported_reverse_request_is_reachable_by_the_actual_primitive_enumerator():
 from benchmark.drive_model import DriveModel
 from benchmark.policies import DEFAULT_DRIVE_PROFILE
 from supported_four_point_planning import KinodynamicPlanRequest,plan_kinodynamic_path
 d=DriveModel.from_json(DEFAULT_DRIVE_PROFILE);kw=dict(occupancy_grid=np.zeros((80,80),np.uint8),resolution_m=.1,origin_xy=(-4.,-4.),start_xy=(0.,0.),start_yaw_rad=0.,goal_xy=(-1.,0.),node_budget=3000)
 plan=plan_kinodynamic_path(KinodynamicPlanRequest(**kw),d,primitive_curvature_source=supported_curvature,primitive_requests=supported_requests())
 assert plan.status=='success' and plan.path_length_m>.5
 actions=[(w.main_command_rad_s,w.auxiliary_angle_rad) for w in plan.waypoints if w.main_command_rad_s!=0]
 assert actions and any(m==-.3 for m,a in actions)
 assert all(supported_curvature(m,a) is not None for m,a in actions)
