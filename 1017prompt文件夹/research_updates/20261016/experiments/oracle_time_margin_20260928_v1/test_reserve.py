from pathlib import Path
import numpy as np
from benchmark.drive_model import DriveModel
from reserve_planner import KinodynamicPlanRequest,plan_kinodynamic_path
MODEL=DriveModel.from_json(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json'))
def test_planned_commands_and_integrated_geometry_reserve_curvature():
    request=KinodynamicPlanRequest(occupancy_grid=np.zeros((61,61),np.uint8),resolution_m=.1,origin_xy=(-3.,-3.),start_xy=(0.,0.),start_yaw_rad=0.,goal_xy=(1.2,.5),node_budget=3000)
    plan=plan_kinodynamic_path(request,MODEL)
    assert plan.status=='success'
    for a,b in zip(plan.waypoints,plan.waypoints[1:]):
        k=MODEL.curvature(a.main_command_rad_s,a.auxiliary_angle_rad)
        assert abs(k)<=.5+1e-8
        sign=1 if a.drive_direction=='forward' else -1
        dyaw=(b.yaw_rad-a.yaw_rad+np.pi)%(2*np.pi)-np.pi
        np.testing.assert_allclose(dyaw,k*sign*.3,atol=1e-8)

def test_all_candidate_primitives_have_curvature_reserve_and_legal_targets():
    for main in [-1.5,-1.,-.5,.5,1.,1.5]:
        for target in [0.,-.25,.25,-.5,.5]:
            angle=MODEL.auxiliary_for_curvature(main,target)
            assert abs(angle)<=np.pi/6+1e-8
            assert abs(MODEL.curvature(main,angle))<=.5+1e-8
