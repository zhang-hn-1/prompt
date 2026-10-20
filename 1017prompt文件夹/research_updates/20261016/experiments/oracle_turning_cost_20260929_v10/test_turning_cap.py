"""A bounded path experiment: avoid reference turns that force the 0.5 tier."""

from pathlib import Path

import numpy as np

from benchmark.drive_model import DriveModel
from benchmark.kinodynamic_planning import KinodynamicPlanRequest
from time_planning import plan_kinodynamic_path


PROFILE = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json')


def test_planned_reference_never_crosses_slow_turn_threshold():
    model = DriveModel.from_json(PROFILE)
    request = KinodynamicPlanRequest(
        occupancy_grid=np.zeros((51, 51), dtype=np.uint8),
        resolution_m=0.2,
        origin_xy=(-5.0, -5.0),
        start_xy=(0.0, 0.0),
        start_yaw_rad=0.0,
        goal_xy=(2.0, 2.0),
        node_budget=20_000,
    )
    result = plan_kinodynamic_path(request, model)
    assert result.status == 'success'
    assert max(
        abs(model.curvature(w.main_command_rad_s, w.auxiliary_angle_rad))
        for w in result.waypoints[:-1]
    ) <= 0.55 + 1e-12
