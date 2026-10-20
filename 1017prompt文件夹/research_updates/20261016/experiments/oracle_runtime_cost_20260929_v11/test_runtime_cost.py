"""Planner edge time must reflect the Oracle's deterministic turn speed tiers."""

import math
from pathlib import Path

from benchmark.drive_model import DriveModel
import time_planning


PROFILE = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json')


def test_turning_edge_uses_the_command_that_runtime_will_request():
    assert hasattr(time_planning, 'scheduled_primitive_time_s')
    model = DriveModel.from_json(PROFILE)
    main = 1.5
    straight = time_planning.scheduled_primitive_time_s(main, 0., .3, model)
    moderate = time_planning.scheduled_primitive_time_s(main, math.radians(-20), .3, model)
    tight = time_planning.scheduled_primitive_time_s(main, math.radians(-30), .3, model)
    assert straight < moderate < tight
    assert math.isclose(straight, .3 / model.speed(main, 0.), rel_tol=1e-9)
    high_curvature = model.curvature(main, math.radians(-30))
    aux_at_half = model.auxiliary_for_curvature(.5, high_curvature)
    assert math.isclose(tight, .3 / model.speed(.5, aux_at_half), rel_tol=1e-9)


def test_reverse_turn_uses_reverse_supported_speed():
    assert hasattr(time_planning, 'scheduled_primitive_time_s')
    model = DriveModel.from_json(PROFILE)
    main = -1.5
    auxiliary = math.radians(30)
    curvature = model.curvature(main, auxiliary)
    capped_main = -.5 if abs(curvature) >= .55 else -1. if abs(curvature) >= .25 else main
    capped_auxiliary = model.auxiliary_for_curvature(capped_main, curvature)
    expected = .3 / model.speed(capped_main, capped_auxiliary)
    assert math.isclose(time_planning.scheduled_primitive_time_s(main, auxiliary, .3, model), expected, rel_tol=1e-9)
