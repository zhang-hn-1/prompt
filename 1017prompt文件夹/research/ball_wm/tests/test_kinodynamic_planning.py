"""Pure-compute tests for bidirectional kinodynamic planning."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from benchmark.drive_model import DriveModel
from benchmark.kinodynamic_planning import (
    KinodynamicPlanRequest,
    _grid_distance_to_goal,
    plan_kinodynamic_path,
    primitive_is_collision_free,
    sample_motion_primitive,
    world_to_cell,
    yaw_to_bin,
)


PROFILE_PATH = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "drive_dynamics_v1.json"
)


def test_world_grid_and_yaw_quantization_are_deterministic():
    assert world_to_cell((1.2, -0.7), origin_xy=(1.0, -1.0), resolution_m=0.1) == (
        2,
        3,
    )
    assert yaw_to_bin(0.0) == 0
    assert yaw_to_bin(math.radians(4.9)) == 1
    assert yaw_to_bin(math.pi) == 36
    assert yaw_to_bin(2.0 * math.pi) == 0


def test_motion_primitive_integrates_forward_reverse_and_curvature():
    forward = sample_motion_primitive(
        start_pose=(0.0, 0.0, 0.0),
        signed_arc_length_m=0.3,
        curvature_1_m=0.0,
        sample_spacing_m=0.05,
    )
    reverse = sample_motion_primitive(
        start_pose=(0.0, 0.0, 0.0),
        signed_arc_length_m=-0.3,
        curvature_1_m=0.0,
        sample_spacing_m=0.05,
    )
    curved = sample_motion_primitive(
        start_pose=(0.0, 0.0, 0.0),
        signed_arc_length_m=0.3,
        curvature_1_m=1.0,
        sample_spacing_m=0.05,
    )

    np.testing.assert_allclose(forward[-1], (0.3, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(reverse[-1], (-0.3, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(
        curved[-1], (math.sin(0.3), 1.0 - math.cos(0.3), 0.3), atol=1e-9
    )
    assert np.max(np.linalg.norm(np.diff(forward[:, :2], axis=0), axis=1)) <= 0.05 + 1e-9


def test_primitive_collision_checks_every_sample_and_map_boundary():
    grid = np.zeros((11, 11), dtype=np.uint8)
    grid[7, 5] = 1
    samples = sample_motion_primitive(
        start_pose=(0.0, 0.0, 0.0),
        signed_arc_length_m=0.3,
        curvature_1_m=0.0,
        sample_spacing_m=0.05,
    )

    assert not primitive_is_collision_free(
        samples,
        occupancy_grid=grid,
        origin_xy=(-0.5, -0.5),
        resolution_m=0.1,
    )
    assert not primitive_is_collision_free(
        samples + np.asarray((1.0, 0.0, 0.0)),
        occupancy_grid=grid,
        origin_xy=(-0.5, -0.5),
        resolution_m=0.1,
    )


def test_holonomic_distance_field_respects_obstacles_and_gap():
    grid = np.zeros((9, 9), dtype=np.uint8)
    grid[4, :] = 1
    grid[4, 8] = 0

    distance = _grid_distance_to_goal(
        grid,
        goal_cell=(7, 4),
        resolution_m=0.1,
        occupied=1,
    )

    assert np.isinf(distance[4, 4])
    assert distance[1, 4] > 0.6
    assert distance[7, 4] == pytest.approx(0.0)


def _request(
    grid: np.ndarray,
    *,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    start_yaw_rad: float = 0.0,
    node_budget: int = 250_000,
    origin_xy: tuple[float, float] = (-4.0, -4.0),
) -> KinodynamicPlanRequest:
    return KinodynamicPlanRequest(
        occupancy_grid=grid,
        resolution_m=0.1,
        origin_xy=origin_xy,
        start_xy=start_xy,
        start_yaw_rad=start_yaw_rad,
        goal_xy=goal_xy,
        node_budget=node_budget,
    )


def test_planner_uses_forward_motion_for_an_open_goal_ahead():
    model = DriveModel.from_json(PROFILE_PATH)
    request = _request(
        np.zeros((81, 81), dtype=np.uint8),
        start_xy=(-3.0, 0.0),
        goal_xy=(-2.1, 0.0),
    )

    result = plan_kinodynamic_path(request, model)

    assert result.status == "success"
    assert result.waypoints[0].drive_direction == "forward"
    assert all(point.main_command_rad_s >= 0.0 for point in result.waypoints)
    assert math.dist(
        (result.waypoints[-1].x_m, result.waypoints[-1].y_m), request.goal_xy
    ) <= 0.2 + 1e-9


def test_planner_can_start_in_reverse_when_goal_is_behind():
    model = DriveModel.from_json(PROFILE_PATH)
    request = _request(
        np.zeros((81, 81), dtype=np.uint8),
        start_xy=(0.0, 0.0),
        goal_xy=(-1.2, 0.0),
    )

    result = plan_kinodynamic_path(request, model)

    assert result.status == "success"
    assert result.waypoints[0].drive_direction == "reverse"
    assert any(point.main_command_rad_s < 0.0 for point in result.waypoints)


def test_planner_detours_around_an_obstacle_without_sampled_collision():
    model = DriveModel.from_json(PROFILE_PATH)
    grid = np.zeros((81, 81), dtype=np.uint8)
    grid[40, 32:49] = 1
    request = _request(grid, start_xy=(-2.0, 0.0), goal_xy=(2.0, 0.0))

    result = plan_kinodynamic_path(request, model)

    assert result.status == "success"
    assert max(abs(point.y_m) for point in result.waypoints) > 0.8
    path = np.asarray(
        [(point.x_m, point.y_m, point.yaw_rad) for point in result.waypoints]
    )
    assert primitive_is_collision_free(
        path,
        occupancy_grid=grid,
        origin_xy=request.origin_xy,
        resolution_m=request.resolution_m,
    )


def test_planner_can_reverse_from_a_blocked_heading_then_continue_forward():
    model = DriveModel.from_json(PROFILE_PATH)
    grid = np.zeros((41, 41), dtype=np.uint8)
    grid[32, 15:26] = 1
    request = KinodynamicPlanRequest(
        occupancy_grid=grid,
        resolution_m=0.1,
        origin_xy=(-2.0, -2.0),
        start_xy=(1.0, 0.0),
        start_yaw_rad=0.0,
        goal_xy=(1.5, 1.5),
    )

    result = plan_kinodynamic_path(request, model)

    directions = [point.drive_direction for point in result.waypoints[:-1]]
    assert result.status == "success"
    assert result.direction_changes == 1
    assert directions[:2] == ["reverse", "reverse"]
    assert directions[2:] == ["forward"] * 6


def test_planner_reports_invalid_unreachable_and_budget_exhaustion():
    model = DriveModel.from_json(PROFILE_PATH)
    occupied = np.zeros((81, 81), dtype=np.uint8)
    occupied[40, 40] = 1
    invalid = plan_kinodynamic_path(
        _request(occupied, start_xy=(0.0, 0.0), goal_xy=(1.0, 0.0)), model
    )
    assert invalid.status == "invalid_start"

    split = np.zeros((9, 9), dtype=np.uint8)
    split[4, :] = 1
    unreachable = plan_kinodynamic_path(
        _request(
            split,
            start_xy=(-0.3, 0.0),
            goal_xy=(0.3, 0.0),
            origin_xy=(-0.4, -0.4),
        ),
        model,
    )
    assert unreachable.status == "unreachable"

    exhausted = plan_kinodynamic_path(
        _request(
            np.zeros((81, 81), dtype=np.uint8),
            start_xy=(-3.0, 0.0),
            goal_xy=(3.0, 0.0),
            node_budget=1,
        ),
        model,
    )
    assert exhausted.status == "node_budget_exhausted"
    assert exhausted.expanded_nodes == 1


def test_planner_precomputes_measured_primitives_once(monkeypatch):
    model = DriveModel.from_json(PROFILE_PATH)
    original = DriveModel.curvature
    calls = 0

    def counted(self, main_command_rad_s, auxiliary_angle_rad):
        nonlocal calls
        calls += 1
        return original(self, main_command_rad_s, auxiliary_angle_rad)

    monkeypatch.setattr(DriveModel, "curvature", counted)
    result = plan_kinodynamic_path(
        _request(
            np.zeros((81, 81), dtype=np.uint8),
            start_xy=(-3.0, 0.0),
            goal_xy=(3.0, 0.0),
            node_budget=2,
        ),
        model,
    )

    assert result.status == "node_budget_exhausted"
    assert calls == 2 * 3 * 7


def test_planner_is_deterministic_for_identical_requests():
    model = DriveModel.from_json(PROFILE_PATH)
    request = _request(
        np.zeros((81, 81), dtype=np.uint8),
        start_xy=(-2.0, -0.5),
        goal_xy=(1.0, 1.0),
        start_yaw_rad=math.radians(20.0),
    )

    first = plan_kinodynamic_path(request, model)
    second = plan_kinodynamic_path(request, model)

    assert first == second
