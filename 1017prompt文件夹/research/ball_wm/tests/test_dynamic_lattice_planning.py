"""Tests for continuous, rate-limited dynamic motion primitives."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import benchmark.dynamic_lattice_planning as dynamic

from benchmark.drive_model import DriveModel
from benchmark.dynamic_lattice_planning import (
    DynamicControlTarget,
    DynamicPrimitive,
    DynamicState,
    dynamic_primitive_is_collision_free,
    propagate_dynamic_primitive,
    valid_control_targets,
)
from benchmark.grid_planning import astar_path, inflate_obstacles
from benchmark.task_cards import load_occupancy_npz


PROFILE_PATH = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "benchmark_v1"
    / "drive_dynamics_v1.json"
)
ROOT = Path(__file__).resolve().parents[1]
HARD_0_MANIFEST_PATH = ROOT / "configs" / "benchmark_v1" / "hard_0_dev_manifest.json"
HARD_0_OCCUPANCY_PATH = (
    ROOT / "data" / "benchmark_v1" / "occupancy" / "cluttered_hard_0_bodyband.npz"
)
HARD_0_OCCUPANCY_SHA256 = (
    "547dd73c52b45f9cc1aa8dca561c779349391f6ecef06ef59026921284b467da"
)


def test_primitive_rate_limits_controls_at_50_hz() -> None:
    model = DriveModel.from_json(PROFILE_PATH)

    primitive = propagate_dynamic_primitive(
        start=DynamicState(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.5
        ),
        target=DynamicControlTarget(1.5, math.radians(30.0)),
        drive_model=model,
    )

    assert primitive.end.main_command_rad_s == pytest.approx(0.5)
    assert primitive.end.auxiliary_angle_rad == pytest.approx(math.radians(30.0))
    assert primitive.samples.shape == (51, 8)
    assert np.isfinite(primitive.samples).all()
    np.testing.assert_allclose(np.diff(primitive.samples[:, 0]), 0.02, atol=1e-12)


def test_control_targets_require_a_half_second_stop_before_reversing() -> None:
    moving_forward = DynamicState(
        0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0, "forward", 0.0
    )
    just_stopped = DynamicState(
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, "stopped", 0.49
    )
    reversal_ready = DynamicState(
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.01, "stopped", 0.5
    )

    assert all(
        target.main_command_rad_s >= 0.0
        for target in valid_control_targets(moving_forward)
    )
    assert {
        target.main_command_rad_s for target in valid_control_targets(just_stopped)
    } == {0.0}

    ready_targets = valid_control_targets(reversal_ready)
    assert tuple(ready_targets[::7]) == tuple(
        DynamicControlTarget(main, 0.0)
        for main in (1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5)
    )


def test_braking_then_dwelling_unlocks_reverse_targets() -> None:
    model = DriveModel.from_json(PROFILE_PATH)
    moving_forward = DynamicState(
        0.0, 0.0, 0.0, 0.5, 0.0, model.speed(0.5, 0.0), 0.0, "forward", 0.0
    )

    braking = propagate_dynamic_primitive(
        start=moving_forward,
        target=DynamicControlTarget(0.0, 0.0),
        drive_model=model,
    )
    dwelling = propagate_dynamic_primitive(
        start=braking.end,
        target=DynamicControlTarget(0.0, 0.0),
        drive_model=model,
    )

    assert braking.end.drive_direction == "stopped"
    assert braking.end.stopped_time_s == pytest.approx(0.02)
    assert not any(
        target.main_command_rad_s < 0.0
        for target in valid_control_targets(braking.end)
    )
    assert dwelling.end.stopped_time_s == pytest.approx(1.02)
    assert any(
        target.main_command_rad_s < 0.0
        for target in valid_control_targets(dwelling.end)
    )


def test_zero_main_command_uses_zero_speed_without_querying_curvature() -> None:
    class ZeroMainGuard:
        def speed(self, main_command_rad_s: float, auxiliary_angle_rad: float) -> float:
            return 999.0

        def curvature(
            self, main_command_rad_s: float, auxiliary_angle_rad: float
        ) -> float:
            raise AssertionError("curvature requires a non-zero main command")

    primitive = propagate_dynamic_primitive(
        start=DynamicState(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, "stopped", 0.5
        ),
        target=DynamicControlTarget(0.0, math.radians(30.0)),
        drive_model=ZeroMainGuard(),  # type: ignore[arg-type]
    )

    assert primitive.end.speed_m_s == 0.0
    assert primitive.end.stopped_time_s == pytest.approx(1.5)
    np.testing.assert_allclose(primitive.samples[:, 1:3], 0.0)


def test_dynamic_collision_check_rejects_an_occupied_middle_sample() -> None:
    samples = np.asarray(
        [
            (0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0),
            (0.5, 1.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0),
            (1.0, 2.0, 0.0, 0.0, 0.5, 0.0, 0.2, 0.0),
        ],
        dtype=np.float64,
    )
    primitive = DynamicPrimitive(
        target=DynamicControlTarget(0.5, 0.0),
        samples=samples,
        end=DynamicState(2.0, 0.0, 0.0, 0.5, 0.0, 0.2, 1.0, "forward", 0.0),
        path_length_m=2.0,
    )
    grid = np.zeros((3, 3), dtype=np.uint8)
    grid[1, 0] = 1

    assert not dynamic_primitive_is_collision_free(
        primitive,
        occupancy_grid=grid,
        origin_xy=(0.0, 0.0),
        resolution_m=1.0,
    )


def test_state_key_keeps_each_dynamic_dimension_and_braking_speed() -> None:
    state = DynamicState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.5)
    key = dynamic.dynamic_state_key(state, origin_xy=(0.0, 0.0), resolution_m=0.1)
    assert key == (0, 0, 0, 0, 0, 0, "stopped")
    for field, value, index in (
        ("x_m", 0.1, 0), ("y_m", 0.1, 1), ("yaw_rad", math.radians(5), 2),
        ("main_command_rad_s", 0.25, 3),
        ("auxiliary_angle_rad", math.radians(5), 4), ("speed_m_s", 0.05, 5),
    ):
        changed = dynamic.dynamic_state_key(
            replace(state, **{field: value}), origin_xy=(0.0, 0.0), resolution_m=0.1
        )
        assert changed[index] == 1
        assert changed[:index] + changed[index + 1:] == key[:index] + key[index + 1:]
    assert dynamic.dynamic_state_key(
        replace(state, yaw_rad=2 * math.pi), origin_xy=(0.0, 0.0), resolution_m=0.1
    ) == key


def test_risk_integral_prefers_clearance_for_equal_length_primitives() -> None:
    model = DriveModel.from_json(PROFILE_PATH)
    grid = np.zeros((40, 40), dtype=np.uint8)
    grid[:, 0] = 1
    field = dynamic.obstacle_distance_field(grid, resolution_m=0.1)
    assert field[10, 1] == pytest.approx(0.05)
    costs = []
    for y in (0.1, 1.0):
        start = DynamicState(1.0, y, 0.0, 0.5, 0.0, model.speed(0.5, 0), 0, "forward", 0)
        primitive = propagate_dynamic_primitive(
            start=start, target=DynamicControlTarget(0.5, 0.0), drive_model=model
        )
        costs.append(dynamic.dynamic_primitive_cost(
            primitive, previous_target=primitive.target, previous_direction="forward",
            distance_field=field, origin_xy=(0.0, 0.0), resolution_m=0.1,
        ))
    assert costs[1] < costs[0]
    assert costs[0] == pytest.approx(primitive.path_length_m + 0.05 + 0.2)


def _request(**overrides):
    values = dict(
        occupancy_grid=np.zeros((31, 31), dtype=np.uint8), resolution_m=0.1,
        origin_xy=(0.0, 0.0),
        initial_state=DynamicState(0.5, 1.5, 0.0, 0.0, 0.0, 0.0, 7.0, "stopped", 0.5),
        goal_xy=(1.0, 1.5),
    )
    values.update(overrides)
    return dynamic.DynamicPlanRequest(**values)


def _model() -> DriveModel:
    return DriveModel.from_json(PROFILE_PATH)


def _hard_0_e1_0002_request() -> dynamic.DynamicPlanRequest:
    payload = json.loads(HARD_0_MANIFEST_PATH.read_text(encoding="utf-8"))
    episode = next(
        item for item in payload["episodes"]
        if item["episode_id"] == "hard_0_e1_0002"
    )
    actual_sha256 = hashlib.sha256(HARD_0_OCCUPANCY_PATH.read_bytes()).hexdigest()
    assert episode["occupancy_sha256"] == HARD_0_OCCUPANCY_SHA256
    assert actual_sha256 == HARD_0_OCCUPANCY_SHA256

    occupancy = load_occupancy_npz(HARD_0_OCCUPANCY_PATH)
    planning_grid = np.where(occupancy.grid == 0, 0, 1).astype(np.uint8)
    inflated_grid = inflate_obstacles(
        planning_grid,
        radius_m=0.5,
        resolution_m=occupancy.resolution_m,
        occupied=1,
    )
    start_xy = tuple(float(value) for value in episode["start_position_world_m"][:2])
    goal_xy = tuple(float(value) for value in episode["goal_position_world_m"][:2])
    quaternion = episode["start_quaternion_world_wxyz"]
    yaw_rad = math.atan2(
        2.0 * (quaternion[0] * quaternion[3]),
        1.0 - 2.0 * quaternion[3] ** 2,
    )
    assert start_xy == pytest.approx((-9.85, 4.05))
    assert goal_xy == pytest.approx((-7.75, -4.15))
    assert yaw_rad == pytest.approx(-1.3200870673734872)

    start_cell = dynamic.world_to_cell(
        start_xy,
        origin_xy=occupancy.global_origin_xy,
        resolution_m=occupancy.resolution_m,
    )
    goal_cell = dynamic.world_to_cell(
        goal_xy,
        origin_xy=occupancy.global_origin_xy,
        resolution_m=occupancy.resolution_m,
    )
    guidance_cells = astar_path(
        inflated_grid,
        start=start_cell,
        goal=goal_cell,
        occupied=1,
    )
    return dynamic.DynamicPlanRequest(
        occupancy_grid=inflated_grid,
        resolution_m=occupancy.resolution_m,
        origin_xy=tuple(float(value) for value in occupancy.global_origin_xy),
        initial_state=DynamicState(
            start_xy[0], start_xy[1], yaw_rad, 0.0, 0.0, 0.0,
            0.0, "stopped", 0.5,
        ),
        goal_xy=goal_xy,
        guidance_cells=guidance_cells,
    )


def test_reference_rollout_solves_hash_verified_hard_0_e1_0002_static_input() -> None:
    request = _hard_0_e1_0002_request()

    first = dynamic.plan_dynamic_lattice(request, _model())
    second = dynamic.plan_dynamic_lattice(request, _model())

    assert first.status == second.status == "success"
    assert first.controls == second.controls
    assert first.waypoints == second.waypoints
    assert 0.0 < first.duration_s <= 32.0
    assert first.controls
    assert first.waypoints
    waypoint_values = np.asarray(
        [
            (
                waypoint.x_m,
                waypoint.y_m,
                waypoint.yaw_rad,
                waypoint.main_command_rad_s,
                waypoint.auxiliary_angle_rad,
                waypoint.speed_m_s,
                waypoint.time_from_start_s,
            )
            for waypoint in first.waypoints
        ],
        dtype=np.float64,
    )
    assert np.isfinite(waypoint_values).all()
    cells = dynamic._trusted_edge_position_cells(
        waypoint_values[:, :2],
        origin_xy=request.origin_xy,
        resolution_m=request.resolution_m,
    )
    assert np.all(cells[:, 0] >= 0)
    assert np.all(cells[:, 0] < request.occupancy_grid.shape[0])
    assert np.all(cells[:, 1] >= 0)
    assert np.all(cells[:, 1] < request.occupancy_grid.shape[1])
    assert not np.any(
        request.occupancy_grid[cells[:, 0], cells[:, 1]] == request.occupied
    )


def test_reference_rollout_retains_each_available_drive_direction_per_layer() -> None:
    request = _hard_0_e1_0002_request()
    model = _model()
    clearance = dynamic.obstacle_distance_field(
        request.occupancy_grid,
        resolution_m=request.resolution_m,
        occupied=request.occupied,
    )

    rollout = dynamic._build_reference_rollout(request, model, clearance)
    first_layer = [
        rollout.nodes[index]
        for index in rollout.seed_indices
        if rollout.nodes[index].state.elapsed_s
        == pytest.approx(request.initial_state.elapsed_s + 1.0)
    ]
    available_directions = {
        successor.end.drive_direction
        for parent in first_layer
        for successor in dynamic._iter_valid_successors(
            parent,
            request=request,
            drive_model=model,
            clearance=clearance,
            templates={},
        )
    }
    retained_layer = [
        rollout.nodes[index]
        for index in rollout.seed_indices
        if rollout.nodes[index].state.elapsed_s
        == pytest.approx(request.initial_state.elapsed_s + 2.0)
    ]

    assert available_directions == {"forward", "reverse", "stopped"}
    assert len(retained_layer) <= 8
    assert {node.state.drive_direction for node in retained_layer} == available_directions


def test_reference_rollout_does_not_let_unselected_history_prune_later_candidate(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A controlled successor graph isolates beam retention, not dynamics."""
    grid = np.zeros((60, 2), dtype=np.uint8)
    guidance_cells = np.asarray(tuple((row, 0) for row in range(51)), dtype=np.int64)
    request = dynamic.DynamicPlanRequest(
        occupancy_grid=grid,
        resolution_m=1.0,
        origin_xy=(0.0, 0.0),
        initial_state=DynamicState(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.5
        ),
        goal_xy=(50.0, 0.0),
        guidance_cells=guidance_cells,
    )
    target = DynamicControlTarget(0.5, 0.0)

    # Replacing only the successor iterator makes this a controlled search
    # graph; it tests beam retention semantics, not physical feasibility.
    def fake_successors(node, *, request, drive_model, clearance, templates, deadline_check=None):
        del request, drive_model, clearance, templates, deadline_check
        if node.state.elapsed_s == 0.0:
            endpoint_rows = range(1, 10)
        elif node.state.elapsed_s == 1.0 and node.state.x_m == pytest.approx(2.0):
            endpoint_rows = (1,)
        else:
            endpoint_rows = ()
        for endpoint_row in endpoint_rows:
            end = DynamicState(
                float(endpoint_row),
                0.0,
                0.0,
                0.5,
                0.0,
                0.1,
                node.state.elapsed_s + 1.0,
                "forward",
                0.0,
            )
            samples = np.asarray(
                (
                    (node.state.elapsed_s, node.state.x_m, 0.0, 0.0, 0.5, 0.0, 0.1, 0.0),
                    (end.elapsed_s, end.x_m, end.y_m, end.yaw_rad, 0.5, 0.0, 0.1, 0.0),
                ),
                dtype=np.float64,
            )
            primitive = DynamicPrimitive(target, samples, end, 0.1)
            yield dynamic._DynamicSuccessor(end, primitive, "forward", 0.1)

    monkeypatch.setattr(dynamic, "_iter_valid_successors", fake_successors)

    rollout = dynamic._build_reference_rollout(
        request,
        _model(),
        np.full(grid.shape, np.inf),
    )

    retained_by_layer: dict[float, list[dynamic._SearchNode]] = {}
    for index in rollout.seed_indices:
        retained_by_layer.setdefault(rollout.nodes[index].state.elapsed_s, []).append(
            rollout.nodes[index]
        )
    assert max(len(nodes) for nodes in retained_by_layer.values()) <= 8
    assert all(
        node.state.x_m != pytest.approx(1.0)
        for node in retained_by_layer[1.0]
    )
    assert any(
        rollout.nodes[index].state.elapsed_s == pytest.approx(2.0)
        and rollout.nodes[index].state.x_m == pytest.approx(1.0)
        for index in rollout.seed_indices
    )


def test_planner_translates_reference_rollout_deadline_to_timeout(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    request = _request(
        guidance_cells=np.asarray(
            ((5, 15), (6, 15), (7, 15), (8, 15), (9, 15), (10, 15)),
            dtype=np.int64,
        )
    )

    def deadline(*args, **kwargs):
        raise dynamic._PlanningDeadlineExceeded

    monkeypatch.setattr(dynamic, "_build_reference_rollout", deadline)

    result = dynamic.plan_dynamic_lattice(request, _model())

    assert result == dynamic.DynamicPlanResult("timeout", expanded_nodes=0)


def _detour_reference_cells() -> np.ndarray:
    return np.asarray(
        (
            (5, 15),
            (5, 14),
            (5, 13),
            (5, 12),
            (6, 11),
            (7, 10),
            (8, 11),
            (9, 12),
            (10, 13),
            (10, 14),
            (10, 15),
        ),
        dtype=np.int64,
    )


def _tight_reference_cells() -> np.ndarray:
    return np.asarray(
        (
            *((5, column) for column in range(15, 9, -1)),
            *((row, 10) for row in range(6, 11)),
            *((10, column) for column in range(11, 16)),
        ),
        dtype=np.int64,
    )


def _turning_reference_request(*, occupancy_grid: np.ndarray | None = None):
    grid = (
        np.zeros((61, 61), dtype=np.uint8)
        if occupancy_grid is None
        else occupancy_grid
    )
    guidance_cells = np.asarray(
        tuple((20 + offset, 20 + offset) for offset in range(16)),
        dtype=np.int64,
    )
    return _request(
        occupancy_grid=grid,
        initial_state=DynamicState(
            2.0, 2.0, math.pi / 2.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.5
        ),
        goal_xy=(3.5, 3.5),
        guidance_cells=guidance_cells,
    )


def _rollout_signature(rollout) -> tuple:
    return (
        rollout.seed_indices,
        rollout.goal_index,
        rollout.expanded_nodes,
        tuple(
            (
                node.state,
                node.cost,
                node.parent,
                node.primitive.target if node.primitive is not None else None,
                (
                    node.primitive.samples.shape,
                    node.primitive.samples.dtype.str,
                    node.primitive.samples.tobytes(),
                    node.primitive.path_length_m,
                )
                if node.primitive is not None
                else None,
                node.last_direction,
                node.previous_target,
                node.reference_index,
                node.source,
            )
            for node in rollout.nodes
        ),
    )


def test_reference_rollout_reaches_stopped_goal_from_misaligned_yaw() -> None:
    request = _turning_reference_request()
    model = _model()
    clearance = dynamic.obstacle_distance_field(
        request.occupancy_grid, resolution_m=request.resolution_m
    )

    rollout = dynamic._build_reference_rollout(request, model, clearance)

    assert rollout.goal_index is not None
    result = dynamic._reconstruct_dynamic(
        list(rollout.nodes), rollout.goal_index, rollout.expanded_nodes
    )
    assert result.duration_s <= 32.0
    assert result.waypoints[-1].drive_direction == "stopped"
    assert result.waypoints[-1].speed_m_s < 0.05
    assert abs(result.waypoints[-1].main_command_rad_s) <= 0.05


def test_reference_rollout_propagates_deadline_during_beam_expansion() -> None:
    request = _turning_reference_request()
    model = _model()
    clearance = dynamic.obstacle_distance_field(
        request.occupancy_grid, resolution_m=request.resolution_m
    )

    with pytest.raises(dynamic._PlanningDeadlineExceeded):
        dynamic._build_reference_rollout(
            request, model, clearance, deadline_check=lambda: True
        )


def test_reference_rollout_rejects_colliding_high_progress_primitive(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    grid = np.zeros((61, 61), dtype=np.uint8)
    # Every forward primitive that makes first-layer reference progress enters
    # this free-but-off-reference cell; stopping candidates do not.
    grid[20, 21] = 1
    request = _turning_reference_request(occupancy_grid=grid)
    model = _model()
    clearance = dynamic.obstacle_distance_field(
        request.occupancy_grid, resolution_m=request.resolution_m
    )
    collision_results: list[bool] = []
    original_check = dynamic._trusted_edge_is_collision_free

    def recording_check(*args, **kwargs):
        result = original_check(*args, **kwargs)
        collision_results.append(result)
        return result

    monkeypatch.setattr(dynamic, "_trusted_edge_is_collision_free", recording_check)

    rollout = dynamic._build_reference_rollout(request, model, clearance)

    assert False in collision_results
    retained = [node.primitive for node in rollout.nodes if node.primitive is not None]
    assert retained
    assert all(
        dynamic_primitive_is_collision_free(
            primitive,
            occupancy_grid=request.occupancy_grid,
            origin_xy=request.origin_xy,
            resolution_m=request.resolution_m,
            occupied=request.occupied,
        )
        for primitive in retained
    )


def test_reference_rollout_is_deterministic_and_has_safe_off_reference_seed() -> None:
    request = _turning_reference_request()
    model = _model()
    clearance = dynamic.obstacle_distance_field(
        request.occupancy_grid, resolution_m=request.resolution_m
    )

    first = dynamic._build_reference_rollout(request, model, clearance)
    second = dynamic._build_reference_rollout(request, model, clearance)

    assert _rollout_signature(first) == _rollout_signature(second)
    assert first.goal_index is not None
    assert dynamic._reconstruct_dynamic(
        list(first.nodes), first.goal_index, first.expanded_nodes
    ) == dynamic._reconstruct_dynamic(
        list(second.nodes), second.goal_index, second.expanded_nodes
    )
    assert first.seed_indices
    retained_per_layer: dict[float, int] = {}
    for index in first.seed_indices:
        elapsed_s = first.nodes[index].state.elapsed_s
        retained_per_layer[elapsed_s] = retained_per_layer.get(elapsed_s, 0) + 1
    assert max(retained_per_layer.values()) <= 8
    assert any(
        first.nodes[index].source == "reference_rollout"
        and _distance_from_reference(
            dynamic.DynamicWaypoint(
                first.nodes[index].state.x_m,
                first.nodes[index].state.y_m,
                first.nodes[index].state.yaw_rad,
                first.nodes[index].state.main_command_rad_s,
                first.nodes[index].state.auxiliary_angle_rad,
                first.nodes[index].state.speed_m_s,
                first.nodes[index].state.elapsed_s,
                first.nodes[index].state.drive_direction,
            ),
            request.guidance_cells,
        )
        > 0.05
        for index in first.seed_indices
    )


def test_reference_projection_uses_nearest_cell_in_nonretreating_suffix() -> None:
    reference_xy = np.asarray(
        ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0),
         (0.0, 1.0), (0.0, 0.1)),
        dtype=np.float64,
    )
    state = DynamicState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.5)

    index, lateral_m = dynamic._nearest_reference_suffix(
        state, reference_xy, parent_progress=3
    )

    assert index == 6
    assert lateral_m == pytest.approx(0.1)


def _distance_from_reference(
    waypoint: dynamic.DynamicWaypoint, reference_cells: np.ndarray
) -> float:
    reference_xy = reference_cells.astype(np.float64) * 0.1
    return float(
        np.min(
            np.hypot(
                reference_xy[:, 0] - waypoint.x_m,
                reference_xy[:, 1] - waypoint.y_m,
            )
        )
    )


def test_request_preserves_legacy_optional_positional_arguments() -> None:
    state = DynamicState(
        0.5, 1.5, 0.0, 0.0, 0.0, 0.0, 7.0, "stopped", 0.5
    )

    request = dynamic.DynamicPlanRequest(
        np.zeros((31, 31), dtype=np.uint8),
        0.1,
        (0.0, 0.0),
        state,
        (1.0, 1.5),
        123,
        4.0,
        7,
    )

    assert request.node_budget == 123
    assert request.planning_timeout_s == 4.0
    assert request.occupied == 7
    assert request.guidance_cells is None
    assert request.guidance_weight == 0.35


def test_guidance_cells_are_frozen_free_integer_path_ending_at_goal() -> None:
    request = _request(guidance_cells=np.asarray(
        ((5, 15), (6, 15), (7, 15), (8, 15), (9, 15), (10, 15)),
        dtype=np.int64,
    ))

    assert request.guidance_cells is not None
    assert not request.guidance_cells.flags.writeable
    np.testing.assert_array_equal(
        request.guidance_cells,
        np.asarray(
            ((5, 15), (6, 15), (7, 15), (8, 15), (9, 15), (10, 15)),
            dtype=np.int64,
        ),
    )
    with pytest.raises(ValueError):
        request.guidance_cells[0, 0] = 0


def test_guidance_cells_are_a_defensive_copy_of_caller_input() -> None:
    guidance_cells = np.asarray(
        ((5, 15), (6, 15), (7, 15), (8, 15), (9, 15), (10, 15)),
        dtype=np.int64,
    )
    request = _request(guidance_cells=guidance_cells)

    guidance_cells[0, 0] = 4

    assert request.guidance_cells is not None
    assert request.guidance_cells[0, 0] == 5


@pytest.mark.parametrize("bad", [
    np.asarray((1, 2), dtype=np.int64),
    np.asarray(((1.5, 1.0),), dtype=np.float64),
    np.asarray(((99, 99),), dtype=np.int64),
])
def test_guidance_cells_reject_invalid_arrays(bad: np.ndarray) -> None:
    with pytest.raises(ValueError):
        _request(guidance_cells=bad)


def test_guidance_cells_reject_occupied_and_wrong_goal_cells() -> None:
    grid = np.zeros((31, 31), dtype=np.uint8)
    grid[6, 15] = 1

    with pytest.raises(ValueError):
        _request(
            occupancy_grid=grid,
            guidance_cells=np.asarray(((5, 15), (6, 15), (10, 15)), dtype=np.int64),
        )
    with pytest.raises(ValueError):
        _request(guidance_cells=np.asarray(((5, 15), (9, 15)), dtype=np.int64))


@pytest.mark.parametrize("guidance_weight", [0.0, -0.1, np.inf, np.nan])
def test_guidance_weight_must_be_finite_and_positive(guidance_weight: float) -> None:
    with pytest.raises(ValueError):
        _request(guidance_weight=guidance_weight)


def test_guided_remaining_field_uses_nearest_path_cell_and_8_neighbor_lengths() -> None:
    field = dynamic._guided_remaining_field(
        np.asarray(((0, 0), (0, 1), (1, 2)), dtype=np.int64),
        grid_shape=(3, 3),
        resolution_m=2.0,
    )

    assert field.shape == (3, 3)
    assert field[0, 0] == pytest.approx(2.0 + 2.0 * math.sqrt(2.0))
    assert field[0, 1] == pytest.approx(2.0 * math.sqrt(2.0))
    assert field[1, 2] == 0.0
    assert field[2, 2] == 0.0


def test_reference_progress_fields_are_metric_without_creating_a_corridor() -> None:
    request = _request(
        goal_xy=(0.2, 0.3),
        guidance_cells=np.asarray(((1, 1), (1, 2), (2, 3)), dtype=np.int64),
    )

    fields = dynamic._reference_progress_fields(request)

    assert fields is not None
    remaining, lateral = fields
    assert remaining[2, 3] == pytest.approx(0.0)
    assert remaining[4, 4] > 0.0
    assert math.isfinite(float(remaining[4, 4]))
    assert lateral[4, 4] > 0.0
    assert lateral[2, 3] == pytest.approx(0.0)


def test_shared_successor_foundation_keeps_no_guidance_result_deterministic() -> None:
    request = _request(planning_timeout_s=5.0)

    first = dynamic.plan_dynamic_lattice(request, _model())
    second = dynamic.plan_dynamic_lattice(request, _model())

    assert first == second


def test_reference_progress_fields_are_absent_without_guidance() -> None:
    assert dynamic._reference_progress_fields(_request()) is None


def test_planner_preserves_timeout_after_first_colliding_propagation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    grid = np.zeros((31, 31), dtype=np.uint8)
    request = _request(
        occupancy_grid=grid,
        initial_state=DynamicState(
            1.5, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.5
        ),
        goal_xy=(2.5, 1.5),
        planning_timeout_s=1.0,
    )
    clock = [0.0]
    original_propagate = dynamic.propagate_dynamic_primitive

    def propagate_without_expiring(*, start, target, drive_model):
        primitive = original_propagate(start=start, target=target, drive_model=drive_model)
        return primitive

    collision_calls: list[int] = []

    def collision_then_expire(*args, **kwargs):
        collision_calls.append(1)
        clock[0] = 2.0
        return False

    monkeypatch.setattr(dynamic.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(dynamic, "propagate_dynamic_primitive", propagate_without_expiring)
    monkeypatch.setattr(dynamic, "_trusted_edge_is_collision_free", collision_then_expire)

    result = dynamic.plan_dynamic_lattice(request, _model())

    assert result.status == "timeout"
    assert collision_calls


@pytest.mark.parametrize("overrides", [
    {"occupancy_grid": np.zeros((0, 3))}, {"occupancy_grid": [[np.nan]]},
    {"resolution_m": 0}, {"origin_xy": (0, np.inf)}, {"goal_xy": (np.nan, 0)},
    {"node_budget": True}, {"node_budget": 0}, {"node_budget": 1.5},
    {"planning_timeout_s": 0}, {"planning_timeout_s": np.inf},
    {"initial_state": DynamicState(0, 0, 0, 0, 0, -1, 0, "stopped", 0.5)},
    {"initial_state": DynamicState(0, 0, np.nan, 0, 0, 0, 0, "stopped", 0.5)},
])
def test_request_rejects_invalid_inputs(overrides) -> None:
    with pytest.raises(ValueError):
        _request(**overrides)


def test_dynamic_search_stops_at_goal_and_is_deterministic() -> None:
    request = _request()
    model = DriveModel.from_json(PROFILE_PATH)
    first = dynamic.plan_dynamic_lattice(request, model)
    second = dynamic.plan_dynamic_lattice(request, model)
    assert first.status == second.status == "success"
    assert first.waypoints == second.waypoints
    assert first.controls == second.controls
    assert first.waypoints[0].time_from_start_s == 0.0
    assert first.waypoints[0].x_m == 0.5
    assert first.waypoints[-1].speed_m_s < 0.05
    assert abs(first.waypoints[-1].main_command_rad_s) <= 0.05
    assert math.hypot(first.waypoints[-1].x_m - 1, first.waypoints[-1].y_m - 1.5) <= 0.2
    assert first.duration_s <= 32.0
    assert all(b.time_from_start_s > a.time_from_start_s for a, b in zip(first.waypoints, first.waypoints[1:]))
    assert first.waypoints[-1].time_from_start_s == first.duration_s


def test_guidance_is_soft_and_keeps_a_feasible_wider_dynamic_detour() -> None:
    reference_cells = _tight_reference_cells()

    result = dynamic.plan_dynamic_lattice(
        _request(guidance_cells=reference_cells), _model()
    )

    assert result.status == "success"
    assert any(
        _distance_from_reference(waypoint, reference_cells) > 0.1
        for waypoint in result.waypoints
    )


def test_guided_search_is_deterministic() -> None:
    request = _request(guidance_cells=_detour_reference_cells())

    assert dynamic.plan_dynamic_lattice(request, _model()) == dynamic.plan_dynamic_lattice(
        request, _model()
    )


def test_dynamic_search_detours_around_obstacle() -> None:
    grid = np.zeros((31, 31), dtype=np.uint8)
    grid[10:12, 15] = 1
    request = _request(occupancy_grid=grid, goal_xy=(1.7, 1.5))
    result = dynamic.plan_dynamic_lattice(request, DriveModel.from_json(PROFILE_PATH))
    assert result.status == "success"
    assert result.duration_s <= 32.0
    assert max(abs(w.y_m - 1.5) for w in result.waypoints) > 0.05
    assert all(grid[round(w.x_m / 0.1), round(w.y_m / 0.1)] == 0 for w in result.waypoints)


@pytest.mark.parametrize("status", ["invalid_start", "invalid_goal", "unreachable", "node_budget_exhausted", "timeout"])
def test_dynamic_search_failure_statuses(status, monkeypatch) -> None:
    grid = np.zeros((31, 31), dtype=np.uint8)
    overrides = {"occupancy_grid": grid}
    if status == "invalid_start":
        grid[5, 15] = 1
    elif status == "invalid_goal":
        grid[10, 15] = 1
    elif status == "unreachable":
        grid[7, :] = 1
    elif status == "node_budget_exhausted":
        overrides["node_budget"] = 1
    else:
        times = iter((0.0, 61.0))
        monkeypatch.setattr(dynamic.time, "monotonic", lambda: next(times, 61.0))
    result = dynamic.plan_dynamic_lattice(_request(**overrides), DriveModel.from_json(PROFILE_PATH))
    assert result.status == status
    assert result.waypoints == ()


def test_timeout_includes_obstacle_heuristic_precomputation(monkeypatch) -> None:
    grid = np.zeros((31, 31), dtype=np.uint8)
    grid[7, :] = 1
    times = iter((0.0, 0.0, 61.0))
    monkeypatch.setattr(
        dynamic.time, "monotonic", lambda: next(times, 61.0)
    )

    result = dynamic.plan_dynamic_lattice(
        _request(occupancy_grid=grid), DriveModel.from_json(PROFILE_PATH)
    )

    assert result.status == "timeout"
    assert result.expanded_nodes == 0


def test_dynamic_search_preserves_stop_dwell_to_allow_reverse() -> None:
    request = _request(goal_xy=(0.0, 1.5), initial_state=DynamicState(
        0.5, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, "stopped", 0.0
    ))
    result = dynamic.plan_dynamic_lattice(request, DriveModel.from_json(PROFILE_PATH))
    assert result.status == "success"
    assert result.controls[0].main_command_rad_s == 0.0
    assert any(control.main_command_rad_s < 0 for control in result.controls)


def test_already_stopped_within_goal_tolerance_needs_no_route_to_goal_cell() -> None:
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[1, :] = 1
    result = dynamic.plan_dynamic_lattice(_request(
        occupancy_grid=grid, goal_xy=(0.2, 0.2),
        initial_state=DynamicState(0, 0.2, 0, 0, 0, 0, 0, "stopped", 0.5),
    ), DriveModel.from_json(PROFILE_PATH))
    assert result.status == "success"
    assert result.duration_s == 0
    assert len(result.waypoints) == 1


def test_braking_within_goal_tolerance_needs_no_route_to_goal_cell() -> None:
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[1, :] = 1
    model = DriveModel.from_json(PROFILE_PATH)
    result = dynamic.plan_dynamic_lattice(_request(
        occupancy_grid=grid, goal_xy=(0.2, 0.2),
        initial_state=DynamicState(
            0.0, 0.2, 0.0, 0.1, 0.0, model.speed(0.1, 0.0),
            0.0, "forward", 0.0,
        ),
    ), model)
    assert result.status == "success"
    assert result.duration_s == 1.0
    assert result.waypoints[-1].speed_m_s == 0.0


def test_full_cost_accounts_for_reverse_switch_and_control_changes() -> None:
    model = DriveModel.from_json(PROFILE_PATH)
    state = DynamicState(1, 1, 0, -0.5, 0, model.speed(-0.5, 0), 0, "reverse", 0)
    primitive = propagate_dynamic_primitive(start=state, target=DynamicControlTarget(-0.5, 0), drive_model=model)
    cost = dynamic.dynamic_primitive_cost(
        primitive, previous_target=DynamicControlTarget(0, math.radians(30)),
        previous_direction="forward", distance_field=np.full((30, 30), np.inf),
        origin_xy=(0, 0), resolution_m=0.1,
    )
    assert cost == pytest.approx(1.25 * primitive.path_length_m + 0.05 + 0.5 + 0.05 + 0.1)


def test_search_controls_replay_continuous_state_without_quantized_snapping() -> None:
    model = DriveModel.from_json(PROFILE_PATH)
    request = _request(initial_state=DynamicState(0.513, 1.503, 0.017, 0, 0, 0, 0, "stopped", 0.5))
    result = dynamic.plan_dynamic_lattice(request, model)
    assert result.status == "success"
    state = request.initial_state
    for index, target in enumerate(result.controls):
        assert target in valid_control_targets(state)
        assert abs(target.main_command_rad_s - round(state.main_command_rad_s / 0.25) * 0.25) <= 0.5 + 1e-12
        assert abs(target.auxiliary_angle_rad - round(state.auxiliary_angle_rad / math.radians(5)) * math.radians(5)) <= math.radians(30) + 1e-12
        primitive = propagate_dynamic_primitive(start=state, target=target, drive_model=model)
        saved = result.waypoints[index * 50:(index + 1) * 50 + 1]
        np.testing.assert_allclose(
            [[w.time_from_start_s, w.x_m, w.y_m, w.yaw_rad, w.main_command_rad_s, w.auxiliary_angle_rad, w.speed_m_s] for w in saved],
            primitive.samples[:, :7], atol=1e-12,
        )
        state = primitive.end


def test_planner_reuses_accepted_edge_cells_without_changing_public_helpers(
    monkeypatch,
) -> None:
    """Accepted edges must not floor their later samples again for risk cost."""
    model = DriveModel.from_json(PROFILE_PATH)
    primitive = propagate_dynamic_primitive(
        start=DynamicState(0.5, 1.5, 0.0, 0.0, 0.0, 0.0, 7.0, "stopped", 0.5),
        target=DynamicControlTarget(0.5, 0.0),
        drive_model=model,
    )
    grid = np.zeros((31, 31), dtype=np.uint8)
    field = dynamic.obstacle_distance_field(grid, resolution_m=0.1)
    assert dynamic_primitive_is_collision_free(
        primitive,
        occupancy_grid=grid,
        origin_xy=(0.0, 0.0),
        resolution_m=0.1,
    )
    assert dynamic.dynamic_primitive_cost(
        primitive,
        previous_target=DynamicControlTarget(0.0, 0.0),
        previous_direction="stopped",
        distance_field=field,
        origin_xy=(0.0, 0.0),
        resolution_m=0.1,
    ) == pytest.approx(0.2087488267004275)
    with pytest.raises(ValueError, match="primitive samples must have shape"):
        dynamic_primitive_is_collision_free(
            replace(primitive, samples=np.empty((0, 8))),
            occupancy_grid=grid,
            origin_xy=(0.0, 0.0),
            resolution_m=0.1,
        )

    conversions: list[np.ndarray] = []
    original_floor = np.floor

    def tracking_floor(values, *args, **kwargs):
        array = np.asarray(values)
        if array.ndim == 2 and array.shape[1:] == (2,) and array.shape[0] in (50, 51):
            conversions.append(array.copy())
        return original_floor(values, *args, **kwargs)

    monkeypatch.setattr(dynamic.np, "floor", tracking_floor)
    result = dynamic.plan_dynamic_lattice(_request(), model)

    assert result.status == "success"
    assert not any(
        full.shape == (51, 2)
        and later.shape == (50, 2)
        and np.array_equal(full[1:], later)
        for full in conversions
        for later in conversions
    )
