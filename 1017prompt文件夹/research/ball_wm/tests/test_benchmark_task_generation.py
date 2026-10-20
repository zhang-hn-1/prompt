"""Tests for auditable E0/E1/E2 benchmark task validation."""

from __future__ import annotations

import numpy as np
import pytest

from benchmark.task_generation import (
    TaskEvidence,
    build_two_sided_corridor_mask,
    validate_task,
)


RESOLUTION_M = 0.1


def open_grid() -> np.ndarray:
    return np.zeros((90, 90), dtype=np.uint8)


def e1_grid() -> np.ndarray:
    grid = open_grid()
    grid[15:76, 45] = 1
    return grid


def e2_grid_and_corridor() -> tuple[np.ndarray, np.ndarray]:
    grid = np.zeros((100, 100), dtype=np.uint8)
    grid[:43, 50] = 1
    grid[57:, 50] = 1
    corridor = np.zeros_like(grid, dtype=bool)
    corridor[43:57, 45:56] = True
    return grid, corridor


def test_e0_requires_a_free_straight_line():
    evidence = validate_task(
        open_grid(),
        start=(45, 10),
        goal=(45, 70),
        difficulty="E0",
        resolution_m=RESOLUTION_M,
    )

    assert isinstance(evidence, TaskEvidence)
    assert evidence.line_is_free
    assert evidence.euclidean_distance_m == pytest.approx(6.0)
    assert evidence.oracle_path_length_m == pytest.approx(6.0)
    assert evidence.detour_ratio == pytest.approx(1.0)
    assert evidence.corridor_intersection is False
    assert evidence.corridor_width_m is None


def test_e0_counts_distinct_obstacles_near_the_oracle_path():
    grid = open_grid()
    grid[30:33, 24:27] = 1
    grid[30:33, 44:47] = 1
    grid[30:33, 64:67] = 1

    evidence = validate_task(
        grid,
        start=(45, 10),
        goal=(45, 80),
        difficulty="E0",
        resolution_m=RESOLUTION_M,
        minimum_nearby_obstacles=3,
        nearby_obstacle_radius_m=2.0,
    )

    assert evidence.nearby_obstacle_count == 3


def test_e0_rejects_too_few_obstacles_near_the_path():
    grid = open_grid()
    grid[30:33, 24:27] = 1
    grid[30:33, 64:67] = 1

    with pytest.raises(ValueError, match="nearby obstacles"):
        validate_task(
            grid,
            start=(45, 10),
            goal=(45, 80),
            difficulty="E0",
            resolution_m=RESOLUTION_M,
            minimum_nearby_obstacles=3,
            nearby_obstacle_radius_m=2.0,
        )


def test_e0_rejects_a_blocked_straight_line():
    with pytest.raises(ValueError, match="E0.*straight line"):
        validate_task(
            e1_grid(),
            start=(45, 10),
            goal=(45, 80),
            difficulty="E0",
            resolution_m=RESOLUTION_M,
        )


def test_e1_requires_blocked_line_and_sufficient_detour():
    evidence = validate_task(
        e1_grid(),
        start=(45, 10),
        goal=(45, 80),
        difficulty="E1",
        resolution_m=RESOLUTION_M,
    )

    assert not evidence.line_is_free
    assert evidence.detour_ratio >= 1.15
    assert evidence.oracle_path_length_m > evidence.euclidean_distance_m


def test_e1_rejects_an_open_straight_line():
    with pytest.raises(ValueError, match="E1.*blocked"):
        validate_task(
            open_grid(),
            start=(45, 10),
            goal=(45, 70),
            difficulty="E1",
            resolution_m=RESOLUTION_M,
        )


def test_e1_rejects_a_trivial_detour():
    grid = open_grid()
    grid[45, 45] = 1

    with pytest.raises(ValueError, match="detour ratio"):
        validate_task(
            grid,
            start=(45, 10),
            goal=(45, 70),
            difficulty="E1",
            resolution_m=RESOLUTION_M,
        )


def test_e2_requires_oracle_path_through_valid_narrow_corridor():
    grid, corridor = e2_grid_and_corridor()

    evidence = validate_task(
        grid,
        start=(50, 10),
        goal=(50, 90),
        difficulty="E2",
        resolution_m=RESOLUTION_M,
        corridor_mask=corridor,
        corridor_width_m=1.4,
        minimum_corridor_path_length_m=0.8,
    )

    assert evidence.corridor_intersection
    assert evidence.corridor_width_m == pytest.approx(1.4)
    assert evidence.corridor_path_length_m >= 0.8


def test_e2_rejects_a_brief_corridor_mask_touch():
    grid, corridor = e2_grid_and_corridor()
    corridor[:] = False
    corridor[49:52, 48:53] = True

    with pytest.raises(ValueError, match="continuous corridor path length"):
        validate_task(
            grid,
            start=(50, 10),
            goal=(50, 90),
            difficulty="E2",
            resolution_m=RESOLUTION_M,
            corridor_mask=corridor,
            corridor_width_m=1.4,
            minimum_corridor_path_length_m=0.8,
        )


def test_two_sided_corridor_mask_detects_parallel_walls_not_obstacle_rings():
    corridor_grid = np.zeros((80, 80), dtype=np.uint8)
    corridor_grid[25, 10:70] = 1
    corridor_grid[39, 10:70] = 1

    corridor = build_two_sided_corridor_mask(
        corridor_grid,
        resolution_m=RESOLUTION_M,
        minimum_width_m=1.2,
        maximum_width_m=1.6,
    )

    assert corridor[32, 20:60].all()

    isolated_grid = np.zeros((80, 80), dtype=np.uint8)
    isolated_grid[38:43, 38:43] = 1
    isolated = build_two_sided_corridor_mask(
        isolated_grid,
        resolution_m=RESOLUTION_M,
        minimum_width_m=1.2,
        maximum_width_m=1.6,
    )

    assert not isolated.any()


def test_e2_rejects_corridor_not_used_by_oracle_path():
    grid, corridor = e2_grid_and_corridor()
    corridor[:] = False
    corridor[80:85, 80:85] = True

    with pytest.raises(ValueError, match="corridor"):
        validate_task(
            grid,
            start=(50, 10),
            goal=(50, 90),
            difficulty="E2",
            resolution_m=RESOLUTION_M,
            corridor_mask=corridor,
            corridor_width_m=1.4,
        )


@pytest.mark.parametrize("width_m", (1.19, 1.61, None))
def test_e2_rejects_corridor_outside_width_contract(width_m: float | None):
    grid, corridor = e2_grid_and_corridor()

    with pytest.raises(ValueError, match="corridor width"):
        validate_task(
            grid,
            start=(50, 10),
            goal=(50, 90),
            difficulty="E2",
            resolution_m=RESOLUTION_M,
            corridor_mask=corridor,
            corridor_width_m=width_m,
        )


def test_validation_rejects_start_inside_inflated_obstacle():
    grid = open_grid()
    grid[45, 15] = 1

    with pytest.raises(ValueError, match="start.*occupied"):
        validate_task(
            grid,
            start=(45, 10),
            goal=(45, 70),
            difficulty="E0",
            resolution_m=RESOLUTION_M,
        )


def test_validation_rejects_outside_and_unreachable_tasks():
    with pytest.raises(ValueError, match="outside"):
        validate_task(
            open_grid(),
            start=(90, 10),
            goal=(45, 70),
            difficulty="E0",
            resolution_m=RESOLUTION_M,
        )

    grid = open_grid()
    grid[:, 45] = 1
    with pytest.raises(ValueError, match="unreachable"):
        validate_task(
            grid,
            start=(45, 10),
            goal=(45, 70),
            difficulty="E1",
            resolution_m=RESOLUTION_M,
        )
