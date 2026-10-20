"""Tests for deterministic, footprint-aware grid planning."""

from __future__ import annotations

import math

import numpy as np
import pytest

from benchmark.grid_planning import (
    astar_path,
    inflate_obstacles,
    line_is_free,
    path_length_cells,
)


def test_astar_detours_without_diagonal_corner_cutting():
    grid = np.zeros((9, 9), dtype=np.uint8)
    grid[4, 2:7] = 1

    path = astar_path(grid, start=(1, 4), goal=(7, 4), occupied=1)

    assert tuple(path[0]) == (1, 4)
    assert tuple(path[-1]) == (7, 4)
    assert not np.any(grid[path[:, 0], path[:, 1]] == 1)
    assert path_length_cells(path) > 6.0


def test_inflation_uses_ball_plus_margin():
    grid = np.zeros((21, 21), dtype=np.uint8)
    grid[10, 10] = 1

    inflated = inflate_obstacles(
        grid,
        radius_m=0.6,
        resolution_m=0.1,
        occupied=1,
    )

    assert inflated[16, 10] == 1
    assert inflated[17, 10] == 0
    assert inflated[14, 14] == 1
    assert inflated[15, 15] == 0


def test_astar_rejects_diagonal_corner_cutting():
    grid = np.asarray([[0, 1], [1, 0]], dtype=np.uint8)

    with pytest.raises(ValueError, match="unreachable"):
        astar_path(grid, start=(0, 0), goal=(1, 1), occupied=1)

    assert not line_is_free(grid, start=(0, 0), goal=(1, 1), occupied=1)


@pytest.mark.parametrize("cell", ((-1, 0), (0, 3), (3, 0)))
def test_astar_rejects_cells_outside_map(cell: tuple[int, int]):
    grid = np.zeros((3, 3), dtype=np.uint8)

    with pytest.raises(ValueError, match="outside"):
        astar_path(grid, start=cell, goal=(1, 1), occupied=1)


def test_astar_rejects_occupied_start_and_goal():
    grid = np.zeros((5, 5), dtype=np.uint8)
    grid[1, 1] = 1
    grid[3, 3] = 1

    with pytest.raises(ValueError, match="start.*occupied"):
        astar_path(grid, start=(1, 1), goal=(2, 2), occupied=1)
    with pytest.raises(ValueError, match="goal.*occupied"):
        astar_path(grid, start=(2, 2), goal=(3, 3), occupied=1)


def test_astar_reports_unreachable_goal():
    grid = np.zeros((7, 7), dtype=np.uint8)
    grid[3, :] = 1

    with pytest.raises(ValueError, match="unreachable"):
        astar_path(grid, start=(1, 3), goal=(5, 3), occupied=1)


def test_path_length_uses_unit_and_diagonal_costs():
    path = np.asarray(((0, 0), (1, 0), (2, 1)), dtype=np.int64)

    assert path_length_cells(path) == pytest.approx(1.0 + math.sqrt(2.0))
