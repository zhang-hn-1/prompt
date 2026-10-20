"""Synthetic geometry checks independent of benchmark scene content."""

import numpy as np

from curve_labels import candidate_centerline, clearance_grid, classify_curve


def test_centerline_starts_at_robot_and_lateral_signs() -> None:
    start = np.array([2.0, -3.0])
    straight = candidate_centerline(start, 0.0, "straight")
    left = candidate_centerline(start, 0.0, "left")
    right = candidate_centerline(start, 0.0, "right")
    assert np.allclose(straight[0], start)
    assert np.allclose(left[0], start)
    assert np.allclose(right[0], start)
    assert np.isclose(straight[-1, 0], start[0] + 4.0)
    assert np.isclose(left[-1, 1], start[1] + 2.3)
    assert np.isclose(right[-1, 1], start[1] - 2.3)


def test_blocked_straight_and_clear_detours() -> None:
    grid = np.zeros((201, 201), dtype=np.uint8)
    grid[120, 100] = 1  # world (2, 0) at 0.1 m/cell
    clearance = clearance_grid(grid, 0.1)
    origin = np.array([-10.0, -10.0])
    start = np.array([0.0, 0.0])
    labels = {side: classify_curve(candidate_centerline(start, 0.0, side),
                                   clearance, origin, 0.1)["label"]
              for side in ("straight", "left", "right")}
    assert labels == {"straight": "blocked", "left": "clear", "right": "clear"}


def test_map_exit_is_unknown_unless_known_block_precedes_it() -> None:
    grid = np.zeros((31, 31), dtype=np.uint8)
    clearance = clearance_grid(grid, 0.1)
    start = np.array([0.0, 0.0])
    origin = np.array([-1.5, -1.5])
    assert classify_curve(candidate_centerline(start, 0.0, "straight"),
                          clearance, origin, 0.1)["label"] == "unknown"
    grid[25, 15] = 1
    blocked = clearance_grid(grid, 0.1)
    assert classify_curve(candidate_centerline(start, 0.0, "straight"),
                          blocked, origin, 0.1)["label"] == "blocked"


def test_diagnostic_radius_threshold_does_not_change_frozen_default() -> None:
    grid = np.zeros((101, 101), dtype=np.uint8)
    grid[70, 55] = 1  # 0.5 m lateral to straight path at x=2 m
    distance = clearance_grid(grid, 0.1)
    path = candidate_centerline(np.array([0.0, 0.0]), 0.0, "straight")
    origin = np.array([-5.0, -5.0])
    assert classify_curve(path, distance, origin, 0.1)["label"] == "blocked"
    assert classify_curve(path, distance, origin, 0.1,
                          required_clearance_m=0.4)["label"] == "clear"
