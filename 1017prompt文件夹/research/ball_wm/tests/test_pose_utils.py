"""Behavior tests for odometry-frame PointGoal and relative-pose geometry."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from training.pose_utils import pointgoal_in_base, relative_planar_pose
except ModuleNotFoundError:
    pointgoal_in_base = None
    relative_planar_pose = None


def _wxyz_from_yaw(yaw: float) -> np.ndarray:
    return np.asarray(
        [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
        dtype=np.float32,
    )


def test_relative_pose_rotates_world_displacement_into_source_base_frame():
    """Catches a label implementation that leaves XY displacement in world axes."""

    if relative_planar_pose is None:
        pytest.fail("relative_planar_pose is not available")
    label = relative_planar_pose(
        np.asarray([1.0, 2.0, 0.0], dtype=np.float32),
        _wxyz_from_yaw(math.pi / 2.0),
        np.asarray([1.0, 3.0, 0.0], dtype=np.float32),
        _wxyz_from_yaw(math.pi),
    )

    np.testing.assert_allclose(label[:2], [1.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(label[2:], [1.0, 0.0], atol=1e-6)


def test_pointgoal_in_base_uses_current_yaw_without_environment_map():
    """Catches a PointGoal transform that ignores odometry yaw or swaps base axes."""

    if pointgoal_in_base is None:
        pytest.fail("pointgoal_in_base is not available")
    goal_base = pointgoal_in_base(
        np.asarray([0.0, 0.0], dtype=np.float32),
        math.pi / 2.0,
        np.asarray([0.0, 2.0], dtype=np.float32),
    )

    np.testing.assert_allclose(goal_base, [2.0, 0.0], atol=1e-6)
