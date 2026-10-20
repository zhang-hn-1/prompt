"""Pure planar pose helpers shared by local PointGoal training and planning."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def _finite_xy(value: Any, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim != 1 or array.shape[0] not in (2, 3) or not np.isfinite(array).all():
        raise ValueError(f"{name} must be a finite vector with shape (2,) or (3,)")
    return array[:2]


def _finite_wxyz(value: Any, name: str) -> np.ndarray:
    quaternion = np.asarray(value, dtype=np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError(f"{name} must be a finite quaternion with shape (4,)")
    norm = float(np.linalg.norm(quaternion))
    if norm <= np.finfo(np.float64).eps:
        raise ValueError(f"{name} must have non-zero norm")
    return quaternion / norm


def planar_yaw_from_wxyz(quaternion_wxyz: Any) -> float:
    """Return the finite planar yaw angle from a normalized-or-not WXYZ quaternion."""

    w, x, y, z = _finite_wxyz(quaternion_wxyz, "quaternion_wxyz")
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def pointgoal_in_base(
    current_position_xy: Any,
    current_yaw: float,
    goal_position_xy: Any,
) -> np.ndarray:
    """Express an odometry-frame PointGoal as XY in the current base frame."""

    current = _finite_xy(current_position_xy, "current_position_xy")
    goal = _finite_xy(goal_position_xy, "goal_position_xy")
    yaw = float(current_yaw)
    if not math.isfinite(yaw):
        raise ValueError("current_yaw must be finite")
    delta = goal - current
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        [
            cosine * delta[0] + sine * delta[1],
            -sine * delta[0] + cosine * delta[1],
        ],
        dtype=np.float32,
    )


def relative_planar_pose(
    source_position: Any,
    source_quaternion_wxyz: Any,
    target_position: Any,
    target_quaternion_wxyz: Any,
) -> np.ndarray:
    """Return target translation/yaw relative to the source robot base frame."""

    source_yaw = planar_yaw_from_wxyz(source_quaternion_wxyz)
    target_yaw = planar_yaw_from_wxyz(target_quaternion_wxyz)
    relative_xy = pointgoal_in_base(source_position, source_yaw, target_position)
    delta_yaw = target_yaw - source_yaw
    return np.asarray(
        [
            relative_xy[0],
            relative_xy[1],
            math.sin(delta_yaw),
            math.cos(delta_yaw),
        ],
        dtype=np.float32,
    )
