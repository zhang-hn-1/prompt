"""Project fixed-rig RGB floor evidence into world XY without depth input."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


ROOT_HEIGHT_M = 0.4
GROUND_HEIGHT_M = 0.0
FOCAL_PX = 16.45 / 18.0 * 640.0
CX, CY = 319.5, 239.5
CAMERA_OFFSET_BASE_M = {
    "left": np.array([0.1, 0.3, 0.1], dtype=np.float64),
    "right": np.array([0.1, -0.3, 0.1], dtype=np.float64),
}
# Camera optical axes in base_link coordinates: x up, y toward robot right,
# z forward. The vertical 0.6 m stereo baseline follows from these axes.
OPTICAL_TO_BASE = np.array([[0.0, 0.0, 1.0],
                            [0.0, -1.0, 0.0],
                            [1.0, 0.0, 0.0]], dtype=np.float64)


def camera_pose_from_root(
    position_world_xy: np.ndarray,
    quaternion_world_wxyz: np.ndarray,
    eye: str,
    *,
    root_height_m: float = ROOT_HEIGHT_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Return world camera origin and optical-to-world rotation for the frozen rig."""
    if eye not in CAMERA_OFFSET_BASE_M:
        raise ValueError("eye must be left or right")
    xy = np.asarray(position_world_xy, dtype=np.float64)
    quaternion = np.asarray(quaternion_world_wxyz, dtype=np.float64)
    if xy.shape != (2,) or quaternion.shape != (4,) or not np.isfinite(xy).all() or not np.isfinite(quaternion).all():
        raise ValueError("position and quaternion must be finite XY and WXYZ values")
    if not np.isfinite(root_height_m):
        raise ValueError("root_height_m must be finite")
    root_rotation = Rotation.from_quat(np.r_[quaternion[1:], quaternion[0]]).as_matrix()
    root = np.array([xy[0], xy[1], root_height_m], dtype=np.float64)
    origin = root + root_rotation @ CAMERA_OFFSET_BASE_M[eye]
    return origin, root_rotation @ OPTICAL_TO_BASE


def project_ground_pixels(
    free_mask: np.ndarray,
    position_world_xy: np.ndarray,
    quaternion_world_wxyz: np.ndarray,
    eye: str,
) -> tuple[np.ndarray, np.ndarray]:
    """Return projected world XY and validity aligned to selected 4x pixels.

    A returned point is visible floor evidence only. It cannot certify any
    unobserved cell as free and must not be treated as an obstacle-free ray.
    """
    mask = np.asarray(free_mask, dtype=bool)
    if mask.shape != (120, 160):
        raise ValueError("free_mask must have shape (120, 160)")
    origin, optical_to_world = camera_pose_from_root(position_world_xy, quaternion_world_wxyz, eye)
    rows, cols = np.mgrid[0:480:4, 0:640:4]
    u = cols[mask].astype(np.float64)
    v = rows[mask].astype(np.float64)
    rays = np.column_stack(((u - CX) / FOCAL_PX,
                            (v - CY) / FOCAL_PX,
                            np.ones(len(u)))) @ optical_to_world.T
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = (GROUND_HEIGHT_M - origin[2]) / rays[:, 2]
    valid = np.isfinite(distance) & (distance >= 0.3) & (distance <= 8.0)
    xy = origin[:2] + rays[valid, :2] * distance[valid, None]
    return xy, valid
