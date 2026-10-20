"""Isolated, non-privileged classical stereo local-navigation baseline."""

import math

import numpy as np

from vertical_depth import FOCAL_LENGTH_PX, vertical_stereo_depth


def points_from_depth(
    depth_m: np.ndarray,
    valid: np.ndarray,
    *,
    image_row_limit: int = 300,
    camera: str = "left",
) -> np.ndarray:
    """Project left-camera pixels to robot-forward XY, excluding self view."""
    depth = np.asarray(depth_m)
    valid_mask = np.asarray(valid, dtype=bool)
    if depth.shape != (480, 640) or valid_mask.shape != depth.shape:
        raise ValueError("expected 480x640 depth and validity")
    if camera not in {"left", "right"}:
        raise ValueError("camera must be left or right")
    row, column = np.where(valid_mask & (np.arange(480)[:, None] < image_row_limit))
    z_cam = depth[row, column]
    forward = z_cam + 0.1
    lateral_origin = 0.3 if camera == "left" else -0.3
    lateral = lateral_origin - (row.astype(np.float32) - 239.5) * z_cam / FOCAL_LENGTH_PX
    height = 0.1 + (column.astype(np.float32) - 319.5) * z_cam / FOCAL_LENGTH_PX
    keep = (
        (forward >= 0.6)
        & (forward <= 4.5)
        & (np.abs(lateral) <= 3.0)
        & (height >= -0.15)
        & (height <= 1.2)
    )
    return np.stack((forward[keep], lateral[keep]), axis=1).astype(np.float32)


def occupied_cells(points: np.ndarray, *, min_pixels: int = 25) -> np.ndarray:
    """Require a supported 25-cm cell before treating disparity as an obstacle."""
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float32)
    x_edges = np.arange(0.0, 4.75, 0.25)
    y_edges = np.arange(-3.0, 3.25, 0.25)
    histogram, _, _ = np.histogram2d(points[:, 0], points[:, 1], bins=(x_edges, y_edges))
    ix, iy = np.where(histogram >= min_pixels)
    return np.stack((x_edges[ix] + 0.125, y_edges[iy] + 0.125), axis=1).astype(np.float32)


def choose_local_heading(pointgoal_xy: np.ndarray, obstacles_xy: np.ndarray) -> tuple[float, bool]:
    """Choose a short straight corridor with fixed obstacle clearance."""
    goal = np.asarray(pointgoal_xy, dtype=np.float64)
    obstacles = np.asarray(obstacles_xy, dtype=np.float64).reshape(-1, 2)
    if goal.shape != (2,) or not np.isfinite(goal).all():
        raise ValueError("pointgoal must be a finite XY vector")
    nominal = math.atan2(float(goal[1]), float(goal[0]))
    candidates = np.clip(nominal + np.linspace(-0.9, 0.9, 19), -1.05, 1.05)
    candidates = np.unique(candidates)
    range_m = min(2.2, max(0.8, float(np.linalg.norm(goal))))
    best_score = float("inf")
    best_heading = nominal
    found_clear = False
    for angle in candidates:
        direction = np.array([math.cos(float(angle)), math.sin(float(angle))])
        along = obstacles @ direction
        side = np.abs(obstacles[:, 0] * direction[1] - obstacles[:, 1] * direction[0])
        threatened = (along > 0.35) & (along < range_m) & (side < 0.5)
        if threatened.any():
            continue
        found_clear = True
        score = 1.8 * abs(float(angle) - nominal) + 0.15 * abs(float(angle))
        if score < best_score:
            best_score = score
            best_heading = float(angle)
    return best_heading, bool(len(obstacles) and not found_clear)


class StereoLocalPolicy:
    policy_name = "stereo_local_v1"
    privileged = False
    uses_shared_tracker = False
    online_input_permissions = frozenset({"stereo_rgb", "proprio_history", "action_history", "pointgoal_base_xy"})

    def __init__(self, protocol):
        self.protocol = protocol
        self.records = []
        self._ready = False

    def reset(self, context):
        if context.privileged or context.oracle_path_xy is not None or context.kinodynamic_reference is not None or context.dynamic_reference is not None:
            raise PermissionError("stereo policy cannot receive an Oracle reference")
        if context.action_protocol_id != self.protocol.protocol_id:
            raise ValueError("protocol mismatch")
        self.records.clear()
        self._ready = True

    def act(self, observation, pointgoal_base_xy):
        if not self._ready:
            raise RuntimeError("reset must precede act")
        goal = np.asarray(pointgoal_base_xy, dtype=np.float64)
        depth, valid = vertical_stereo_depth(observation.stereo_left_rgb, observation.stereo_right_rgb)
        points = points_from_depth(depth, valid)
        cells = occupied_cells(points)
        heading, blocked = choose_local_heading(goal, cells)
        distance = float(np.linalg.norm(goal))
        main = 0.0 if blocked or distance <= 0.2 else (0.7 if abs(heading) > 0.45 else 1.2)
        if distance < 0.6:
            main *= max(0.0, distance / 0.6)
        auxiliary = float(np.clip(-heading, -math.pi / 6, math.pi / 6))
        block = np.tile(np.array([main, auxiliary], dtype=np.float32), 5)
        action = self.protocol.clip_block(block)
        self.records.append({
            "goal_distance_m": distance,
            "goal_heading_rad": math.atan2(float(goal[1]), float(goal[0])),
            "selected_heading_rad": heading,
            "blocked": blocked,
            "valid_depth_fraction": float(valid.mean()),
            "obstacle_cells": int(len(cells)),
            "main_request_rad_s": float(action[0]),
            "aux_request_rad": float(action[1]),
        })
        return action
