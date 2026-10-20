"""Stereo RGB local policy for an isolated front-external camera rig."""

import math

import numpy as np

from vertical_depth import FOCAL_LENGTH_PX, vertical_stereo_depth
from visual_local_policy import StereoLocalPolicy, occupied_cells


def _root_rotation(quaternion_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion_wxyz, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("expected finite wxyz attitude")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def project_external_depth(
    depth_m: np.ndarray,
    valid: np.ndarray,
    camera: str,
    quaternion_world_wxyz: np.ndarray,
) -> np.ndarray:
    """Project optical depth to gravity-horizontal, robot-yaw XY coordinates."""
    depth = np.asarray(depth_m)
    mask = np.asarray(valid, dtype=bool)
    if depth.shape != (480, 640) or mask.shape != depth.shape:
        raise ValueError("expected benchmark 480x640 depth")
    if camera not in {"left", "right"}:
        raise ValueError("camera must be left or right")
    row, column = np.where(mask)
    distance = depth[row, column]
    lateral_origin = 0.1 if camera == "left" else -0.1
    local = np.stack((
        distance + 0.48,
        lateral_origin - (row - 239.5) * distance / FOCAL_LENGTH_PX,
        0.1 + (column - 319.5) * distance / FOCAL_LENGTH_PX,
    ), axis=1)
    rotation = _root_rotation(quaternion_world_wxyz)
    world_relative = local @ rotation.T
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    forward = cosine * world_relative[:, 0] + sine * world_relative[:, 1]
    lateral = -sine * world_relative[:, 0] + cosine * world_relative[:, 1]
    keep = (
        (forward >= 0.5) & (forward <= 4.5)
        & (np.abs(lateral) <= 3.0)
        & (world_relative[:, 2] >= -0.35)
        & (world_relative[:, 2] <= 0.55)
    )
    return np.stack((forward[keep], lateral[keep]), axis=1).astype(np.float32)


def choose_external_heading(pointgoal_xy: np.ndarray, obstacles_xy: np.ndarray) -> tuple[float, bool]:
    goal = np.asarray(pointgoal_xy, dtype=np.float64)
    obstacles = np.asarray(obstacles_xy, dtype=np.float64).reshape(-1, 2)
    if goal.shape != (2,) or not np.isfinite(goal).all():
        raise ValueError("pointgoal must be finite XY")
    nominal = math.atan2(float(goal[1]), float(goal[0]))
    candidates = np.unique(np.clip(nominal + np.linspace(-0.9, 0.9, 19), -1.05, 1.05))
    horizon = min(2.2, max(0.8, float(np.linalg.norm(goal))))
    best = nominal
    best_cost = float("inf")
    clear = False
    for angle in candidates:
        direction = np.array([math.cos(float(angle)), math.sin(float(angle))])
        along = obstacles @ direction
        side = np.abs(obstacles[:, 0] * direction[1] - obstacles[:, 1] * direction[0])
        if np.any((along > 0.35) & (along < horizon) & (side < 0.45)):
            continue
        clear = True
        cost = 1.8 * abs(float(angle) - nominal) + 0.15 * abs(float(angle))
        if cost < best_cost:
            best, best_cost = float(angle), cost
    return best, bool(len(obstacles) and not clear)


class StereoExternalPolicyV3(StereoLocalPolicy):
    policy_name = "stereo_external_v3"
    online_input_permissions = StereoLocalPolicy.online_input_permissions | frozenset({
        "quaternion_world_wxyz_history",
    })

    def act(self, observation, pointgoal_base_xy):
        if not self._ready:
            raise RuntimeError("reset must precede act")
        if observation.quaternion_world_wxyz_history is None:
            raise ValueError("current measured attitude is required")
        quaternion = observation.quaternion_world_wxyz_history[-1]
        goal = np.asarray(pointgoal_base_xy, dtype=np.float64)
        cells = []
        coverage = {}
        for camera in ("left", "right"):
            depth, valid = vertical_stereo_depth(
                observation.stereo_left_rgb,
                observation.stereo_right_rgb,
                anchor=camera,
                baseline_m=0.2,
            )
            coverage[camera] = float(valid.mean())
            cells.append(occupied_cells(project_external_depth(depth, valid, camera, quaternion)))
        obstacles = np.concatenate(cells, axis=0)
        heading, blocked = choose_external_heading(goal, obstacles)
        distance = float(np.linalg.norm(goal))
        main = 0.0 if blocked or distance <= 0.2 else (0.7 if abs(heading) > 0.45 else 1.2)
        if distance < 0.6:
            main *= max(0.0, distance / 0.6)
        auxiliary = float(np.clip(-heading, -math.pi / 6, math.pi / 6))
        action = self.protocol.clip_block(
            np.tile(np.array([main, auxiliary], dtype=np.float32), 5)
        )
        self.records.append({
            "goal_distance_m": distance,
            "goal_heading_rad": math.atan2(float(goal[1]), float(goal[0])),
            "selected_heading_rad": heading,
            "blocked": blocked,
            "valid_left_fraction": coverage["left"],
            "valid_right_fraction": coverage["right"],
            "left_obstacle_cells": int(len(cells[0])),
            "right_obstacle_cells": int(len(cells[1])),
            "main_request_rad_s": float(action[0]),
            "aux_request_rad": float(action[1]),
        })
        return action
