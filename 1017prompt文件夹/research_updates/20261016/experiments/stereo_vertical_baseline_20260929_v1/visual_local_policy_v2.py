"""Symmetric, texture-supported stereo candidate; isolated from V1."""

import math

import cv2
import numpy as np

from vertical_depth import vertical_stereo_depth
from visual_local_policy import (
    StereoLocalPolicy,
    choose_local_heading,
    occupied_cells,
    points_from_depth,
)


def supported_depth_mask(image_rgb: np.ndarray, valid: np.ndarray, *, camera: str) -> np.ndarray:
    """Drop dark matching artifacts and fixed self-obscured image regions."""
    image = np.asarray(image_rgb)
    mask = np.asarray(valid, dtype=bool)
    if image.shape != (480, 640, 3) or mask.shape != (480, 640):
        raise ValueError("expected benchmark RGB and disparity validity")
    if camera not in {"left", "right"}:
        raise ValueError("camera must be left or right")
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    row = np.arange(480)[:, None]
    column = np.arange(640)[None, :]
    visible = row < 270 if camera == "left" else (row > 90) & (row < 350)
    return mask & (gray > 20) & visible & (column > 120)


class StereoLocalPolicyV2(StereoLocalPolicy):
    policy_name = "stereo_local_v2_bilateral_texture"

    def act(self, observation, pointgoal_base_xy):
        if not self._ready:
            raise RuntimeError("reset must precede act")
        goal = np.asarray(pointgoal_base_xy, dtype=np.float64)
        left = observation.stereo_left_rgb
        right = observation.stereo_right_rgb
        cells = []
        fractions = {}
        for camera in ("left", "right"):
            depth, valid = vertical_stereo_depth(left, right, anchor=camera)
            image = left if camera == "left" else right
            mask = supported_depth_mask(image, valid, camera=camera)
            fractions[camera] = float(mask.mean())
            cells.append(occupied_cells(points_from_depth(depth, mask, camera=camera)))
        obstacles = np.concatenate(cells, axis=0)
        heading, blocked = choose_local_heading(goal, obstacles)
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
            "supported_left_fraction": fractions["left"],
            "supported_right_fraction": fractions["right"],
            "left_obstacle_cells": int(len(cells[0])),
            "right_obstacle_cells": int(len(cells[1])),
            "main_request_rad_s": float(action[0]),
            "aux_request_rad": float(action[1]),
        })
        return action
