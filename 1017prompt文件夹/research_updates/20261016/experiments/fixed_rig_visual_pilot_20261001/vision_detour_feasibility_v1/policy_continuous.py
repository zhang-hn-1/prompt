"""Continuous local reference after the frozen RGB trigger."""

from __future__ import annotations

import math

import numpy as np

from policy import VisionDetourPolicy


def smoothstep(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


class ContinuousVisionDetourPolicy(VisionDetourPolicy):
    policy_name = "vision_detour_continuous_v1"

    def __init__(self, protocol):
        super().__init__(protocol)
        self._path_xy = None
        self._path_s = None
        self._index = 0
        self._triggered = False

    def reset(self, context):
        super().reset(context)
        self._path_xy = None
        self._path_s = None
        self._index = 0
        self._triggered = False

    def _make_path(self, observation, goal):
        pos = np.asarray(observation.position_world_xy, dtype=np.float64)
        yaw = float(observation.yaw_world_rad)
        c, s = math.cos(yaw), math.sin(yaw)
        goal_world = pos + np.array([c * goal[0] - s * goal[1],
                                     s * goal[0] + c * goal[1]])
        delta = goal_world - pos
        distance = float(np.linalg.norm(delta))
        if distance < 5.0:
            return
        forward = delta / distance
        right = np.array([forward[1], -forward[0]])
        x = np.append(np.arange(0.0, distance, 0.05), distance)
        offset = np.empty_like(x)
        first = x <= 3.0
        middle = (x > 3.0) & (x < distance - 2.6)
        last = ~(first | middle)
        offset[first] = 2.3 * smoothstep(x[first] / 3.0)
        offset[middle] = 2.3
        offset[last] = 2.3 * (1 - smoothstep((x[last] - (distance - 2.6)) / 2.6))
        self._path_xy = pos + x[:, None] * forward + offset[:, None] * right
        segments = np.linalg.norm(np.diff(self._path_xy, axis=0), axis=1)
        self._path_s = np.concatenate(([0.0], np.cumsum(segments)))
        self._index = 0

    def act(self, observation, pointgoal_base_xy):
        goal = np.asarray(pointgoal_base_xy, dtype=np.float64)
        count = self._obstacle_count(observation.stereo_left_rgb)
        if not self._triggered:
            self._consecutive = self._consecutive + 1 if count >= 1000 else 0
            if self._consecutive >= 2:
                self._triggered = True
                self._make_path(observation, goal)

        target = goal
        if self._path_xy is not None:
            pos = np.asarray(observation.position_world_xy, dtype=np.float64)
            local_nearest = int(np.argmin(np.linalg.norm(self._path_xy[self._index:] - pos, axis=1)))
            self._index += local_nearest
            target_s = self._path_s[self._index] + 1.0
            target_index = min(int(np.searchsorted(self._path_s, target_s)), len(self._path_xy) - 1)
            delta = self._path_xy[target_index] - pos
            yaw = float(observation.yaw_world_rad)
            c, s = math.cos(yaw), math.sin(yaw)
            target = np.array([c * delta[0] + s * delta[1],
                               -s * delta[0] + c * delta[1]])
            if target_index == len(self._path_xy) - 1:
                target = goal

        action = super(VisionDetourPolicy, self).act(observation, target)
        self.records.append({
            "obstacle_pixels": count,
            "consecutive": self._consecutive,
            "detour_triggered": self._triggered,
            "reference_index": self._index,
            "position_world_xy": observation.position_world_xy.tolist(),
            "goal_base_xy": goal.tolist(),
            "target_base_xy": target.tolist(),
            "main_request_rad_s": float(action[0]),
            "aux_request_rad": float(action[1]),
        })
        return action
