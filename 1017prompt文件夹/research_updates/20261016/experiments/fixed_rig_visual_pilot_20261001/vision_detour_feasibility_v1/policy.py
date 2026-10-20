"""Map-free RGB-triggered right-side detour feasibility controller."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

from benchmark.policies import GoalGreedyPolicy


HERE = Path(__file__).resolve().parent
PILOT = HERE.parent
sys.path.insert(0, str(PILOT))
import pilot  # noqa: E402


MODEL = PILOT / "illuminated_replay_v1" / "augmented" / "model_holdout_e1.pt"


class VisionDetourPolicy(GoalGreedyPolicy):
    policy_name = "vision_detour_feasibility_v1"
    online_input_permissions = frozenset({
        "stereo_rgb", "proprio_history", "action_history", "pointgoal_base_xy",
        "position_world_xy", "yaw_world_rad",
    })

    def __init__(self, protocol):
        super().__init__(protocol)
        import torch

        self._torch = torch
        self._torch.set_num_threads(2)
        self._model = pilot.TinyPixelNet().eval()
        self._model.load_state_dict(torch.load(MODEL, map_location="cpu", weights_only=True))
        self.records = []
        self._consecutive = 0
        self._waypoints = None
        self._stage = 0

    def reset(self, context):
        super().reset(context)
        self.records.clear()
        self._consecutive = 0
        self._waypoints = None
        self._stage = 0

    def _obstacle_count(self, rgb: np.ndarray) -> int:
        image = np.asarray(rgb)[::4, ::4][None]
        with self._torch.no_grad():
            x = pilot.rgb_input(image, np.array([0]), self._torch.device("cpu"))
            pred = self._model(x).argmax(1).numpy()[0]
        return int((pred[40:100, 40:140] == pilot.OBSTACLE).sum())

    def act(self, observation, pointgoal_base_xy):
        goal = np.asarray(pointgoal_base_xy, dtype=np.float64)
        count = self._obstacle_count(observation.stereo_left_rgb)
        if self._waypoints is None:
            self._consecutive = self._consecutive + 1 if count >= 1000 else 0
            if self._consecutive >= 2:
                pos = np.asarray(observation.position_world_xy, dtype=np.float64)
                yaw = float(observation.yaw_world_rad)
                c, s = math.cos(yaw), math.sin(yaw)
                goal_world = pos + np.array([c * goal[0] - s * goal[1],
                                             s * goal[0] + c * goal[1]])
                forward = goal_world - pos
                forward /= max(np.linalg.norm(forward), 1e-9)
                right = np.array([forward[1], -forward[0]])
                self._waypoints = (pos + 1.0 * forward + 1.9 * right,
                                   pos + 4.6 * forward + 1.9 * right)
                self._stage = 0

        target = goal
        if self._waypoints is not None and self._stage < len(self._waypoints):
            pos = np.asarray(observation.position_world_xy, dtype=np.float64)
            if np.linalg.norm(self._waypoints[self._stage] - pos) < 0.5:
                self._stage += 1
            if self._stage < len(self._waypoints):
                delta = self._waypoints[self._stage] - pos
                yaw = float(observation.yaw_world_rad)
                c, s = math.cos(yaw), math.sin(yaw)
                target = np.array([c * delta[0] + s * delta[1],
                                   -s * delta[0] + c * delta[1]])

        action = super().act(observation, target)
        self.records.append({
            "obstacle_pixels": count,
            "consecutive": self._consecutive,
            "detour_triggered": self._waypoints is not None,
            "stage": self._stage,
            "position_world_xy": observation.position_world_xy.tolist(),
            "goal_base_xy": goal.tolist(),
            "target_base_xy": target.tolist(),
            "main_request_rad_s": float(action[0]),
            "aux_request_rad": float(action[1]),
        })
        return action
