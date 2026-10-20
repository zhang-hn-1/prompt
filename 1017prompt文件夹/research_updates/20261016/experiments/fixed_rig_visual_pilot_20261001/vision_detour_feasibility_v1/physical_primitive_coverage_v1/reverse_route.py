"""One privileged continuation of the measured reverse-positive escape."""

from __future__ import annotations

import numpy as np

from benchmark.grid_planning import astar_path, inflate_obstacles
from benchmark.policies import geometric_tracker_action
from primitive_measurement import primitive_block


def reverse_tracker_action(protocol, target_base_xy: np.ndarray) -> np.ndarray:
    return -geometric_tracker_action(protocol, -np.asarray(target_base_xy, dtype=float))


class ReverseEscapeOracle:
    policy_name = "oracle_reverse_escape_continuation_v1"
    privileged = True
    uses_shared_tracker = True
    online_input_permissions = frozenset({"known_occupancy_map", "position_world_xy",
                                           "yaw_world_rad", "pointgoal_base_xy"})

    def __init__(self, protocol, occupancy_path):
        self.protocol = protocol
        with np.load(occupancy_path) as occ:
            grid = np.where(occ["grid"] == 0, 0, 1).astype(np.uint8)
            self._origin = occ["global_origin_xy"].astype(float)
            self._resolution = float(occ["resolution_m"])
        self._grid = inflate_obstacles(grid, radius_m=0.6,
                                       resolution_m=self._resolution)
        self.records = []
        self._path = None
        self._arc = None
        self._index = 0
        self._blocks = 0

    def reset(self, context):
        if not context.privileged or context.action_protocol_id != self.protocol.protocol_id:
            raise ValueError("requires matching privileged context")
        self.records.clear()
        self._path = None
        self._arc = None
        self._index = 0
        self._blocks = 0

    def _plan_from_current(self, position, yaw, goal_base):
        c, s = np.cos(yaw), np.sin(yaw)
        goal_world = position + np.array([[c, -s], [s, c]]) @ goal_base
        start_cell = tuple(np.rint((position - self._origin) / self._resolution).astype(int))
        goal_cell = tuple(np.rint((goal_world - self._origin) / self._resolution).astype(int))
        cells = astar_path(self._grid, start=start_cell, goal=goal_cell)
        path = self._origin + cells * self._resolution
        self._path = np.vstack((position, path[1:-1], goal_world))
        self._arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self._path, axis=0), axis=1))]

    def act(self, observation, pointgoal_base_xy):
        goal = np.asarray(pointgoal_base_xy, dtype=float)
        position = np.asarray(observation.position_world_xy, dtype=float)
        yaw = float(observation.yaw_world_rad)
        if self._blocks < 60:
            action = primitive_block("reverse_positive", controls_per_block=self.protocol.controls_per_block)
            stage = "escape"
            target = None
        else:
            if self._path is None:
                self._plan_from_current(position, yaw, goal)
            self._index += int(np.argmin(np.linalg.norm(self._path[self._index:] - position, axis=1)))
            target_index = min(int(np.searchsorted(self._arc, self._arc[self._index] + 1.0)), len(self._path) - 1)
            delta = self._path[target_index] - position
            c, s = np.cos(yaw), np.sin(yaw)
            target = np.array([[c, s], [-s, c]]) @ delta
            if target_index == len(self._path) - 1:
                target = goal
            action = reverse_tracker_action(self.protocol, target)
            stage = "reverse_route"
        self.records.append({"block": self._blocks, "stage": stage,
                             "position_world_xy": position.tolist(), "yaw_world_rad": yaw,
                             "goal_base_xy": goal.tolist(),
                             "target_base_xy": None if target is None else target.tolist(),
                             "reference_index": self._index,
                             "request_main_aux": action[:2].tolist()})
        self._blocks += 1
        return action
