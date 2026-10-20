"""Deterministic bidirectional Hybrid A* for the spherical robot."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Literal

import numpy as np

from benchmark.drive_model import DriveModel


DriveDirection = Literal["forward", "reverse"]
PlanStatus = Literal[
    "success",
    "invalid_start",
    "invalid_goal",
    "unreachable",
    "node_budget_exhausted",
]
YAW_BIN_COUNT = 72
YAW_BIN_WIDTH_RAD = 2.0 * math.pi / YAW_BIN_COUNT
MAIN_MAGNITUDES_RAD_S = (1.5, 1.0, 0.5)
AUXILIARY_ANGLES_RAD = tuple(
    math.radians(value) for value in (0.0, -10.0, 10.0, -20.0, 20.0, -30.0, 30.0)
)
_GRID_NEIGHBORS = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)


def _finite_vector(value: object, *, name: str, length: int) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain {length} finite values") from error
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {length} finite values")
    return result


def _positive_float(value: object, *, name: str, allow_zero: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite and positive") from error
    lower_ok = result >= 0.0 if allow_zero else result > 0.0
    if not math.isfinite(result) or not lower_ok:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


def world_to_cell(
    world_xy: object, *, origin_xy: object, resolution_m: float
) -> tuple[int, int]:
    """Map world ``(x, y)`` to this project's ``(row, column)`` convention."""

    world = _finite_vector(world_xy, name="world_xy", length=2)
    origin = _finite_vector(origin_xy, name="origin_xy", length=2)
    resolution = _positive_float(resolution_m, name="resolution_m")
    indices = np.floor((world - origin) / resolution + 0.5).astype(np.int64)
    return int(indices[0]), int(indices[1])


def yaw_to_bin(yaw_rad: float) -> int:
    """Quantize yaw to the nearest of 72 cyclic bins."""

    yaw = float(yaw_rad)
    if not math.isfinite(yaw):
        raise ValueError("yaw_rad must be finite")
    normalized = yaw % (2.0 * math.pi)
    return int(math.floor(normalized / YAW_BIN_WIDTH_RAD + 0.5)) % YAW_BIN_COUNT


def _normalized_yaw(yaw_rad: float) -> float:
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi


def sample_motion_primitive(
    *,
    start_pose: object,
    signed_arc_length_m: float,
    curvature_1_m: float,
    sample_spacing_m: float,
) -> np.ndarray:
    """Sample a constant-curvature arc including its start and endpoint."""

    start = _finite_vector(start_pose, name="start_pose", length=3)
    arc_length = float(signed_arc_length_m)
    curvature = float(curvature_1_m)
    spacing = _positive_float(sample_spacing_m, name="sample_spacing_m")
    if not np.isfinite([arc_length, curvature]).all() or arc_length == 0.0:
        raise ValueError("arc length must be finite and non-zero; curvature must be finite")
    sample_count = max(1, int(math.ceil(abs(arc_length) / spacing)))
    distances = np.linspace(0.0, arc_length, sample_count + 1)
    x0, y0, yaw0 = start
    if abs(curvature) <= 1e-12:
        x = x0 + distances * math.cos(yaw0)
        y = y0 + distances * math.sin(yaw0)
        yaw = np.full_like(distances, yaw0)
    else:
        yaw = yaw0 + curvature * distances
        x = x0 + (np.sin(yaw) - math.sin(yaw0)) / curvature
        y = y0 + (-np.cos(yaw) + math.cos(yaw0)) / curvature
    return np.column_stack((x, y, np.vectorize(_normalized_yaw)(yaw))).astype(
        np.float64
    )


def primitive_is_collision_free(
    samples: object,
    *,
    occupancy_grid: object,
    origin_xy: object,
    resolution_m: float,
    occupied: int | bool = 1,
) -> bool:
    """Return false when any sampled center is outside or occupied."""

    path = np.asarray(samples, dtype=np.float64)
    grid = np.asarray(occupancy_grid)
    if path.ndim != 2 or path.shape[1] != 3 or path.shape[0] == 0:
        raise ValueError("samples must have shape (N, 3)")
    if not np.isfinite(path).all():
        raise ValueError("samples must be finite")
    if grid.ndim != 2 or 0 in grid.shape:
        raise ValueError("occupancy_grid must be a non-empty 2D array")
    if np.issubdtype(grid.dtype, np.number) and not np.isfinite(grid).all():
        raise ValueError("occupancy_grid must be finite")
    origin = _finite_vector(origin_xy, name="origin_xy", length=2)
    resolution = _positive_float(resolution_m, name="resolution_m")
    cells = np.floor((path[:, :2] - origin) / resolution + 0.5).astype(np.int64)
    inside = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < grid.shape[0])
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < grid.shape[1])
    )
    if not bool(np.all(inside)):
        return False
    return not bool(np.any(grid[cells[:, 0], cells[:, 1]] == occupied))


def _grid_distance_to_goal(
    occupancy_grid: object,
    *,
    goal_cell: tuple[int, int],
    resolution_m: float,
    occupied: int | bool,
) -> np.ndarray:
    """Compute an obstacle-aware holonomic cost-to-go field for Hybrid A*."""

    grid = np.asarray(occupancy_grid)
    resolution = _positive_float(resolution_m, name="resolution_m")
    goal_row, goal_column = goal_cell
    if (
        grid.ndim != 2
        or not 0 <= goal_row < grid.shape[0]
        or not 0 <= goal_column < grid.shape[1]
    ):
        raise ValueError("goal_cell must lie inside a non-empty 2D grid")
    distance = np.full(grid.shape, np.inf, dtype=np.float64)
    if grid[goal_row, goal_column] == occupied:
        return distance
    distance[goal_row, goal_column] = 0.0
    frontier: list[tuple[float, int, int]] = [(0.0, goal_row, goal_column)]
    while frontier:
        cost, row, column = heapq.heappop(frontier)
        if cost > distance[row, column] + 1e-12:
            continue
        for delta_row, delta_column, multiplier in _GRID_NEIGHBORS:
            next_row = row + delta_row
            next_column = column + delta_column
            if (
                next_row < 0
                or next_row >= grid.shape[0]
                or next_column < 0
                or next_column >= grid.shape[1]
                or grid[next_row, next_column] == occupied
            ):
                continue
            candidate = cost + multiplier * resolution
            if candidate + 1e-12 < distance[next_row, next_column]:
                distance[next_row, next_column] = candidate
                heapq.heappush(frontier, (candidate, next_row, next_column))
    return distance


def _transform_local_primitive(
    local_samples: np.ndarray,
    *,
    x_m: float,
    y_m: float,
    yaw_rad: float,
) -> np.ndarray:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    samples = np.empty_like(local_samples)
    samples[:, 0] = x_m + cosine * local_samples[:, 0] - sine * local_samples[:, 1]
    samples[:, 1] = y_m + sine * local_samples[:, 0] + cosine * local_samples[:, 1]
    samples[:, 2] = (
        local_samples[:, 2] + yaw_rad + math.pi
    ) % (2.0 * math.pi) - math.pi
    return samples


def _primitive_is_collision_free_unchecked(
    samples: np.ndarray,
    *,
    grid: np.ndarray,
    origin_xy: np.ndarray,
    resolution_m: float,
    occupied: int | bool,
) -> bool:
    cells = np.floor((samples[:, :2] - origin_xy) / resolution_m + 0.5).astype(
        np.int64
    )
    inside = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < grid.shape[0])
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < grid.shape[1])
    )
    return bool(np.all(inside)) and not bool(
        np.any(grid[cells[:, 0], cells[:, 1]] == occupied)
    )


@dataclass(frozen=True)
class KinodynamicPlanRequest:
    occupancy_grid: np.ndarray
    resolution_m: float
    origin_xy: tuple[float, float]
    start_xy: tuple[float, float]
    start_yaw_rad: float
    goal_xy: tuple[float, float]
    primitive_length_m: float = 0.3
    collision_sample_spacing_m: float = 0.05
    reverse_cost_multiplier: float = 1.25
    direction_change_cost_m: float = 0.5
    goal_tolerance_m: float = 0.2
    node_budget: int = 250_000
    occupied: int | bool = 1

    def __post_init__(self) -> None:
        grid = np.asarray(self.occupancy_grid)
        if grid.ndim != 2 or 0 in grid.shape:
            raise ValueError("occupancy_grid must be a non-empty 2D array")
        if np.issubdtype(grid.dtype, np.number) and not np.isfinite(grid).all():
            raise ValueError("occupancy_grid must be finite")
        immutable_grid = grid.copy()
        immutable_grid.setflags(write=False)
        origin = _finite_vector(self.origin_xy, name="origin_xy", length=2)
        start = _finite_vector(self.start_xy, name="start_xy", length=2)
        goal = _finite_vector(self.goal_xy, name="goal_xy", length=2)
        yaw = float(self.start_yaw_rad)
        if not math.isfinite(yaw):
            raise ValueError("start_yaw_rad must be finite")
        if isinstance(self.node_budget, bool) or not isinstance(
            self.node_budget, (int, np.integer)
        ) or self.node_budget <= 0:
            raise ValueError("node_budget must be a positive integer")
        object.__setattr__(self, "occupancy_grid", immutable_grid)
        object.__setattr__(self, "resolution_m", _positive_float(self.resolution_m, name="resolution_m"))
        object.__setattr__(self, "origin_xy", (float(origin[0]), float(origin[1])))
        object.__setattr__(self, "start_xy", (float(start[0]), float(start[1])))
        object.__setattr__(self, "goal_xy", (float(goal[0]), float(goal[1])))
        object.__setattr__(self, "start_yaw_rad", _normalized_yaw(yaw))
        object.__setattr__(
            self, "primitive_length_m", _positive_float(self.primitive_length_m, name="primitive_length_m")
        )
        object.__setattr__(
            self,
            "collision_sample_spacing_m",
            _positive_float(self.collision_sample_spacing_m, name="collision_sample_spacing_m"),
        )
        if self.collision_sample_spacing_m > 0.05 + 1e-12:
            raise ValueError("collision_sample_spacing_m must not exceed 0.05")
        object.__setattr__(
            self,
            "reverse_cost_multiplier",
            _positive_float(self.reverse_cost_multiplier, name="reverse_cost_multiplier"),
        )
        object.__setattr__(
            self,
            "direction_change_cost_m",
            _positive_float(self.direction_change_cost_m, name="direction_change_cost_m", allow_zero=True),
        )
        object.__setattr__(
            self,
            "goal_tolerance_m",
            _positive_float(self.goal_tolerance_m, name="goal_tolerance_m"),
        )
        object.__setattr__(self, "node_budget", int(self.node_budget))


@dataclass(frozen=True)
class KinodynamicWaypoint:
    x_m: float
    y_m: float
    yaw_rad: float
    drive_direction: DriveDirection
    main_command_rad_s: float
    auxiliary_angle_rad: float


@dataclass(frozen=True)
class KinodynamicPlanResult:
    status: PlanStatus
    waypoints: tuple[KinodynamicWaypoint, ...]
    expanded_nodes: int
    path_length_m: float
    direction_changes: int


StateKey = tuple[int, int, int, DriveDirection, int]


def _valid_cell(request: KinodynamicPlanRequest, xy: tuple[float, float]) -> bool:
    row, column = world_to_cell(
        xy, origin_xy=request.origin_xy, resolution_m=request.resolution_m
    )
    return (
        0 <= row < request.occupancy_grid.shape[0]
        and 0 <= column < request.occupancy_grid.shape[1]
        and request.occupancy_grid[row, column] != request.occupied
    )


def _reconstruct(
    goal_key: StateKey,
    *,
    parents: dict[StateKey, StateKey | None],
    poses: dict[StateKey, tuple[float, float, float]],
    incoming_actions: dict[StateKey, tuple[float, float]],
    primitive_length_m: float,
    expanded_nodes: int,
) -> KinodynamicPlanResult:
    keys = [goal_key]
    while parents[keys[-1]] is not None:
        keys.append(parents[keys[-1]])  # type: ignore[arg-type]
    keys.reverse()
    actions = [incoming_actions[key] for key in keys[1:]]
    waypoints: list[KinodynamicWaypoint] = []
    for index, key in enumerate(keys):
        if index < len(actions):
            main, auxiliary = actions[index]
            direction: DriveDirection = "forward" if main > 0.0 else "reverse"
        else:
            main, auxiliary = 0.0, 0.0
            direction = key[3]
        x, y, yaw = poses[key]
        waypoints.append(
            KinodynamicWaypoint(x, y, yaw, direction, main, auxiliary)
        )
    directions = ["forward" if main > 0.0 else "reverse" for main, _ in actions]
    changes = sum(left != right for left, right in zip(directions, directions[1:]))
    return KinodynamicPlanResult(
        status="success",
        waypoints=tuple(waypoints),
        expanded_nodes=expanded_nodes,
        path_length_m=len(actions) * primitive_length_m,
        direction_changes=changes,
    )


def plan_kinodynamic_path(
    request: KinodynamicPlanRequest, drive_model: DriveModel, *,
    initial_direction: DriveDirection | None = None,
    first_primitive_direction: DriveDirection | None = None,
    primitive_curvature_source=None,
    primitive_requests=None,
) -> KinodynamicPlanResult:
    """Search measured forward/reverse primitives on an inflated occupancy map."""

    if first_primitive_direction not in (None, "forward", "reverse"):
        raise ValueError("first_primitive_direction must be None, forward or reverse")
    if initial_direction not in (None, "forward", "reverse"):
        raise ValueError("initial_direction must be None, forward or reverse")
    if not isinstance(request, KinodynamicPlanRequest):
        raise TypeError("request must be a KinodynamicPlanRequest")
    if not isinstance(drive_model, DriveModel):
        raise TypeError("drive_model must be a DriveModel")
    if not _valid_cell(request, request.start_xy):
        return KinodynamicPlanResult("invalid_start", (), 0, 0.0, 0)
    if not _valid_cell(request, request.goal_xy):
        return KinodynamicPlanResult("invalid_goal", (), 0, 0.0, 0)

    start_cell = world_to_cell(
        request.start_xy,
        origin_xy=request.origin_xy,
        resolution_m=request.resolution_m,
    )
    start_yaw_bin = yaw_to_bin(request.start_yaw_rad)
    goal_cell = world_to_cell(
        request.goal_xy,
        origin_xy=request.origin_xy,
        resolution_m=request.resolution_m,
    )
    grid_distance = _grid_distance_to_goal(
        request.occupancy_grid,
        goal_cell=goal_cell,
        resolution_m=request.resolution_m,
        occupied=request.occupied,
    )

    def heuristic(x: float, y: float) -> float:
        row = int(math.floor((x - request.origin_xy[0]) / request.resolution_m + 0.5))
        column = int(math.floor((y - request.origin_xy[1]) / request.resolution_m + 0.5))
        if 0 <= row < grid_distance.shape[0] and 0 <= column < grid_distance.shape[1]:
            obstacle_aware = float(grid_distance[row, column])
            if math.isfinite(obstacle_aware):
                return max(0.0, obstacle_aware - request.goal_tolerance_m)
        return max(
            0.0,
            math.hypot(x - request.goal_xy[0], y - request.goal_xy[1])
            - request.goal_tolerance_m,
        )

    primitive_templates: list[
        tuple[DriveDirection, float, float, np.ndarray]
    ] = []
    if primitive_requests is None:
        commands = tuple((sign*magnitude, auxiliary) for sign in (1.,-1.) for magnitude in MAIN_MAGNITUDES_RAD_S for auxiliary in AUXILIARY_ANGLES_RAD)
    else:
        if primitive_curvature_source is None:
            raise ValueError('explicit supported commands require an explicit curvature source')
        commands = tuple(primitive_requests)
        if not commands or any(len(c)!=2 or not all(math.isfinite(v) for v in c) or c[0]==0 for c in commands):
            raise ValueError('finite nonzero supported primitive requests required')
    for main, auxiliary in commands:
        sign = 1. if main > 0 else -1.
        direction = "forward" if sign > 0 else "reverse"
        curvature = drive_model.curvature(main, auxiliary) if primitive_curvature_source is None else primitive_curvature_source(main, auxiliary)
        if curvature is None:
            continue
        if not math.isfinite(curvature):
            raise ValueError('finite supported primitive curvature')
        primitive_templates.append((direction, main, auxiliary, sample_motion_primitive(
            start_pose=(0.0, 0.0, 0.0), signed_arc_length_m=sign*request.primitive_length_m,
            curvature_1_m=curvature, sample_spacing_m=request.collision_sample_spacing_m)))

    # Discrete initial-prefix phase is part of search identity, not a timed control wait.
    # Frozen .3m primitives require TWO to exceed the inherited .372076m coast window.
    if first_primitive_direction is not None and abs(request.primitive_length_m-.3)>1e-12:
        raise ValueError('minimum initial run only frozen .3m primitive domain')
    required_prefix = 2 if first_primitive_direction is not None else 0
    tie = itertools.count()
    frontier: list[tuple[float, int, StateKey]] = []
    costs: dict[StateKey, float] = {}
    parents: dict[StateKey, StateKey | None] = {}
    poses: dict[StateKey, tuple[float, float, float]] = {}
    incoming_actions: dict[StateKey, tuple[float, float]] = {}
    for direction in ("forward", "reverse"):
        if first_primitive_direction is not None and direction != first_primitive_direction:
            continue
        key: StateKey = (*start_cell, start_yaw_bin, direction, 0)
        costs[key] = (request.direction_change_cost_m
                      if initial_direction is not None and direction != initial_direction
                      else 0.0)
        parents[key] = None
        poses[key] = (*request.start_xy, request.start_yaw_rad)
        heapq.heappush(
            frontier,
            (costs[key] + heuristic(*request.start_xy), next(tie), key),
        )

    closed: set[StateKey] = set()
    origin_array = np.asarray(request.origin_xy, dtype=np.float64)
    expanded = 0
    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if expanded >= request.node_budget:
            return KinodynamicPlanResult(
                "node_budget_exhausted", (), expanded, 0.0, 0
            )
        closed.add(current)
        expanded += 1
        x, y, yaw = poses[current]
        if math.hypot(x - request.goal_xy[0], y - request.goal_xy[1]) <= (
            request.goal_tolerance_m + 1e-12
        ):
            return _reconstruct(
                current,
                parents=parents,
                poses=poses,
                incoming_actions=incoming_actions,
                primitive_length_m=request.primitive_length_m,
                expanded_nodes=expanded,
            )

        for direction, main, auxiliary, local_samples in primitive_templates:
            # Only the initial two-expansion phase is restricted; later opposite actions remain legal.
            if current[4] < required_prefix and direction != first_primitive_direction:
                continue
            samples = _transform_local_primitive(
                local_samples, x_m=x, y_m=y, yaw_rad=yaw
            )
            if not _primitive_is_collision_free_unchecked(
                samples,
                grid=request.occupancy_grid,
                origin_xy=origin_array,
                resolution_m=request.resolution_m,
                occupied=request.occupied,
            ):
                continue
            next_x, next_y, next_yaw = (float(value) for value in samples[-1])
            row, column = world_to_cell(
                (next_x, next_y),
                origin_xy=request.origin_xy,
                resolution_m=request.resolution_m,
            )
            neighbor: StateKey = (
                row,
                column,
                yaw_to_bin(next_yaw),
                direction,
                min(required_prefix, current[4]+1),
            )
            edge_cost = request.primitive_length_m
            if direction == "reverse":
                edge_cost *= request.reverse_cost_multiplier
            if direction != current[3]:
                edge_cost += request.direction_change_cost_m
            candidate = costs[current] + edge_cost
            if candidate + 1e-12 >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = candidate
            parents[neighbor] = current
            poses[neighbor] = (next_x, next_y, next_yaw)
            incoming_actions[neighbor] = (main, auxiliary)
            heapq.heappush(
                frontier,
                (candidate + heuristic(next_x, next_y), next(tie), neighbor),
            )

    return KinodynamicPlanResult("unreachable", (), expanded, 0.0, 0)
