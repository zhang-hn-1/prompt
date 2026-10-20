"""Pure NumPy dynamics for rate-limited lattice motion primitives."""

from __future__ import annotations

from dataclasses import dataclass, replace
import heapq
import itertools
import math
import time
from typing import Callable, Literal

import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter

from .drive_model import DriveModel
from .kinodynamic_planning import (
    _finite_vector,
    _grid_distance_to_goal,
    _positive_float,
    world_to_cell,
    yaw_to_bin,
)


DriveDirection = Literal["forward", "reverse", "stopped"]
MAIN_TARGETS_RAD_S = (1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5)
AUXILIARY_TARGETS_RAD = tuple(
    math.radians(value) for value in (0.0, -10.0, 10.0, -20.0, 20.0, -30.0, 30.0)
)


@dataclass(frozen=True)
class DynamicState:
    x_m: float
    y_m: float
    yaw_rad: float
    main_command_rad_s: float
    auxiliary_angle_rad: float
    speed_m_s: float
    elapsed_s: float
    drive_direction: DriveDirection
    stopped_time_s: float


@dataclass(frozen=True)
class DynamicControlTarget:
    main_command_rad_s: float
    auxiliary_angle_rad: float


@dataclass(frozen=True)
class DynamicPrimitive:
    target: DynamicControlTarget
    samples: np.ndarray
    end: DynamicState
    path_length_m: float


def dynamic_state_key(
    state: DynamicState, *, origin_xy: object, resolution_m: float
) -> tuple[int, int, int, int, int, int, DriveDirection]:
    """Quantize only the lookup key; never replace the continuous payload."""
    return (
        *world_to_cell((state.x_m, state.y_m), origin_xy=origin_xy, resolution_m=resolution_m),
        yaw_to_bin(state.yaw_rad),
        int(math.floor(state.main_command_rad_s / 0.25 + 0.5)),
        int(math.floor(state.auxiliary_angle_rad / math.radians(5.0) + 0.5)),
        int(math.floor(state.speed_m_s / 0.05 + 0.5)),
        state.drive_direction,
    )


def obstacle_distance_field(
    occupancy_grid: np.ndarray, *, resolution_m: float, occupied: int | bool = 1
) -> np.ndarray:
    """Distance at cell centers to the inflated occupied-cell boundaries."""
    mask = np.asarray(occupancy_grid) == occupied
    if not np.any(mask):
        return np.full(mask.shape, np.inf)
    # A half-cell lattice represents each occupied square's edges and corners,
    # giving exact Euclidean boundary distances at original cell centers.
    boundary = np.zeros((2 * mask.shape[0] + 1, 2 * mask.shape[1] + 1), dtype=bool)
    boundary[1::2, 1::2] = mask
    boundary = maximum_filter(boundary, size=3, mode="constant")
    return distance_transform_edt(~boundary)[1::2, 1::2] * (resolution_m / 2.0)


def _guided_remaining_field(
    guidance_cells: np.ndarray,
    *,
    grid_shape: tuple[int, int],
    resolution_m: float,
) -> np.ndarray:
    """Return each cell's remaining guidance-path distance via its nearest cell."""
    path_mask = np.zeros(grid_shape, dtype=bool)
    path_mask[guidance_cells[:, 0], guidance_cells[:, 1]] = True
    _, nearest_cells = distance_transform_edt(~path_mask, return_indices=True)

    step_lengths_m = (
        np.hypot(*np.diff(guidance_cells, axis=0).T) * resolution_m
    )
    remaining_m = np.empty(len(guidance_cells), dtype=np.float64)
    remaining_m[-1] = 0.0
    remaining_m[:-1] = np.cumsum(step_lengths_m[::-1])[::-1]

    path_indices = np.empty(grid_shape, dtype=np.intp)
    path_indices[guidance_cells[:, 0], guidance_cells[:, 1]] = np.arange(
        len(guidance_cells)
    )
    return remaining_m[path_indices[nearest_cells[0], nearest_cells[1]]]


def _trusted_edge_position_cells(
    positions_xy: np.ndarray, *, origin_xy: object, resolution_m: float
) -> np.ndarray:
    """Convert planner-owned finite positions to grid cells without validation."""
    return np.floor(
        (positions_xy - np.asarray(origin_xy)) / resolution_m + 0.5
    ).astype(int)


def _trusted_edge_is_collision_free(
    cells: np.ndarray,
    *,
    occupancy_grid: np.ndarray,
    occupied: int | bool,
) -> bool:
    """Evaluate already-converted planner cells against a trusted grid."""
    inside = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < occupancy_grid.shape[0])
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < occupancy_grid.shape[1])
    )
    return bool(np.all(inside)) and not bool(
        np.any(occupancy_grid[cells[:, 0], cells[:, 1]] == occupied)
    )


def _trusted_dynamic_primitive_cost(
    primitive: DynamicPrimitive,
    *,
    previous_target: DynamicControlTarget,
    previous_direction: DriveDirection,
    distance_field: np.ndarray,
    cells: np.ndarray,
) -> float:
    """Calculate cost from trusted cells aligned to samples after the start."""
    samples = primitive.samples
    clearance = distance_field[cells[:, 0], cells[:, 1]]
    intervals = np.diff(samples[:, 0])
    reverse_length = float(np.sum(samples[1:, 6] * intervals * (samples[1:, 4] < 0.0)))
    moving = samples[1:, 4][samples[1:, 4] != 0.0]
    direction = ("forward" if moving[0] > 0 else "reverse") if len(moving) else "stopped"
    changed = (
        direction != "stopped"
        and previous_direction != "stopped"
        and direction != previous_direction
    )
    return (
        primitive.path_length_m + 0.25 * reverse_length
        + 0.05 * float(samples[-1, 0] - samples[0, 0])
        + 0.5 * changed
        + 0.10 * abs(primitive.target.main_command_rad_s - previous_target.main_command_rad_s)
        + 0.10
        * abs(
            primitive.target.auxiliary_angle_rad
            - previous_target.auxiliary_angle_rad
        )
        / math.radians(30.0)
        + float(np.sum(0.02 / (clearance + 0.05) * intervals))
    )


def dynamic_primitive_cost(
    primitive: DynamicPrimitive,
    *,
    previous_target: DynamicControlTarget,
    previous_direction: DriveDirection,
    distance_field: np.ndarray,
    origin_xy: object,
    resolution_m: float,
) -> float:
    """Integrate distance, elapsed time, reversal, target changes, and risk."""
    samples = primitive.samples
    return _trusted_dynamic_primitive_cost(
        primitive,
        previous_target=previous_target,
        previous_direction=previous_direction,
        distance_field=distance_field,
        cells=_trusted_edge_position_cells(
            samples[1:, 1:3], origin_xy=origin_xy, resolution_m=resolution_m
        ),
    )


def valid_control_targets(state: DynamicState) -> tuple[DynamicControlTarget, ...]:
    """Return deterministic targets that obey the stopped reversal gate."""

    if state.main_command_rad_s > 0.0:
        main_targets = MAIN_TARGETS_RAD_S[:4]
    elif state.main_command_rad_s < 0.0:
        main_targets = MAIN_TARGETS_RAD_S[3:]
    elif state.stopped_time_s >= 0.5:
        main_targets = MAIN_TARGETS_RAD_S
    else:
        main_targets = (0.0,)
    return tuple(
        DynamicControlTarget(main_command_rad_s, auxiliary_angle_rad)
        for main_command_rad_s in main_targets
        for auxiliary_angle_rad in AUXILIARY_TARGETS_RAD
    )


def dynamic_primitive_is_collision_free(
    primitive: DynamicPrimitive,
    *,
    occupancy_grid: object,
    origin_xy: object,
    resolution_m: float,
    occupied: int | bool = 1,
) -> bool:
    """Return false if any sampled center is outside or occupied."""

    samples = np.asarray(primitive.samples, dtype=np.float64)
    grid = np.asarray(occupancy_grid)
    origin = np.asarray(origin_xy, dtype=np.float64)
    resolution = float(resolution_m)
    if samples.ndim != 2 or samples.shape[0] == 0 or samples.shape[1] != 8:
        raise ValueError("primitive samples must have shape (N, 8)")
    if not np.isfinite(samples).all():
        raise ValueError("primitive samples must be finite")
    if grid.ndim != 2 or 0 in grid.shape:
        raise ValueError("occupancy_grid must be a non-empty 2D array")
    if np.issubdtype(grid.dtype, np.number) and not np.isfinite(grid).all():
        raise ValueError("occupancy_grid must be finite")
    if origin.shape != (2,) or not np.isfinite(origin).all():
        raise ValueError("origin_xy must contain two finite values")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolution_m must be finite and positive")

    return _trusted_edge_is_collision_free(
        _trusted_edge_position_cells(
            samples[:, 1:3], origin_xy=origin, resolution_m=resolution
        ),
        occupancy_grid=grid,
        occupied=occupied,
    )


def _move_toward(value: float, target: float, maximum_delta: float) -> float:
    return value + max(-maximum_delta, min(maximum_delta, target - value))


def _normalized_yaw(yaw_rad: float) -> float:
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi


def propagate_dynamic_primitive(
    *,
    start: DynamicState,
    target: DynamicControlTarget,
    drive_model: DriveModel,
) -> DynamicPrimitive:
    """Propagate one second of continuous controls at 50 Hz."""

    duration_s = 1.0
    dt_s = 0.02
    main_acceleration_limit_rad_s2 = 0.5
    auxiliary_rate_limit_rad_s = math.radians(30.0)
    step_count = int(round(duration_s / dt_s))

    samples = np.empty((step_count + 1, 8), dtype=np.float64)
    samples[0] = (
        start.elapsed_s,
        start.x_m,
        start.y_m,
        start.yaw_rad,
        start.main_command_rad_s,
        start.auxiliary_angle_rad,
        start.speed_m_s,
        start.stopped_time_s,
    )

    x_m = start.x_m
    y_m = start.y_m
    yaw_rad = start.yaw_rad
    main_command_rad_s = start.main_command_rad_s
    auxiliary_angle_rad = start.auxiliary_angle_rad
    stopped_time_s = start.stopped_time_s
    path_length_m = 0.0
    speed_m_s = start.speed_m_s
    drive_direction = start.drive_direction

    for step in range(1, step_count + 1):
        main_command_rad_s = _move_toward(
            main_command_rad_s,
            target.main_command_rad_s,
            main_acceleration_limit_rad_s2 * dt_s,
        )
        auxiliary_angle_rad = _move_toward(
            auxiliary_angle_rad,
            target.auxiliary_angle_rad,
            auxiliary_rate_limit_rad_s * dt_s,
        )

        if abs(main_command_rad_s) <= 1e-12:
            main_command_rad_s = 0.0
            speed_m_s = 0.0
            curvature_1_m = 0.0
            drive_direction = "stopped"
            stopped_time_s += dt_s
        else:
            speed_m_s = drive_model.speed(
                main_command_rad_s, auxiliary_angle_rad
            )
            curvature_1_m = drive_model.curvature(
                main_command_rad_s, auxiliary_angle_rad
            )
            drive_direction = "forward" if main_command_rad_s > 0.0 else "reverse"
            stopped_time_s = 0.0

        signed_speed_m_s = speed_m_s if main_command_rad_s >= 0.0 else -speed_m_s
        x_m += signed_speed_m_s * math.cos(yaw_rad) * dt_s
        y_m += signed_speed_m_s * math.sin(yaw_rad) * dt_s
        yaw_rad = _normalized_yaw(
            yaw_rad + signed_speed_m_s * curvature_1_m * dt_s
        )
        path_length_m += speed_m_s * dt_s
        elapsed_s = start.elapsed_s + step * dt_s
        samples[step] = (
            elapsed_s,
            x_m,
            y_m,
            yaw_rad,
            main_command_rad_s,
            auxiliary_angle_rad,
            speed_m_s,
            stopped_time_s,
        )

    end = DynamicState(
        x_m=x_m,
        y_m=y_m,
        yaw_rad=yaw_rad,
        main_command_rad_s=main_command_rad_s,
        auxiliary_angle_rad=auxiliary_angle_rad,
        speed_m_s=speed_m_s,
        elapsed_s=start.elapsed_s + duration_s,
        drive_direction=drive_direction,
        stopped_time_s=stopped_time_s,
    )
    return DynamicPrimitive(
        target=target,
        samples=samples,
        end=end,
        path_length_m=path_length_m,
    )


@dataclass(frozen=True)
class DynamicPlanRequest:
    occupancy_grid: np.ndarray
    resolution_m: float
    origin_xy: tuple[float, float]
    initial_state: DynamicState
    goal_xy: tuple[float, float]
    node_budget: int = 500_000
    planning_timeout_s: float = 60.0
    occupied: int | bool = 1
    guidance_cells: np.ndarray | None = None
    guidance_weight: float = 0.35

    def __post_init__(self) -> None:
        grid = np.asarray(self.occupancy_grid)
        if grid.ndim != 2 or 0 in grid.shape:
            raise ValueError("occupancy_grid must be a non-empty 2D array")
        if grid.dtype.kind not in "biuf" or not np.isfinite(grid).all():
            raise ValueError("occupancy_grid must be finite numeric or boolean data")
        grid = grid.copy()
        grid.setflags(write=False)
        object.__setattr__(self, "occupancy_grid", grid)
        for name in ("origin_xy", "goal_xy"):
            value = _finite_vector(getattr(self, name), name=name, length=2)
            object.__setattr__(self, name, tuple(float(v) for v in value))
        for name in ("resolution_m", "planning_timeout_s"):
            object.__setattr__(
                self, name, _positive_float(getattr(self, name), name=name)
            )
        object.__setattr__(
            self,
            "guidance_weight",
            _positive_float(self.guidance_weight, name="guidance_weight"),
        )
        if self.guidance_cells is not None:
            guidance_cells = np.asarray(self.guidance_cells)
            if (
                guidance_cells.ndim != 2
                or guidance_cells.shape[0] == 0
                or guidance_cells.shape[1] != 2
                or not np.issubdtype(guidance_cells.dtype, np.integer)
            ):
                raise ValueError("guidance_cells must be a non-empty (K, 2) integer array")
            if (
                np.any(guidance_cells[:, 0] < 0)
                or np.any(guidance_cells[:, 0] >= grid.shape[0])
                or np.any(guidance_cells[:, 1] < 0)
                or np.any(guidance_cells[:, 1] >= grid.shape[1])
            ):
                raise ValueError("guidance_cells must be within occupancy_grid bounds")
            if np.any(grid[guidance_cells[:, 0], guidance_cells[:, 1]] == self.occupied):
                raise ValueError("guidance_cells must be free")
            goal_cell = world_to_cell(
                self.goal_xy,
                origin_xy=self.origin_xy,
                resolution_m=self.resolution_m,
            )
            if tuple(guidance_cells[-1]) != goal_cell:
                raise ValueError("guidance_cells must end at the goal cell")
            guidance_cells = guidance_cells.copy()
            guidance_cells.setflags(write=False)
            object.__setattr__(self, "guidance_cells", guidance_cells)
        if (
            isinstance(self.node_budget, bool)
            or not isinstance(self.node_budget, (int, np.integer))
            or self.node_budget <= 0
        ):
            raise ValueError("node_budget must be a positive integer")
        state = self.initial_state
        if not isinstance(state, DynamicState):
            raise ValueError("initial_state must be a DynamicState")
        values = (
            state.x_m,
            state.y_m,
            state.yaw_rad,
            state.main_command_rad_s,
            state.auxiliary_angle_rad,
            state.speed_m_s,
            state.elapsed_s,
            state.stopped_time_s,
        )
        if not np.isfinite(values).all():
            raise ValueError("initial_state must be finite")
        if min(state.speed_m_s, state.elapsed_s, state.stopped_time_s) < 0:
            raise ValueError("initial speed and times must be non-negative")
        if (
            abs(state.main_command_rad_s) > 1.5
            or abs(state.auxiliary_angle_rad) > math.radians(30) + 1e-12
        ):
            raise ValueError("initial controls exceed the accepted operating range")
        if state.main_command_rad_s > 0:
            expected_direction: DriveDirection = "forward"
        elif state.main_command_rad_s < 0:
            expected_direction = "reverse"
        else:
            expected_direction = "stopped"
        if state.drive_direction != expected_direction:
            raise ValueError("initial drive_direction must match the main command")


@dataclass(frozen=True)
class DynamicWaypoint:
    x_m: float
    y_m: float
    yaw_rad: float
    main_command_rad_s: float
    auxiliary_angle_rad: float
    speed_m_s: float
    time_from_start_s: float
    drive_direction: DriveDirection


@dataclass(frozen=True)
class DynamicPlanResult:
    status: Literal[
        "success",
        "invalid_start",
        "invalid_goal",
        "unreachable",
        "node_budget_exhausted",
        "timeout",
    ]
    waypoints: tuple[DynamicWaypoint, ...] = ()
    controls: tuple[DynamicControlTarget, ...] = ()
    expanded_nodes: int = 0
    path_length_m: float = 0.0
    duration_s: float = 0.0
    direction_changes: int = 0
    total_cost: float = 0.0


@dataclass(frozen=True)
class _SearchNode:
    state: DynamicState
    cost: float
    parent: int | None
    primitive: DynamicPrimitive | None
    last_direction: DriveDirection
    previous_target: DynamicControlTarget
    reference_index: int | None = None
    source: Literal["root", "reference_rollout", "lattice"] = "root"


@dataclass(frozen=True)
class _DynamicSuccessor:
    """A collision-checked successor produced by the shared primitive loop."""

    end: DynamicState
    primitive: DynamicPrimitive
    direction: DriveDirection
    edge_cost: float


@dataclass(frozen=True)
class _ReferenceRollout:
    nodes: tuple[_SearchNode, ...]
    seed_indices: tuple[int, ...]
    goal_index: int | None
    expanded_nodes: int


class _PlanningDeadlineExceeded(RuntimeError):
    """Internal signal used to preserve planner timeout semantics."""


def _reference_progress_fields(
    request: DynamicPlanRequest,
) -> tuple[np.ndarray, np.ndarray] | None:
    """Return metric remaining and lateral fields for an immutable reference path."""
    if request.guidance_cells is None:
        return None
    path = request.guidance_cells
    remaining = _guided_remaining_field(
        path,
        grid_shape=request.occupancy_grid.shape,
        resolution_m=request.resolution_m,
    )
    path_mask = np.zeros(request.occupancy_grid.shape, dtype=bool)
    path_mask[path[:, 0], path[:, 1]] = True
    lateral = distance_transform_edt(~path_mask) * request.resolution_m
    # Off-route cells inherit the nearest route progress plus their metric
    # offset, avoiding a zero-valued plateau beyond the route endpoint.
    return remaining + lateral, lateral


def _iter_valid_successors(
    node: _SearchNode,
    *,
    request: DynamicPlanRequest,
    drive_model: DriveModel,
    clearance: np.ndarray,
    templates: dict[tuple, DynamicPrimitive],
    deadline_check: Callable[[], bool] | None = None,
):
    """Yield all physically valid one-second successors in stable action order."""
    state = node.state
    grid = request.occupancy_grid
    origin = np.asarray(request.origin_xy)
    quantized_main = math.floor(state.main_command_rad_s / 0.25 + 0.5) * 0.25
    quantized_aux = (
        math.floor(state.auxiliary_angle_rad / math.radians(5) + 0.5)
        * math.radians(5)
    )
    for target in valid_control_targets(state):
        if deadline_check is not None and deadline_check():
            raise _PlanningDeadlineExceeded
        if (
            abs(target.main_command_rad_s - quantized_main) > 0.5 + 1e-12
            or abs(target.auxiliary_angle_rad - quantized_aux)
            > math.radians(30) + 1e-12
        ):
            continue
        local_start = replace(
            state, x_m=0.0, y_m=0.0, yaw_rad=0.0, elapsed_s=0.0
        )
        cache_key = (local_start, target)
        if cache_key not in templates:
            if deadline_check is not None and deadline_check():
                raise _PlanningDeadlineExceeded
            templates[cache_key] = propagate_dynamic_primitive(
                start=local_start,
                target=target,
                drive_model=drive_model,
            )
        local = templates[cache_key]
        samples = local.samples.copy()
        cosine, sine = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
        samples[:, 1] = state.x_m + cosine * local.samples[:, 1] - sine * local.samples[:, 2]
        samples[:, 2] = state.y_m + sine * local.samples[:, 1] + cosine * local.samples[:, 2]
        samples[:, 3] = (
            local.samples[:, 3] + state.yaw_rad + math.pi
        ) % (2 * math.pi) - math.pi
        samples[:, 0] += state.elapsed_s
        end = replace(
            local.end,
            x_m=float(samples[-1, 1]),
            y_m=float(samples[-1, 2]),
            yaw_rad=float(samples[-1, 3]),
            elapsed_s=state.elapsed_s + 1.0,
        )
        primitive = DynamicPrimitive(target, samples, end, local.path_length_m)
        if deadline_check is not None and deadline_check():
            raise _PlanningDeadlineExceeded
        cells = _trusted_edge_position_cells(
            samples[:, 1:3],
            origin_xy=origin,
            resolution_m=request.resolution_m,
        )
        collision_free = _trusted_edge_is_collision_free(
            cells, occupancy_grid=grid, occupied=request.occupied
        )
        if deadline_check is not None and deadline_check():
            raise _PlanningDeadlineExceeded
        if not collision_free:
            continue
        edge = _trusted_dynamic_primitive_cost(
            primitive,
            previous_target=node.previous_target,
            previous_direction=node.last_direction,
            distance_field=clearance,
            cells=cells[1:],
        )
        moving = samples[1:, 4][samples[1:, 4] != 0]
        direction = (
            ("forward" if moving[-1] > 0 else "reverse")
            if len(moving)
            else node.last_direction
        )
        yield _DynamicSuccessor(end, primitive, direction, edge)


def _nearest_reference_suffix(
    state: DynamicState,
    reference_xy: np.ndarray,
    *,
    parent_progress: int,
) -> tuple[int, float]:
    """Project onto the nearest reference point at or after parent progress."""
    suffix_distances = np.hypot(
        reference_xy[parent_progress:, 0] - state.x_m,
        reference_xy[parent_progress:, 1] - state.y_m,
    )
    suffix_index = int(np.argmin(suffix_distances))
    return parent_progress + suffix_index, float(suffix_distances[suffix_index])


def _build_reference_rollout(
    request: DynamicPlanRequest,
    drive_model: DriveModel,
    clearance: np.ndarray,
    deadline_check: Callable[[], bool] | None = None,
) -> _ReferenceRollout:
    """Build a deterministic beam of physically valid reference followers."""
    start = request.initial_state
    root = _SearchNode(
        start,
        0.0,
        None,
        None,
        start.drive_direction,
        DynamicControlTarget(
            start.main_command_rad_s, start.auxiliary_angle_rad
        ),
    )
    if request.guidance_cells is None:
        return _ReferenceRollout((root,), (), None, 0)

    reference_xy = (
        np.asarray(request.origin_xy, dtype=np.float64)
        + request.guidance_cells.astype(np.float64) * request.resolution_m
    )
    step_lengths_m = np.hypot(*np.diff(reference_xy, axis=0).T)
    remaining_m = np.empty(len(reference_xy), dtype=np.float64)
    remaining_m[-1] = 0.0
    remaining_m[:-1] = np.cumsum(step_lengths_m[::-1])[::-1]

    def stopped_at_goal(state: DynamicState) -> bool:
        return (
            math.hypot(state.x_m - request.goal_xy[0], state.y_m - request.goal_xy[1])
            <= 0.2 + 1e-12
            and state.speed_m_s < 0.05
            and abs(state.main_command_rad_s) <= 0.05
        )

    root_progress, _ = _nearest_reference_suffix(
        start, reference_xy, parent_progress=0
    )
    root = replace(root, reference_index=root_progress)
    nodes = [root]
    beam = [0]
    seed_indices: list[int] = []
    expanded = 0
    creation = itertools.count()
    templates: dict[tuple, DynamicPrimitive] = {}

    def dominance_label(node: _SearchNode) -> tuple:
        state = node.state
        return (
            state.x_m,
            state.y_m,
            state.yaw_rad,
            state.main_command_rad_s,
            state.auxiliary_angle_rad,
            state.speed_m_s,
            state.drive_direction,
            state.stopped_time_s >= 0.5,
            node.last_direction,
            node.previous_target,
        )

    best_seen = {dominance_label(root): root.cost}

    for _ in range(32):
        candidates: list[tuple[tuple, _SearchNode]] = []
        for parent_index in beam:
            parent = nodes[parent_index]
            if parent.state.elapsed_s - start.elapsed_s + 1.0 > 32.0:
                continue
            for successor in _iter_valid_successors(
                parent,
                request=request,
                drive_model=drive_model,
                clearance=clearance,
                templates=templates,
                deadline_check=deadline_check,
            ):
                expanded += 1
                parent_progress = (
                    parent.reference_index
                    if parent.reference_index is not None
                    else 0
                )
                progress, lateral_m = _nearest_reference_suffix(
                    successor.end,
                    reference_xy,
                    parent_progress=parent_progress,
                )
                child = _SearchNode(
                    successor.end,
                    parent.cost + successor.edge_cost,
                    parent_index,
                    successor.primitive,
                    successor.direction,
                    successor.primitive.target,
                    reference_index=progress,
                    source="reference_rollout",
                )
                rank = (
                    float(remaining_m[progress]),
                    lateral_m,
                    child.cost,
                    successor.primitive.target.main_command_rad_s,
                    successor.primitive.target.auxiliary_angle_rad,
                    next(creation),
                )
                candidates.append((rank, child))

        if not candidates:
            break
        candidates.sort(key=lambda item: item[0])
        goal_candidates = [item for item in candidates if stopped_at_goal(item[1].state)]
        if goal_candidates:
            nodes.append(goal_candidates[0][1])
            goal_index = len(nodes) - 1
            seed_indices.append(goal_index)
            return _ReferenceRollout(
                tuple(nodes), tuple(seed_indices), goal_index, expanded
            )

        layer_best: dict[tuple, _SearchNode] = {}
        for _, child in candidates:
            label = dominance_label(child)
            if child.cost + 1e-12 >= best_seen.get(label, math.inf):
                continue
            previous = layer_best.get(label)
            if previous is None:
                layer_best[label] = child
            elif child.cost + 1e-12 < previous.cost:
                # candidates are rank-sorted; assigning an existing dict key
                # retains its first insertion position while keeping the
                # cheapest duplicate generated in this layer.
                layer_best[label] = child
        nondominated = list(layer_best.values())

        beam = []
        selected: list[_SearchNode] = []
        selected_ids: set[int] = set()
        selected_directions: set[DriveDirection] = set()
        for child in nondominated:
            if child.state.drive_direction in selected_directions:
                continue
            selected.append(child)
            selected_ids.add(id(child))
            selected_directions.add(child.state.drive_direction)
            if len(selected) == 8:
                break
        if len(selected) < 8:
            for child in nondominated:
                if id(child) in selected_ids:
                    continue
                selected.append(child)
                selected_ids.add(id(child))
                if len(selected) == 8:
                    break

        for child in selected:
            nodes.append(child)
            child_index = len(nodes) - 1
            beam.append(child_index)
            seed_indices.append(child_index)
            # Cross-layer history contains only nodes that the beam retained.
            label = dominance_label(child)
            best_seen[label] = min(best_seen.get(label, math.inf), child.cost)

    return _ReferenceRollout(tuple(nodes), tuple(seed_indices), None, expanded)


def _search_label(node: _SearchNode, request: DynamicPlanRequest) -> tuple:
    # Dwell eligibility and the last moving direction affect future actions/cost.
    # Time is retained so a cheaper but later arrival cannot dominate one that
    # can still satisfy the 32 second horizon.
    return (
        dynamic_state_key(
            node.state,
            origin_xy=request.origin_xy,
            resolution_m=request.resolution_m,
        ),
        node.state.stopped_time_s >= 0.5,
        node.last_direction,
        node.previous_target,
        round(node.state.elapsed_s - request.initial_state.elapsed_s, 8),
    )


def _reconstruct_dynamic(
    nodes: list[_SearchNode], index: int, expanded: int
) -> DynamicPlanResult:
    chain: list[DynamicPrimitive] = []
    goal = nodes[index]
    while nodes[index].parent is not None:
        primitive = nodes[index].primitive
        assert primitive is not None
        chain.append(primitive)
        index = nodes[index].parent
    chain.reverse()
    start = nodes[index].state
    initial = np.array(
        [[
            start.elapsed_s,
            start.x_m,
            start.y_m,
            start.yaw_rad,
            start.main_command_rad_s,
            start.auxiliary_angle_rad,
            start.speed_m_s,
            start.stopped_time_s,
        ]]
    )
    samples = np.concatenate(
        [initial, *(primitive.samples[1:] for primitive in chain)]
    )
    waypoints = tuple(
        DynamicWaypoint(
            x_m=float(row[1]),
            y_m=float(row[2]),
            yaw_rad=float(row[3]),
            main_command_rad_s=float(row[4]),
            auxiliary_angle_rad=float(row[5]),
            speed_m_s=float(row[6]),
            time_from_start_s=float(row[0] - start.elapsed_s),
            drive_direction=(
                "forward" if row[4] > 0 else "reverse" if row[4] < 0 else "stopped"
            ),
        )
        for row in samples
    )
    directions = [start.drive_direction] if start.drive_direction != "stopped" else []
    for primitive in chain:
        moving = primitive.samples[1:, 4]
        moving = moving[moving != 0]
        if len(moving):
            directions.append("forward" if moving[0] > 0 else "reverse")
    return DynamicPlanResult(
        "success",
        waypoints,
        tuple(p.target for p in chain),
        expanded,
        sum(p.path_length_m for p in chain),
        goal.state.elapsed_s - start.elapsed_s,
        sum(a != b for a, b in zip(directions, directions[1:])),
        goal.cost,
    )


def plan_dynamic_lattice(
    request: DynamicPlanRequest, drive_model: DriveModel
) -> DynamicPlanResult:
    """A* over continuous 50 Hz rollouts, with deterministic quantized lookup."""
    if not isinstance(request, DynamicPlanRequest):
        raise TypeError("request must be a DynamicPlanRequest")
    if not isinstance(drive_model, DriveModel):
        raise TypeError("drive_model must be a DriveModel")
    started = time.monotonic()
    grid = request.occupancy_grid
    origin = np.asarray(request.origin_xy)

    def timed_out() -> bool:
        return time.monotonic() - started >= request.planning_timeout_s

    def valid_xy(xy: tuple[float, float]) -> bool:
        row, col = world_to_cell(
            xy, origin_xy=origin, resolution_m=request.resolution_m
        )
        return (
            0 <= row < grid.shape[0]
            and 0 <= col < grid.shape[1]
            and grid[row, col] != request.occupied
        )

    def within_goal_position(state: DynamicState) -> bool:
        return math.hypot(
            state.x_m - request.goal_xy[0], state.y_m - request.goal_xy[1]
        ) <= 0.2 + 1e-12

    start = request.initial_state
    if not valid_xy((start.x_m, start.y_m)):
        return DynamicPlanResult("invalid_start")
    if not valid_xy(request.goal_xy):
        return DynamicPlanResult("invalid_goal")
    root = _SearchNode(
        start,
        0.0,
        None,
        None,
        start.drive_direction,
        DynamicControlTarget(
            start.main_command_rad_s, start.auxiliary_angle_rad
        ),
    )
    if (
        within_goal_position(start)
        and start.speed_m_s < 0.05
        and abs(start.main_command_rad_s) <= 0.05
    ):
        return _reconstruct_dynamic([root], 0, 1)
    if timed_out():
        return DynamicPlanResult("timeout")
    distances = _grid_distance_to_goal(
        grid,
        goal_cell=world_to_cell(
            request.goal_xy,
            origin_xy=origin,
            resolution_m=request.resolution_m,
        ),
        resolution_m=request.resolution_m,
        occupied=request.occupied,
    )

    def heuristic(state: DynamicState) -> float:
        if within_goal_position(state):
            return 0.0
        cell = world_to_cell(
            (state.x_m, state.y_m),
            origin_xy=origin,
            resolution_m=request.resolution_m,
        )
        return max(0.0, float(distances[cell]) - 0.2)

    clearance = obstacle_distance_field(
        grid,
        resolution_m=request.resolution_m,
        occupied=request.occupied,
    )
    if timed_out():
        return DynamicPlanResult("timeout")

    if not math.isfinite(heuristic(start)):
        return DynamicPlanResult("unreachable")

    rollout: _ReferenceRollout | None = None
    if request.guidance_cells is not None:
        try:
            rollout = _build_reference_rollout(
                request,
                drive_model,
                clearance,
                deadline_check=timed_out,
            )
        except _PlanningDeadlineExceeded:
            return DynamicPlanResult("timeout", expanded_nodes=0)
        if rollout.goal_index is not None:
            return _reconstruct_dynamic(
                list(rollout.nodes),
                rollout.goal_index,
                rollout.expanded_nodes,
            )

    nodes = list(rollout.nodes) if rollout is not None else [root]
    seed_indices = (
        (0, *rollout.seed_indices) if rollout is not None else (0,)
    )
    best: dict[tuple, float] = {}
    tie = itertools.count()
    frontier: list[tuple[float, int, int]] = []
    for index in seed_indices:
        node = nodes[index]
        label = _search_label(node, request)
        if node.cost + 1e-12 >= best.get(label, math.inf):
            continue
        h = heuristic(node.state)
        if not math.isfinite(h):
            continue
        best[label] = node.cost
        heapq.heappush(frontier, (node.cost + h, next(tie), index))
    expanded = rollout.expanded_nodes if rollout is not None else 0
    templates: dict[tuple, DynamicPrimitive] = {}
    while frontier:
        if timed_out():
            return DynamicPlanResult("timeout", expanded_nodes=expanded)
        _, _, index = heapq.heappop(frontier)
        node = nodes[index]
        if node.cost > best[_search_label(node, request)] + 1e-12:
            continue
        if expanded >= request.node_budget:
            return DynamicPlanResult("node_budget_exhausted", expanded_nodes=expanded)
        expanded += 1
        state = node.state
        duration = state.elapsed_s - start.elapsed_s
        if (
            duration <= 32.0
            and within_goal_position(state)
            and state.speed_m_s < 0.05
            and abs(state.main_command_rad_s) <= 0.05
        ):
            return _reconstruct_dynamic(nodes, index, expanded)
        if duration + 1.0 > 32.0:
            continue
        successors = _iter_valid_successors(
            node,
            request=request,
            drive_model=drive_model,
            clearance=clearance,
            templates=templates,
            deadline_check=timed_out,
        )
        try:
            while True:
                successor = next(successors)
                if timed_out():
                    return DynamicPlanResult("timeout", expanded_nodes=expanded)
                child = _SearchNode(
                    successor.end,
                    node.cost + successor.edge_cost,
                    index,
                    successor.primitive,
                    successor.direction,
                    successor.primitive.target,
                    source="lattice",
                )
                label = _search_label(child, request)
                if child.cost + 1e-12 >= best.get(label, math.inf):
                    continue
                h = heuristic(successor.end)
                if not math.isfinite(h):
                    continue
                best[label] = child.cost
                nodes.append(child)
                heapq.heappush(frontier, (child.cost + h, next(tie), len(nodes) - 1))
        except StopIteration:
            pass
        except _PlanningDeadlineExceeded:
            return DynamicPlanResult("timeout", expanded_nodes=expanded)
    return DynamicPlanResult("unreachable", expanded_nodes=expanded)
