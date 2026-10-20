"""Validation and auditable evidence for E0/E1/E2 benchmark tasks."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal, Sequence

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from .grid_planning import astar_path, inflate_obstacles, line_is_free, path_length_cells


Difficulty = Literal["E0", "E1", "E2"]


@dataclass(frozen=True)
class TaskEvidence:
    line_is_free: bool
    euclidean_distance_m: float
    oracle_path_length_m: float
    detour_ratio: float
    start_clearance_m: float
    goal_clearance_m: float
    corridor_intersection: bool
    corridor_width_m: float | None
    nearby_obstacle_count: int
    corridor_path_length_m: float


def build_two_sided_corridor_mask(
    grid: np.ndarray,
    *,
    resolution_m: float,
    minimum_width_m: float,
    maximum_width_m: float,
    occupied: int | bool = 1,
) -> np.ndarray:
    """Mark free cells bounded by obstacles on opposite sides.

    Unlike a clearance-distance band, this test does not create rings around
    isolated obstacles.  A cell is marked only when occupied cells occur at
    opposite offsets whose total separation lies in the requested width band.
    """

    array = np.asarray(grid)
    if array.ndim != 2:
        raise ValueError("occupancy grid must be two-dimensional")
    resolution = float(resolution_m)
    minimum_width = float(minimum_width_m)
    maximum_width = float(maximum_width_m)
    if not all(math.isfinite(value) for value in (resolution, minimum_width, maximum_width)):
        raise ValueError("corridor mask dimensions must be finite")
    if resolution <= 0.0 or minimum_width <= 0.0 or maximum_width < minimum_width:
        raise ValueError("corridor width range and resolution must be positive and ordered")

    obstacles = array == occupied
    corridor = np.zeros(array.shape, dtype=bool)
    minimum_half_cells = minimum_width / (2.0 * resolution)
    maximum_half_cells = maximum_width / (2.0 * resolution)
    maximum_offset = int(math.ceil(maximum_half_cells))
    rows, columns = array.shape

    for row_offset in range(0, maximum_offset + 1):
        for column_offset in range(-maximum_offset, maximum_offset + 1):
            if row_offset == 0 and column_offset <= 0:
                continue
            radius = math.hypot(row_offset, column_offset)
            if radius + 1e-9 < minimum_half_cells or radius > maximum_half_cells + 1e-9:
                continue
            row_start = row_offset
            row_stop = rows - row_offset
            column_start = abs(column_offset)
            column_stop = columns - abs(column_offset)
            if row_start >= row_stop or column_start >= column_stop:
                continue
            centers = np.s_[row_start:row_stop, column_start:column_stop]
            plus = np.s_[
                row_start + row_offset : row_stop + row_offset,
                column_start + column_offset : column_stop + column_offset,
            ]
            minus = np.s_[
                row_start - row_offset : row_stop - row_offset,
                column_start - column_offset : column_stop - column_offset,
            ]
            corridor[centers] |= obstacles[plus] & obstacles[minus]
    return corridor & ~obstacles


def _nearby_obstacle_count(
    grid: np.ndarray,
    path: np.ndarray,
    *,
    resolution_m: float,
    radius_m: float,
    occupied: int | bool,
) -> int:
    obstacles = np.asarray(grid) == occupied
    components, _ = label(obstacles, structure=np.ones((3, 3), dtype=np.uint8))
    path_mask = np.zeros(obstacles.shape, dtype=bool)
    path_mask[path[:, 0], path[:, 1]] = True
    distance_to_path_m = distance_transform_edt(~path_mask) * resolution_m
    nearby_labels = np.unique(
        components[obstacles & (distance_to_path_m <= radius_m + 1e-9)]
    )
    return int(np.count_nonzero(nearby_labels))


def _longest_masked_path_length_m(
    path: np.ndarray,
    mask: np.ndarray,
    *,
    resolution_m: float,
) -> float:
    inside = mask[path[:, 0], path[:, 1]]
    longest = 0.0
    current = 0.0
    for index in range(1, path.shape[0]):
        if inside[index - 1] and inside[index]:
            current += float(np.linalg.norm(path[index] - path[index - 1])) * resolution_m
            longest = max(longest, current)
        else:
            current = 0.0
    return longest


def _clearance_map_m(
    grid: np.ndarray,
    *,
    resolution_m: float,
    occupied: int | bool,
) -> np.ndarray:
    free = np.asarray(grid) != occupied
    # Treat space beyond the frozen map as occupied when reporting clearance.
    padded_free = np.pad(free, 1, mode="constant", constant_values=False)
    return distance_transform_edt(padded_free)[1:-1, 1:-1] * resolution_m


def validate_task(
    grid: np.ndarray,
    *,
    start: Sequence[int],
    goal: Sequence[int],
    difficulty: Difficulty,
    resolution_m: float,
    occupied: int | bool = 1,
    ball_radius_m: float = 0.4,
    safety_margin_m: float = 0.2,
    e1_min_detour_ratio: float = 1.15,
    e2_extra_width_m: float = 0.4,
    corridor_mask: np.ndarray | None = None,
    corridor_width_m: float | None = None,
    minimum_nearby_obstacles: int = 0,
    nearby_obstacle_radius_m: float = 2.0,
    minimum_corridor_path_length_m: float = 0.0,
) -> TaskEvidence:
    """Validate a task class and return evidence derived from one frozen grid."""

    if difficulty not in ("E0", "E1", "E2"):
        raise ValueError(f"unsupported difficulty: {difficulty!r}")
    resolution = float(resolution_m)
    ball_radius = float(ball_radius_m)
    safety_margin = float(safety_margin_m)
    minimum_detour = float(e1_min_detour_ratio)
    extra_width = float(e2_extra_width_m)
    nearby_radius = float(nearby_obstacle_radius_m)
    minimum_corridor_length = float(minimum_corridor_path_length_m)
    for name, value in (
        ("resolution_m", resolution),
        ("ball_radius_m", ball_radius),
        ("safety_margin_m", safety_margin),
        ("e1_min_detour_ratio", minimum_detour),
        ("e2_extra_width_m", extra_width),
        ("nearby_obstacle_radius_m", nearby_radius),
        ("minimum_corridor_path_length_m", minimum_corridor_length),
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if resolution <= 0.0:
        raise ValueError("resolution_m must be positive")
    if (
        ball_radius < 0.0
        or safety_margin < 0.0
        or extra_width < 0.0
        or nearby_radius < 0.0
        or minimum_corridor_length < 0.0
    ):
        raise ValueError("robot size, safety margin, and width range must be non-negative")
    if isinstance(minimum_nearby_obstacles, bool) or minimum_nearby_obstacles < 0:
        raise ValueError("minimum_nearby_obstacles must be a non-negative integer")
    if minimum_detour < 1.0:
        raise ValueError("e1_min_detour_ratio must be at least 1.0")

    array = np.asarray(grid)
    inflated = inflate_obstacles(
        array,
        radius_m=ball_radius + safety_margin,
        resolution_m=resolution,
        occupied=occupied,
    )
    path = astar_path(inflated, start=start, goal=goal, occupied=occupied)
    start_cell = tuple(int(value) for value in path[0])
    goal_cell = tuple(int(value) for value in path[-1])
    straight_is_free = line_is_free(
        inflated,
        start=start_cell,
        goal=goal_cell,
        occupied=occupied,
    )

    delta = np.asarray(goal_cell, dtype=np.float64) - np.asarray(
        start_cell,
        dtype=np.float64,
    )
    euclidean_distance_m = float(np.linalg.norm(delta) * resolution)
    if euclidean_distance_m <= 0.0:
        raise ValueError("benchmark start and goal must be distinct")
    oracle_path_length_m = path_length_cells(path) * resolution
    detour_ratio = oracle_path_length_m / euclidean_distance_m

    corridor_intersection = False
    corridor_path_length_m = 0.0
    normalized_corridor_width: float | None = None
    if corridor_mask is not None:
        mask = np.asarray(corridor_mask)
        if mask.shape != array.shape:
            raise ValueError("corridor mask shape must match the occupancy grid")
        if mask.dtype != np.bool_:
            mask = mask.astype(bool)
        corridor_intersection = bool(np.any(mask[path[:, 0], path[:, 1]]))
        corridor_path_length_m = _longest_masked_path_length_m(
            path,
            mask,
            resolution_m=resolution,
        )
    if corridor_width_m is not None:
        normalized_corridor_width = float(corridor_width_m)
        if not math.isfinite(normalized_corridor_width):
            raise ValueError("corridor width must be finite")

    if difficulty == "E0" and not straight_is_free:
        raise ValueError("E0 task requires a free straight line")
    if difficulty == "E1":
        if straight_is_free:
            raise ValueError("E1 task requires a blocked straight line")
        if detour_ratio + 1e-9 < minimum_detour:
            raise ValueError(
                f"E1 detour ratio {detour_ratio:.6f} is below {minimum_detour:.6f}"
            )
    if difficulty == "E2":
        minimum_width = 2.0 * (ball_radius + safety_margin)
        maximum_width = minimum_width + extra_width
        if normalized_corridor_width is None:
            raise ValueError("E2 corridor width is required")
        if not (
            minimum_width - 1e-9
            <= normalized_corridor_width
            <= maximum_width + 1e-9
        ):
            raise ValueError(
                "E2 corridor width must be within "
                f"[{minimum_width:.6f}, {maximum_width:.6f}] m"
            )
        if corridor_mask is None or not corridor_intersection:
            raise ValueError("E2 oracle path must intersect the annotated corridor")
        if corridor_path_length_m + 1e-9 < minimum_corridor_length:
            raise ValueError(
                "E2 continuous corridor path length "
                f"{corridor_path_length_m:.6f} m is below "
                f"{minimum_corridor_length:.6f} m"
            )

    nearby_obstacle_count = _nearby_obstacle_count(
        array,
        path,
        resolution_m=resolution,
        radius_m=nearby_radius,
        occupied=occupied,
    )
    if nearby_obstacle_count < minimum_nearby_obstacles:
        raise ValueError(
            f"task has {nearby_obstacle_count} nearby obstacles; "
            f"requires at least {minimum_nearby_obstacles}"
        )

    clearance = _clearance_map_m(
        array,
        resolution_m=resolution,
        occupied=occupied,
    )
    return TaskEvidence(
        line_is_free=straight_is_free,
        euclidean_distance_m=euclidean_distance_m,
        oracle_path_length_m=oracle_path_length_m,
        detour_ratio=detour_ratio,
        start_clearance_m=float(clearance[start_cell]),
        goal_clearance_m=float(clearance[goal_cell]),
        corridor_intersection=corridor_intersection,
        corridor_width_m=normalized_corridor_width,
        nearby_obstacle_count=nearby_obstacle_count,
        corridor_path_length_m=corridor_path_length_m,
    )
