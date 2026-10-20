"""Deterministic occupancy-grid geometry and A* planning.

Grid cells and paths use ``(row, column)`` coordinates throughout this module.
"""

from __future__ import annotations

import heapq
import itertools
import math
from collections.abc import Iterator, Sequence

import numpy as np
from scipy.ndimage import distance_transform_edt


GridCell = tuple[int, int]

_NEIGHBORS: tuple[tuple[int, int, float], ...] = (
    (-1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, 0, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)


def _as_grid(grid: np.ndarray) -> np.ndarray:
    array = np.asarray(grid)
    if array.ndim != 2 or 0 in array.shape:
        raise ValueError("occupancy grid must be a non-empty 2D array")
    if np.issubdtype(array.dtype, np.number) and not np.all(np.isfinite(array)):
        raise ValueError("occupancy grid must contain only finite values")
    return array


def _cell(cell: Sequence[int], name: str) -> GridCell:
    if len(cell) != 2:
        raise ValueError(f"{name} must be a (row, column) cell")
    row, column = cell
    if isinstance(row, (bool, np.bool_)) or isinstance(column, (bool, np.bool_)):
        raise ValueError(f"{name} must contain integer cell coordinates")
    if not isinstance(row, (int, np.integer)) or not isinstance(column, (int, np.integer)):
        raise ValueError(f"{name} must contain integer cell coordinates")
    return int(row), int(column)


def _require_inside(grid: np.ndarray, cell: GridCell, name: str) -> None:
    row, column = cell
    if row < 0 or row >= grid.shape[0] or column < 0 or column >= grid.shape[1]:
        raise ValueError(f"{name} cell is outside the occupancy map")


def inflate_obstacles(
    grid: np.ndarray,
    *,
    radius_m: float,
    resolution_m: float,
    occupied: int | bool = 1,
) -> np.ndarray:
    """Inflate occupied cells by an inclusive Euclidean metric radius."""

    array = _as_grid(grid)
    radius = float(radius_m)
    resolution = float(resolution_m)
    if not math.isfinite(radius) or radius < 0.0:
        raise ValueError("radius_m must be finite and non-negative")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolution_m must be finite and positive")

    obstacle_mask = array == occupied
    if not np.any(obstacle_mask):
        return array.copy()
    distance_m = distance_transform_edt(~obstacle_mask) * resolution
    inflated_mask = distance_m <= radius + 1e-12
    inflated = array.copy()
    inflated[inflated_mask] = occupied
    return inflated


def astar_path(
    grid: np.ndarray,
    *,
    start: Sequence[int],
    goal: Sequence[int],
    occupied: int | bool = 1,
) -> np.ndarray:
    """Return a deterministic shortest 8-neighbor path without corner cutting."""

    array = _as_grid(grid)
    start_cell = _cell(start, "start")
    goal_cell = _cell(goal, "goal")
    _require_inside(array, start_cell, "start")
    _require_inside(array, goal_cell, "goal")
    blocked = array == occupied
    if blocked[start_cell]:
        raise ValueError("start cell is occupied")
    if blocked[goal_cell]:
        raise ValueError("goal cell is occupied")
    if start_cell == goal_cell:
        return np.asarray((start_cell,), dtype=np.int64)

    def heuristic(cell: GridCell) -> float:
        return math.hypot(cell[0] - goal_cell[0], cell[1] - goal_cell[1])

    tie = itertools.count()
    frontier: list[tuple[float, int, GridCell]] = [(heuristic(start_cell), next(tie), start_cell)]
    costs: dict[GridCell, float] = {start_cell: 0.0}
    parents: dict[GridCell, GridCell] = {}
    closed: set[GridCell] = set()

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if current == goal_cell:
            path = [current]
            while path[-1] != start_cell:
                path.append(parents[path[-1]])
            path.reverse()
            return np.asarray(path, dtype=np.int64)
        closed.add(current)

        row, column = current
        for row_step, column_step, step_cost in _NEIGHBORS:
            neighbor = row + row_step, column + column_step
            neighbor_row, neighbor_column = neighbor
            if (
                neighbor_row < 0
                or neighbor_row >= array.shape[0]
                or neighbor_column < 0
                or neighbor_column >= array.shape[1]
                or blocked[neighbor]
            ):
                continue
            if row_step and column_step:
                if blocked[row + row_step, column] or blocked[row, column + column_step]:
                    continue

            candidate_cost = costs[current] + step_cost
            if candidate_cost + 1e-12 >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = candidate_cost
            parents[neighbor] = current
            priority = candidate_cost + heuristic(neighbor)
            heapq.heappush(frontier, (priority, next(tie), neighbor))

    raise ValueError("goal is unreachable on the occupancy grid")


def path_length_cells(path: np.ndarray) -> float:
    """Return path length in grid-cell units."""

    array = np.asarray(path, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 2 or array.shape[0] == 0:
        raise ValueError("path must have shape (N, 2) with N >= 1")
    if not np.all(np.isfinite(array)):
        raise ValueError("path must contain only finite coordinates")
    if array.shape[0] == 1:
        return 0.0
    return float(np.linalg.norm(np.diff(array, axis=0), axis=1).sum())


def _supercover_steps(start: GridCell, goal: GridCell) -> Iterator[GridCell]:
    """Yield cells crossed by the center-to-center segment in stable order."""

    row, column = start
    goal_row, goal_column = goal
    delta_row = goal_row - row
    delta_column = goal_column - column
    row_count = abs(delta_row)
    column_count = abs(delta_column)
    row_sign = 0 if delta_row == 0 else (1 if delta_row > 0 else -1)
    column_sign = 0 if delta_column == 0 else (1 if delta_column > 0 else -1)
    row_index = 0
    column_index = 0
    yield row, column

    while row_index < row_count or column_index < column_count:
        decision = (1 + 2 * row_index) * column_count - (
            1 + 2 * column_index
        ) * row_count
        if decision == 0:
            row += row_sign
            column += column_sign
            row_index += 1
            column_index += 1
        elif decision < 0:
            row += row_sign
            row_index += 1
        else:
            column += column_sign
            column_index += 1
        yield row, column


def line_is_free(
    grid: np.ndarray,
    *,
    start: Sequence[int],
    goal: Sequence[int],
    occupied: int | bool = 1,
) -> bool:
    """Check a grid line conservatively, including diagonal corner contacts."""

    array = _as_grid(grid)
    start_cell = _cell(start, "start")
    goal_cell = _cell(goal, "goal")
    _require_inside(array, start_cell, "start")
    _require_inside(array, goal_cell, "goal")
    blocked = array == occupied

    previous: GridCell | None = None
    for current in _supercover_steps(start_cell, goal_cell):
        if blocked[current]:
            return False
        if previous is not None and previous[0] != current[0] and previous[1] != current[1]:
            if blocked[previous[0], current[1]] or blocked[current[0], previous[1]]:
                return False
        previous = current
    return True
