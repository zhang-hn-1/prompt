"""Scene-level trajectory overlays for the Isaac Sim collector.

The collector stores local BEV images per stereo frame.  This module renders a
single, human-readable top-down PNG for a whole scene by placing the recorded
world-frame XY paths over the global occupancy raster.  It deliberately uses
Pillow only so it remains usable in a headless Isaac/Python environment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont

try:  # Support both ``scripts.trajectory_map`` and direct script imports.
    from .bev_ground_truth import GlobalOccupancy
except ImportError:  # pragma: no cover - exercised by standalone script entrypoints
    from bev_ground_truth import GlobalOccupancy


CANVAS_SIZE = (1200, 900)
_MARGIN_PX = 48
_HEADER_PX = 62
_BACKGROUND = (250, 251, 252)
_FREE = (231, 235, 239)
_OCCUPIED = (48, 53, 59)
_UNKNOWN = (177, 184, 192)
_TRAJECTORY_COLORS = (
    (26, 115, 232),
    (220, 70, 50),
    (0, 145, 105),
    (158, 79, 201),
    (225, 137, 24),
    (197, 51, 132),
    (40, 145, 210),
    (117, 117, 117),
)
_START_COLOR = (35, 170, 80)
_END_COLOR = (27, 91, 190)
_COLLISION_COLOR = (225, 33, 33)
_INFLATED_COLOR = (220, 53, 69, 118)
_STRAIGHT_COLOR = (221, 145, 32)
_ORACLE_COLOR = (35, 105, 210)
_GOAL_COLOR = (204, 45, 146)


TASK_CARD_LAYERS = frozenset(
    (
        "obstacles",
        "inflated",
        "start_heading",
        "goal",
        "straight_line",
        "oracle_path",
        "metrics",
    )
)


@dataclass(frozen=True)
class TrajectoryRecord:
    """One world-frame path and its optional collision-marked samples."""

    label: str
    points_xy: np.ndarray
    collision_points_xy: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float64)
    )
    color_rgb: tuple[int, int, int] | None = None
    draw_endpoint_markers: bool = True
    end_color_rgb: tuple[int, int, int] = _END_COLOR

    def __post_init__(self) -> None:
        if not isinstance(self.label, str) or not self.label.strip():
            raise ValueError("trajectory label must be a non-empty string")
        points = _as_points(self.points_xy, "points_xy")
        collisions = _as_points(self.collision_points_xy, "collision_points_xy")
        if points.shape[0] == 0:
            raise ValueError("points_xy must contain at least one point")
        color = None if self.color_rgb is None else _as_rgb(self.color_rgb, "color_rgb")
        end_color = _as_rgb(self.end_color_rgb, "end_color_rgb")
        if not isinstance(self.draw_endpoint_markers, bool):
            raise ValueError("draw_endpoint_markers must be a boolean")
        object.__setattr__(self, "points_xy", points)
        object.__setattr__(self, "collision_points_xy", collisions)
        object.__setattr__(self, "color_rgb", color)
        object.__setattr__(self, "end_color_rgb", end_color)


@dataclass(frozen=True)
class NavigationTaskOverlay:
    """Grid-aligned overlays needed for a human-review task card."""

    inflated_grid: np.ndarray
    start_cell: tuple[int, int]
    start_yaw_rad: float
    goal_cell: tuple[int, int]
    oracle_path_cells: np.ndarray
    metrics: tuple[str, ...]

    def __post_init__(self) -> None:
        inflated = np.asarray(self.inflated_grid)
        if inflated.ndim != 2 or 0 in inflated.shape:
            raise ValueError("inflated_grid must be a non-empty 2D array")
        if not np.all(np.isfinite(inflated)):
            raise ValueError("inflated_grid must contain finite values")
        start = _as_grid_cell(self.start_cell, "start_cell")
        goal = _as_grid_cell(self.goal_cell, "goal_cell")
        yaw = float(self.start_yaw_rad)
        if not math.isfinite(yaw):
            raise ValueError("start_yaw_rad must be finite")
        path = np.asarray(self.oracle_path_cells)
        if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] == 0:
            raise ValueError("oracle_path_cells must have shape (N, 2)")
        if not np.all(np.isfinite(path)) or not np.array_equal(path, np.rint(path)):
            raise ValueError("oracle_path_cells must contain finite integer cells")
        metrics = tuple(self.metrics)
        if not metrics or any(not isinstance(line, str) or not line.strip() for line in metrics):
            raise ValueError("metrics must contain non-empty strings")
        object.__setattr__(self, "inflated_grid", inflated.copy())
        object.__setattr__(self, "start_cell", start)
        object.__setattr__(self, "goal_cell", goal)
        object.__setattr__(self, "start_yaw_rad", yaw)
        object.__setattr__(self, "oracle_path_cells", path.astype(np.int64, copy=True))
        object.__setattr__(self, "metrics", metrics)


def _as_grid_cell(value: object, name: str) -> tuple[int, int]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must be a (row, column) cell")
    if any(
        isinstance(item, (bool, np.bool_)) or not isinstance(item, (int, np.integer))
        for item in value
    ):
        raise ValueError(f"{name} must contain integer coordinates")
    return int(value[0]), int(value[1])


def _as_points(value: object, name: str) -> np.ndarray:
    try:
        points = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must have shape (N, 2)") from exc
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(f"{name} must have shape (N, 2)")
    if not np.isfinite(points).all():
        raise ValueError(f"{name} must contain finite values")
    return points.copy()


def _as_rgb(value: object, name: str) -> tuple[int, int, int]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError(f"{name} must be an RGB triple")
    if any(
        isinstance(item, (bool, np.bool_))
        or not isinstance(item, (int, np.integer))
        or not 0 <= int(item) <= 255
        for item in value
    ):
        raise ValueError(f"{name} must contain integer channels in [0, 255]")
    return tuple(int(item) for item in value)


def reconstruct_observed_occupancy(
    frame_maps: Sequence[np.ndarray],
    positions_xy: np.ndarray,
    yaws: np.ndarray,
    config: object,
) -> GlobalOccupancy:
    """Fuse saved local BEV frames into an approximate world-frame map.

    This is intended for post-processing already collected episodes.  The
    collector itself uses its exact global map directly; when that map was not
    persisted, the saved local labels are placed back into world coordinates
    and fused with occupied cells taking precedence over free cells.
    """

    if not hasattr(config, "shape") or not hasattr(config, "resolution_m"):
        raise TypeError("config must provide shape and resolution_m")
    shape = tuple(int(value) for value in config.shape)
    if len(shape) != 2 or any(value <= 0 for value in shape):
        raise ValueError("config.shape must contain two positive dimensions")
    resolution = float(config.resolution_m)
    if not np.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("config.resolution_m must be positive and finite")

    frames = tuple(np.asarray(frame) for frame in frame_maps)
    positions = np.asarray(positions_xy, dtype=np.float64)
    angles = np.asarray(yaws, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise ValueError("positions_xy must have shape (N, 2)")
    if angles.ndim != 1 or angles.shape[0] != positions.shape[0]:
        raise ValueError("yaws must have shape (N,)")
    if len(frames) != positions.shape[0]:
        raise ValueError("frame_maps, positions_xy, and yaws must have equal lengths")
    if len(frames) == 0:
        raise ValueError("at least one local BEV frame is required")
    if not np.isfinite(positions).all() or not np.isfinite(angles).all():
        raise ValueError("positions_xy and yaws must contain finite values")

    rows = np.arange(shape[0], dtype=np.float64)
    cols = np.arange(shape[1], dtype=np.float64)
    center_row, center_col = shape[0] // 2, shape[1] // 2
    local_x, local_y = np.meshgrid(
        (center_row - rows) * resolution,
        (center_col - cols) * resolution,
        indexing="ij",
    )
    local = np.stack((local_x, local_y), axis=-1).reshape(-1, 2)
    world_frames: list[np.ndarray] = []
    normalized_frames: list[np.ndarray] = []
    for frame, position, yaw in zip(frames, positions, angles):
        if frame.shape != shape:
            raise ValueError(f"local BEV frames must have shape {shape}, got {frame.shape}")
        values = np.asarray(frame, dtype=np.float64)
        if not np.isfinite(values).all() or not np.array_equal(values, np.rint(values)):
            raise ValueError("local BEV frames must contain finite integral labels")
        values = np.asarray(values, dtype=np.uint8)
        if np.any(~np.isin(values, (0, 1, 2))):
            raise ValueError("local BEV frames must use occupancy codes 0, 1, and 2")
        cosine, sine = np.cos(yaw), np.sin(yaw)
        world_delta = np.stack(
            (cosine * local[:, 0] - sine * local[:, 1],
             sine * local[:, 0] + cosine * local[:, 1]),
            axis=-1,
        )
        world_frames.append(world_delta + position)
        normalized_frames.append(values.reshape(-1))

    all_world = np.concatenate(world_frames, axis=0)
    origin = all_world.min(axis=0)
    maximum = all_world.max(axis=0)
    grid_shape = tuple(np.floor((maximum - origin) / resolution + 0.5).astype(np.int64) + 1)
    grid = np.full(grid_shape, 2, dtype=np.uint8)
    priority = np.zeros(grid_shape, dtype=np.uint8)
    priority_by_label = np.array([1, 3, 0], dtype=np.uint8)
    for world, values in zip(world_frames, normalized_frames):
        indices = np.floor((world - origin) / resolution + 0.5).astype(np.int64)
        candidate_priority = priority_by_label[values]
        # A rotated local raster can quantize several cells to one global
        # cell.  Resolve those duplicates before reading/writing ``priority``;
        # otherwise a later free value in the same vector could overwrite an
        # earlier occupied value using a stale pre-write priority.
        flat_indices = indices[:, 0] * grid_shape[1] + indices[:, 1]
        order = np.lexsort((-candidate_priority, flat_indices))
        sorted_flat = flat_indices[order]
        keep = np.empty(sorted_flat.shape, dtype=bool)
        keep[0] = True
        keep[1:] = sorted_flat[1:] != sorted_flat[:-1]
        chosen = order[keep]
        chosen_indices = indices[chosen]
        chosen_values = values[chosen]
        chosen_priority = candidate_priority[chosen]
        current_priority = priority[chosen_indices[:, 0], chosen_indices[:, 1]]
        update = chosen_priority > current_priority
        if np.any(update):
            rows_i, cols_i = chosen_indices[update].T
            grid[rows_i, cols_i] = chosen_values[update]
            priority[rows_i, cols_i] = chosen_priority[update]
    return GlobalOccupancy(grid=grid, global_origin_xy=origin, resolution_m=resolution)


def _occupancy_rgb(grid: np.ndarray, free: int, occupied: int, unknown: int) -> np.ndarray:
    """Convert ``grid[world_x, world_y]`` labels to a north-up RGB raster."""

    rgb = np.empty((*grid.shape, 3), dtype=np.uint8)
    rgb[:] = _UNKNOWN
    rgb[grid == free] = _FREE
    rgb[grid == occupied] = _OCCUPIED
    rgb[grid == unknown] = _UNKNOWN
    # Image rows run downward while the global grid's axes run +X, +Y.
    return np.flipud(np.transpose(rgb, (1, 0, 2)))


def _world_to_pixel(
    point_xy: np.ndarray,
    world_min_xy: np.ndarray,
    world_max_xy: np.ndarray,
    canvas_left: float,
    canvas_top: float,
    scale: float,
) -> tuple[int, int]:
    point = np.asarray(point_xy, dtype=np.float64)
    px = canvas_left + (float(point[0]) - float(world_min_xy[0])) * scale
    py = canvas_top + (float(world_max_xy[1]) - float(point[1])) * scale
    return int(round(px)), int(round(py))


def render_trajectory_map(
    output_path: str | Path,
    occupancy: GlobalOccupancy,
    trajectories: Sequence[TrajectoryRecord],
    *,
    title: str = "Robot trajectory",
) -> Path:
    """Render a top-down trajectory map and atomically publish it as a PNG.

    The map uses world X to the right and world Y upward.  Occupancy cells are
    shown as free (light), occupied (dark), or unknown (gray).  Every
    trajectory is drawn in a distinct color; start/end markers and collision
    samples are overlaid for quick collection-quality inspection.
    """

    if not isinstance(occupancy, GlobalOccupancy):
        raise TypeError("occupancy must be a GlobalOccupancy")
    if not trajectories:
        raise ValueError("at least one trajectory is required")
    records = tuple(trajectories)
    if any(not isinstance(record, TrajectoryRecord) for record in records):
        raise TypeError("trajectories must contain TrajectoryRecord values")
    if not isinstance(title, str) or not title.strip():
        raise ValueError("title must be a non-empty string")

    grid = np.asarray(occupancy.grid, dtype=np.uint8)
    resolution = float(occupancy.resolution_m)
    origin = np.asarray(occupancy.global_origin_xy, dtype=np.float64)
    occupied_min = origin - resolution / 2.0
    occupied_max = origin + (np.asarray(grid.shape, dtype=np.float64) - 0.5) * resolution

    all_points = np.concatenate(
        [
            np.concatenate((record.points_xy, record.collision_points_xy), axis=0)
            for record in records
        ],
        axis=0,
    )
    padding = max(0.5, resolution)
    world_min = np.minimum(occupied_min, all_points.min(axis=0)) - padding
    world_max = np.maximum(occupied_max, all_points.max(axis=0)) + padding
    span = np.maximum(world_max - world_min, resolution)

    canvas_width, canvas_height = CANVAS_SIZE
    plot_left = float(_MARGIN_PX)
    plot_top = float(_HEADER_PX)
    plot_width = float(canvas_width - 2 * _MARGIN_PX)
    plot_height = float(canvas_height - _HEADER_PX - _MARGIN_PX)
    scale = min(plot_width / span[0], plot_height / span[1])
    rendered_width = span[0] * scale
    rendered_height = span[1] * scale
    canvas_left = plot_left + (plot_width - rendered_width) / 2.0
    canvas_top = plot_top + (plot_height - rendered_height) / 2.0

    image = Image.new("RGB", CANVAS_SIZE, _BACKGROUND)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    draw.text((_MARGIN_PX, 18), title, fill=(20, 25, 30), font=font)

    # The current collection contract reserves 0/1/2 for free/occupied/unknown.
    occupancy_image = Image.fromarray(
        _occupancy_rgb(grid, 0, 1, 2),
        mode="RGB",
    ).resize(
        (
            max(1, int(round(grid.shape[0] * resolution * scale))),
            max(1, int(round(grid.shape[1] * resolution * scale))),
        ),
        resample=Image.Resampling.NEAREST,
    )
    occ_left, occ_top = _world_to_pixel(
        occupied_min,
        world_min,
        world_max,
        canvas_left,
        canvas_top,
        scale,
    )
    # The cell array's top-left corner is at the world-X minimum and +Y maximum.
    occ_top = _world_to_pixel(
        np.array([occupied_min[0], occupied_max[1]]),
        world_min,
        world_max,
        canvas_left,
        canvas_top,
        scale,
    )[1]
    image.paste(occupancy_image, (occ_left, occ_top))
    draw.rectangle(
        (
            int(round(canvas_left)),
            int(round(canvas_top)),
            int(round(canvas_left + rendered_width)),
            int(round(canvas_top + rendered_height)),
        ),
        outline=(145, 151, 157),
        width=2,
    )

    for index, record in enumerate(records):
        color = (
            _TRAJECTORY_COLORS[index % len(_TRAJECTORY_COLORS)]
            if record.color_rgb is None
            else record.color_rgb
        )
        pixels = [
            _world_to_pixel(point, world_min, world_max, canvas_left, canvas_top, scale)
            for point in record.points_xy
        ]
        if len(pixels) > 1:
            draw.line(pixels, fill=color, width=4, joint="curve")
        for point in record.collision_points_xy:
            px, py = _world_to_pixel(point, world_min, world_max, canvas_left, canvas_top, scale)
            radius = 5
            draw.ellipse((px - radius, py - radius, px + radius, py + radius), fill=_COLLISION_COLOR)
        if record.draw_endpoint_markers:
            start_x, start_y = pixels[0]
            end_x, end_y = pixels[-1]
            draw.ellipse(
                (start_x - 7, start_y - 7, start_x + 7, start_y + 7),
                fill=_START_COLOR,
            )
            draw.ellipse(
                (end_x - 7, end_y - 7, end_x + 7, end_y + 7),
                fill=record.end_color_rgb,
            )
            draw.ellipse(
                (start_x - 3, start_y - 3, start_x + 3, start_y + 3),
                fill=(255, 255, 255),
            )

        legend_x = canvas_width - _MARGIN_PX - 175
        legend_y = 18 + index * 18
        draw.line((legend_x, legend_y + 6, legend_x + 20, legend_y + 6), fill=color, width=4)
        draw.text((legend_x + 28, legend_y), record.label, fill=(20, 25, 30), font=font)

    draw.text(
        (_MARGIN_PX, canvas_height - 24),
        "green=start  endpoint=goal/end  red=collision/clearance <= 0",
        fill=(70, 76, 82),
        font=font,
    )

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".partial")
    image.save(temporary, format="PNG")
    temporary.replace(destination)
    return destination


def _draw_dashed_line(
    draw: ImageDraw.ImageDraw,
    start: tuple[int, int],
    end: tuple[int, int],
    *,
    fill: tuple[int, int, int],
    width: int,
    dash_px: float = 12.0,
    gap_px: float = 8.0,
) -> None:
    delta_x = float(end[0] - start[0])
    delta_y = float(end[1] - start[1])
    length = math.hypot(delta_x, delta_y)
    if length == 0.0:
        return
    cursor = 0.0
    while cursor < length:
        segment_end = min(cursor + dash_px, length)
        p0 = (
            int(round(start[0] + delta_x * cursor / length)),
            int(round(start[1] + delta_y * cursor / length)),
        )
        p1 = (
            int(round(start[0] + delta_x * segment_end / length)),
            int(round(start[1] + delta_y * segment_end / length)),
        )
        draw.line((p0, p1), fill=fill, width=width)
        cursor += dash_px + gap_px


def render_navigation_task_map(
    output_path: str | Path,
    occupancy: GlobalOccupancy,
    overlay: NavigationTaskOverlay,
    *,
    title: str,
) -> tuple[Path, Mapping[str, bool]]:
    """Render the seven mandatory task-review layers to a 1200x900 PNG."""

    if not isinstance(occupancy, GlobalOccupancy):
        raise TypeError("occupancy must be a GlobalOccupancy")
    if not isinstance(overlay, NavigationTaskOverlay):
        raise TypeError("overlay must be a NavigationTaskOverlay")
    if not isinstance(title, str) or not title.strip():
        raise ValueError("title must be a non-empty string")

    grid = np.asarray(occupancy.grid, dtype=np.uint8)
    inflated = np.asarray(overlay.inflated_grid, dtype=np.uint8)
    if inflated.shape != grid.shape:
        raise ValueError("inflated_grid shape must match occupancy grid")
    resolution = float(occupancy.resolution_m)
    origin = np.asarray(occupancy.global_origin_xy, dtype=np.float64)
    if origin.shape != (2,) or not np.all(np.isfinite(origin)):
        raise ValueError("occupancy origin must be a finite XY pair")
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("occupancy resolution must be finite and positive")

    cells = np.vstack(
        (
            np.asarray(overlay.start_cell, dtype=np.int64),
            np.asarray(overlay.goal_cell, dtype=np.int64),
            overlay.oracle_path_cells,
        )
    )
    if (
        np.any(cells[:, 0] < 0)
        or np.any(cells[:, 0] >= grid.shape[0])
        or np.any(cells[:, 1] < 0)
        or np.any(cells[:, 1] >= grid.shape[1])
    ):
        raise ValueError("task overlay contains a cell outside occupancy grid")

    occupied_min = origin - resolution / 2.0
    occupied_max = origin + (np.asarray(grid.shape, dtype=np.float64) - 0.5) * resolution
    padding = max(0.5, resolution)
    world_min = occupied_min - padding
    world_max = occupied_max + padding
    span = world_max - world_min

    canvas_width, canvas_height = CANVAS_SIZE
    plot_left = float(_MARGIN_PX)
    plot_top = float(_HEADER_PX + 50)
    plot_width = float(canvas_width - 2 * _MARGIN_PX)
    plot_height = float(canvas_height - plot_top - _MARGIN_PX)
    scale = min(plot_width / span[0], plot_height / span[1])
    rendered_width = span[0] * scale
    rendered_height = span[1] * scale
    canvas_left = plot_left + (plot_width - rendered_width) / 2.0
    canvas_top = plot_top + (plot_height - rendered_height) / 2.0

    image = Image.new("RGB", CANVAS_SIZE, _BACKGROUND)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    draw.text((_MARGIN_PX, 16), title, fill=(20, 25, 30), font=font)
    for index, metric in enumerate(overlay.metrics):
        column = index % 4
        row = index // 4
        draw.text(
            (_MARGIN_PX + column * 275, 38 + row * 17),
            metric,
            fill=(55, 61, 67),
            font=font,
        )

    map_size = (
        max(1, int(round(grid.shape[0] * resolution * scale))),
        max(1, int(round(grid.shape[1] * resolution * scale))),
    )
    occupancy_image = Image.fromarray(_occupancy_rgb(grid, 0, 1, 2), mode="RGB").resize(
        map_size,
        resample=Image.Resampling.NEAREST,
    )
    occ_left = _world_to_pixel(
        occupied_min,
        world_min,
        world_max,
        canvas_left,
        canvas_top,
        scale,
    )[0]
    occ_top = _world_to_pixel(
        np.asarray((occupied_min[0], occupied_max[1])),
        world_min,
        world_max,
        canvas_left,
        canvas_top,
        scale,
    )[1]
    image.paste(occupancy_image, (occ_left, occ_top))

    newly_inflated = (inflated == 1) & (grid != 1)
    inflation_rgba = np.zeros((*grid.shape, 4), dtype=np.uint8)
    inflation_rgba[newly_inflated] = _INFLATED_COLOR
    inflation_image = Image.fromarray(
        np.flipud(np.transpose(inflation_rgba, (1, 0, 2))),
        mode="RGBA",
    ).resize(map_size, resample=Image.Resampling.NEAREST)
    image.paste(inflation_image, (occ_left, occ_top), inflation_image)
    draw = ImageDraw.Draw(image)

    def cell_pixel(cell: Sequence[int]) -> tuple[int, int]:
        world_xy = origin + np.asarray(cell, dtype=np.float64) * resolution
        return _world_to_pixel(
            world_xy,
            world_min,
            world_max,
            canvas_left,
            canvas_top,
            scale,
        )

    start_pixel = cell_pixel(overlay.start_cell)
    goal_pixel = cell_pixel(overlay.goal_cell)
    _draw_dashed_line(
        draw,
        start_pixel,
        goal_pixel,
        fill=_STRAIGHT_COLOR,
        width=3,
    )
    oracle_pixels = [cell_pixel(cell) for cell in overlay.oracle_path_cells]
    if len(oracle_pixels) > 1:
        draw.line(oracle_pixels, fill=_ORACLE_COLOR, width=4, joint="curve")

    start_radius = 8
    draw.ellipse(
        (
            start_pixel[0] - start_radius,
            start_pixel[1] - start_radius,
            start_pixel[0] + start_radius,
            start_pixel[1] + start_radius,
        ),
        fill=_START_COLOR,
    )
    heading_world = origin + np.asarray(overlay.start_cell) * resolution + 0.5 * np.asarray(
        (math.cos(overlay.start_yaw_rad), math.sin(overlay.start_yaw_rad))
    )
    heading_pixel = _world_to_pixel(
        heading_world,
        world_min,
        world_max,
        canvas_left,
        canvas_top,
        scale,
    )
    draw.line((start_pixel, heading_pixel), fill=_START_COLOR, width=5)
    goal_radius = 9
    draw.ellipse(
        (
            goal_pixel[0] - goal_radius,
            goal_pixel[1] - goal_radius,
            goal_pixel[0] + goal_radius,
            goal_pixel[1] + goal_radius,
        ),
        fill=_GOAL_COLOR,
    )
    draw.rectangle(
        (
            int(round(canvas_left)),
            int(round(canvas_top)),
            int(round(canvas_left + rendered_width)),
            int(round(canvas_top + rendered_height)),
        ),
        outline=(145, 151, 157),
        width=2,
    )
    draw.text(
        (_MARGIN_PX, canvas_height - 24),
        "dark=obstacle  red=safety inflation  amber=straight  blue=A*  green=start  magenta=goal",
        fill=(70, 76, 82),
        font=font,
    )

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".partial")
    image.save(temporary, format="PNG")
    temporary.replace(destination)
    return destination, {name: True for name in sorted(TASK_CARD_LAYERS)}
