"""Deterministic BEV geometry plus lazy Isaac Sim occupancy adapters.

The pure functions in this module import only NumPy.  Isaac Sim modules are
loaded only by :class:`IsaacOccupancyProvider`, ``enable_camera_depth``, and
``read_camera_depth`` when an initialized simulator is actually in use.
"""

from contextlib import contextmanager
from collections import deque
from dataclasses import dataclass
import importlib
import math
from typing import Any, Iterator, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class BEVConfig:
    """Configuration for a square local BEV raster."""

    extent_m: float = 8.0
    resolution_m: float = 0.1
    free: int = 0
    occupied: int = 1
    unknown: int = 2

    def __post_init__(self) -> None:
        if self.extent_m <= 0.0 or self.resolution_m <= 0.0:
            raise ValueError("extent_m and resolution_m must be positive")
        for name in ("free", "occupied", "unknown"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
                raise ValueError(f"{name} must be an integer class value")
            if not 0 <= int(value) <= 255:
                raise ValueError(f"{name} must fit in uint8")

    @property
    def shape(self) -> tuple[int, int]:
        side = int(round(self.extent_m / self.resolution_m))
        return side, side

    @property
    def center(self) -> tuple[int, int]:
        return self.shape[0] // 2, self.shape[1] // 2


@dataclass(frozen=True)
class GlobalOccupancy:
    """A world-axis occupancy map compatible with :func:`crop_global_occupancy`.

    ``grid[0, 0]`` has its center at ``global_origin_xy``.  Grid axis 0
    increases with world X and grid axis 1 increases with world Y.  The
    adapter reshapes Isaac Occupancy Map's flat buffer as ``(width, height)``
    with the first (X) axis varying fastest, matching the buffer observed in
    Isaac Sim.  This is equivalent to a Fortran-order reshape.
    """

    grid: np.ndarray
    global_origin_xy: np.ndarray
    resolution_m: float

    def __post_init__(self) -> None:
        grid = np.asarray(self.grid)
        if grid.ndim != 2:
            raise ValueError(f"grid must have shape (X, Y), got {grid.shape}")
        origin = np.asarray(self.global_origin_xy, dtype=np.float64)
        if origin.shape != (2,):
            raise ValueError(f"global_origin_xy must have shape (2,), got {origin.shape}")
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        object.__setattr__(self, "grid", np.asarray(grid, dtype=np.uint8))
        object.__setattr__(self, "global_origin_xy", origin)


@dataclass(frozen=True)
class GlobalElevation:
    """World-axis lower/upper Z envelope for each global XY cell."""

    height_min: np.ndarray
    height_max: np.ndarray
    valid: np.ndarray
    global_origin_xy: np.ndarray
    resolution_m: float

    def __post_init__(self) -> None:
        minimum = np.asarray(self.height_min)
        maximum = np.asarray(self.height_max)
        valid = np.asarray(self.valid)
        if minimum.ndim != 2 or maximum.shape != minimum.shape or valid.shape != minimum.shape:
            raise ValueError("height_min, height_max, and valid must have the same 2-D shape")
        if minimum.dtype != np.float32 or maximum.dtype != np.float32:
            raise ValueError("height_min and height_max must have dtype float32")
        if valid.dtype != np.uint8 or not np.isin(valid, (0, 1)).all():
            raise ValueError("valid must have uint8 binary dtype")
        origin = np.asarray(self.global_origin_xy, dtype=np.float64)
        if origin.shape != (2,):
            raise ValueError(f"global_origin_xy must have shape (2,), got {origin.shape}")
        if not np.isfinite(origin).all() or self.resolution_m <= 0.0:
            raise ValueError("global_origin_xy must be finite and resolution_m must be positive")
        if not np.isfinite(minimum).all() or not np.isfinite(maximum).all():
            raise ValueError("height bounds must be finite")
        object.__setattr__(self, "height_min", minimum)
        object.__setattr__(self, "height_max", maximum)
        object.__setattr__(self, "valid", valid)
        object.__setattr__(self, "global_origin_xy", origin)


class AnalyticElevationUnsupportedError(RuntimeError):
    """The analytic elevation path cannot faithfully represent scene geometry."""


class AnalyticElevationCoverageError(ValueError):
    """Analytic geometry did not provide height truth for every free cell."""


def omap_buffer_to_global_occupancy(
    buffer: Any, dimensions: Sequence[int] | None = None
) -> np.ndarray:
    """Convert an Isaac OMap buffer to ``grid[world_x, world_y]`` labels.

    The OMap Generator returns a flat 2D buffer and dimensions ordered as
    ``(width, height, depth)``.  Isaac's buffer varies the first dimension
    (world X) fastest, so conversion uses a Fortran-order reshape to preserve
    the ``grid[world_x, world_y]`` convention required by
    :func:`crop_global_occupancy`.  Keep this helper pure and tested because
    an accidental C-order reshape rotates/aliases obstacle geometry.
    """

    values = np.asarray(buffer)
    if dimensions is None:
        if values.ndim != 2:
            raise ValueError("dimensions are required for a flat OMap buffer")
        shaped = values
    else:
        dims = tuple(int(value) for value in dimensions)
        if len(dims) < 2 or dims[0] <= 0 or dims[1] <= 0:
            raise ValueError(f"dimensions must start with two positive values, got {dims}")
        if values.size != dims[0] * dims[1]:
            raise ValueError(
                f"OMap buffer has {values.size} values, expected {dims[0] * dims[1]} "
                f"for dimensions {dims}"
            )
        shaped = values.reshape((dims[0], dims[1]), order="F")
    if not np.all(np.isfinite(shaped)):
        raise ValueError("OMap buffer must contain finite occupancy labels")
    rounded = np.rint(shaped)
    if not np.array_equal(shaped, rounded):
        raise ValueError("OMap buffer must contain integral occupancy labels")
    return np.asarray(rounded, dtype=np.uint8)


def _create_world_bbox_cache(usd_geom: Any, usd: Any) -> Any:
    """Create an OpenUSD cache for default-purpose collision geometry only."""

    return usd_geom.BBoxCache(usd.TimeCode.Default(), [usd_geom.Tokens.default_])


def _omap_transform_arguments(
    robot_origin: Any,
    scene_minimum: Any,
    scene_maximum: Any,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Build Isaac OMap's origin-relative bounds for a 2-D scan plane."""

    origin = np.asarray(robot_origin, dtype=np.float64)
    minimum = np.asarray(scene_minimum, dtype=np.float64)
    maximum = np.asarray(scene_maximum, dtype=np.float64)
    if any(value.shape != (3,) for value in (origin, minimum, maximum)):
        raise ValueError("OMap origin and scene bounds must each have shape (3,)")
    if not np.all(np.isfinite([origin, minimum, maximum])):
        raise ValueError("OMap origin and scene bounds must be finite")
    lower = minimum - origin
    upper = maximum - origin
    # Isaac's occupancy-map UI uses zero lower/upper Z offsets for generate2d,
    # retaining a robot-height 2-D slice instead of the scene's full Z bounds.
    lower[2] = 0.0
    upper[2] = 0.0
    return tuple(origin), tuple(lower), tuple(upper)


def _validate_external_scene_occupancy(
    grid: Any,
    config: BEVConfig,
    scene_root: str | None,
) -> None:
    """Reject external-scene OMaps that cannot provide obstacle labels."""

    occupancy = np.asarray(grid)
    if scene_root and not np.any(occupancy == config.occupied):
        raise RuntimeError(
            f"OMap generated no occupied cells for non-empty scene root {scene_root}; "
            "verify scene collision geometry and OMap scan height"
        )


def _base_xy(base_position: Any) -> np.ndarray:
    position = np.asarray(base_position, dtype=np.float64)
    if position.shape not in ((2,), (3,)):
        raise ValueError(f"base_position must have shape (2,) or (3,), got {position.shape}")
    return position[:2]


def _world_xy(points_world: Any) -> np.ndarray:
    points = np.asarray(points_world, dtype=np.float64)
    if points.ndim < 2 or points.shape[-1] < 2:
        raise ValueError("points_world must have shape (N, >=2)")
    return points[..., :2]


def _world_to_base_xy(points_world: Any, base_position: Any, base_yaw: float) -> np.ndarray:
    points = _world_xy(points_world)
    delta = points - _base_xy(base_position)
    yaw = float(base_yaw)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    # R_world_to_base = [[cos(yaw), sin(yaw)], [-sin(yaw), cos(yaw)]]
    return np.stack((cosine * delta[..., 0] + sine * delta[..., 1],
                     -sine * delta[..., 0] + cosine * delta[..., 1]), axis=-1)


def world_points_to_local_bev(
    points_world: Any, base_position: Any, base_yaw: float, config: BEVConfig
) -> tuple[np.ndarray, np.ndarray]:
    """Map world XY points to integer local BEV row and column indices.

    Base +X points toward decreasing image rows and base +Y toward decreasing
    image columns. Points outside the raster are returned as integer indices;
    callers that sample a raster should use the corresponding bounds check.
    """

    local = _world_to_base_xy(points_world, base_position, base_yaw)
    center_row, center_col = config.center
    rows = np.floor(center_row - local[..., 0] / config.resolution_m).astype(np.int64)
    cols = np.floor(center_col - local[..., 1] / config.resolution_m).astype(np.int64)
    return rows, cols


def _local_grid_centers(config: BEVConfig) -> np.ndarray:
    rows = np.arange(config.shape[0], dtype=np.float64)
    cols = np.arange(config.shape[1], dtype=np.float64)
    center_row, center_col = config.center
    local_x = (center_row - rows) * config.resolution_m
    local_y = (center_col - cols) * config.resolution_m
    grid_x, grid_y = np.meshgrid(local_x, local_y, indexing="ij")
    return np.stack((grid_x, grid_y), axis=-1)


def _local_to_world_xy(local_xy: np.ndarray, base_position: Any, base_yaw: float) -> np.ndarray:
    position = _base_xy(base_position)
    yaw = float(base_yaw)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    world_delta = np.stack(
        (cosine * local_xy[..., 0] - sine * local_xy[..., 1],
         sine * local_xy[..., 0] + cosine * local_xy[..., 1]),
        axis=-1,
    )
    return world_delta + position


def _nearest_indices(source_xy: np.ndarray, origin_xy: Any, resolution_m: float) -> tuple[np.ndarray, np.ndarray]:
    origin = np.asarray(origin_xy, dtype=np.float64)
    if origin.shape != (2,):
        raise ValueError(f"global_origin_xy must have shape (2,), got {origin.shape}")
    source_coords = (source_xy - origin) / resolution_m
    # floor(x + 0.5) gives deterministic nearest-neighbor sampling, including
    # coordinates just below zero, without NumPy's ties-to-even behavior.
    indices = np.floor(source_coords + 0.5).astype(np.int64)
    return indices[..., 0], indices[..., 1]


def _nearest_local_bev_indices(local_xy: np.ndarray, config: BEVConfig) -> tuple[np.ndarray, np.ndarray]:
    """Convert continuous local BEV coordinates to nearest cell indices."""

    center_row, center_col = config.center
    rows = np.floor(center_row - local_xy[..., 0] / config.resolution_m + 0.5).astype(np.int64)
    cols = np.floor(center_col - local_xy[..., 1] / config.resolution_m + 0.5).astype(np.int64)
    return rows, cols


def _sample_nearest(source: np.ndarray, rows: np.ndarray, cols: np.ndarray, unknown: int) -> np.ndarray:
    result = np.full(rows.shape, np.uint8(unknown), dtype=np.uint8)
    valid = (rows >= 0) & (rows < source.shape[0]) & (cols >= 0) & (cols < source.shape[1])
    result[valid] = np.asarray(source[rows[valid], cols[valid]], dtype=np.uint8)
    return result


def crop_global_occupancy(
    global_map: Any,
    global_origin_xy: Any,
    base_position: Any,
    base_yaw: float,
    config: BEVConfig,
) -> np.ndarray:
    """Crop a world-axis global occupancy map into the local BEV raster.

    ``global_origin_xy`` is the world XY coordinate of the global map's
    ``[0, 0]`` cell center. Global map axis 0 is world X and axis 1 is world Y.
    Samples use nearest-neighbor labels; any source coordinate outside the map
    is assigned ``config.unknown``.
    """

    source = np.asarray(global_map)
    if source.ndim != 2:
        raise ValueError(f"global_map must have shape (H, W), got {source.shape}")
    local_grid = _local_grid_centers(config)
    world_grid = _local_to_world_xy(local_grid, base_position, base_yaw)
    rows, cols = _nearest_indices(world_grid, global_origin_xy, config.resolution_m)
    return _sample_nearest(source, rows, cols, config.unknown)


def rasterize_analytic_height_envelope(
    primitives: Sequence[Mapping[str, Any]],
    global_origin_xy: Any,
    grid_shape: tuple[int, int],
    resolution_m: float,
) -> GlobalElevation:
    """Rasterize exact world-Z envelopes for analytic cylinders and spheres."""

    origin = np.asarray(global_origin_xy, dtype=np.float64)
    if origin.shape != (2,) or not np.isfinite(origin).all():
        raise ValueError("global_origin_xy must be a finite vector with shape (2,)")
    if (len(grid_shape) != 2 or any(isinstance(side, (bool, np.bool_))
                                    or not isinstance(side, (int, np.integer))
                                    or int(side) <= 0 for side in grid_shape)):
        raise ValueError(f"grid_shape must be a positive (H, W), got {grid_shape}")
    if resolution_m <= 0.0 or not np.isfinite(resolution_m):
        raise ValueError("resolution_m must be positive and finite")
    shape = tuple(int(side) for side in grid_shape)
    minimum = np.full(shape, np.inf, dtype=np.float32)
    maximum = np.full(shape, -np.inf, dtype=np.float32)
    valid = np.zeros(shape, dtype=np.uint8)
    rows = origin[0] + np.arange(shape[0], dtype=np.float64) * resolution_m
    cols = origin[1] + np.arange(shape[1], dtype=np.float64) * resolution_m
    grid_x, grid_y = np.meshgrid(rows, cols, indexing="ij")

    for primitive in primitives:
        kind = primitive.get("kind")
        center = np.asarray(primitive.get("center_xy"), dtype=np.float64)
        if center.shape != (2,) or not np.isfinite(center).all():
            raise ValueError("center_xy must be a finite vector with shape (2,)")
        radius = float(primitive["radius"])
        if radius <= 0.0 or not np.isfinite(radius):
            raise ValueError("primitive radius must be positive and finite")
        distance_sq = (grid_x - center[0]) ** 2 + (grid_y - center[1]) ** 2
        if kind == "cylinder":
            lower, upper = float(primitive["z_min"]), float(primitive["z_max"])
            if not np.isfinite([lower, upper]).all() or lower > upper:
                raise ValueError("cylinder z_min/z_max must be finite and ordered")
            inside = distance_sq <= radius * radius
            primitive_min = np.full(shape, lower, dtype=np.float32)
            primitive_max = np.full(shape, upper, dtype=np.float32)
        elif kind == "sphere":
            center_z = float(primitive["center_z"])
            if not np.isfinite(center_z):
                raise ValueError("sphere center_z must be finite")
            inside = distance_sq <= radius * radius
            half_height = np.sqrt(np.maximum(0.0, radius * radius - distance_sq))
            primitive_min = np.asarray(center_z - half_height, dtype=np.float32)
            primitive_max = np.asarray(center_z + half_height, dtype=np.float32)
        else:
            raise ValueError(f"unsupported analytic primitive kind: {kind!r}")
        valid_now = (valid == 1) & inside
        first = inside & (valid == 0)
        minimum[first] = primitive_min[first]
        maximum[first] = primitive_max[first]
        minimum[valid_now] = np.minimum(minimum[valid_now], primitive_min[valid_now])
        maximum[valid_now] = np.maximum(maximum[valid_now], primitive_max[valid_now])
        valid[inside] = 1
    minimum[valid == 0] = 0.0
    maximum[valid == 0] = 0.0
    return GlobalElevation(minimum, maximum, valid, origin, resolution_m)


def crop_global_elevation(
    elevation: GlobalElevation,
    base_position: Any,
    base_yaw: float,
    config: BEVConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Crop world-Z elevation into local BEV coordinates and subtract base Z."""

    if not isinstance(elevation, GlobalElevation):
        raise ValueError("elevation must be a GlobalElevation")
    position = np.asarray(base_position, dtype=np.float64)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("base_position must be a finite vector with shape (3,)")
    local_grid = _local_grid_centers(config)
    world_grid = _local_to_world_xy(local_grid, position, base_yaw)
    rows, cols = _nearest_indices(world_grid, elevation.global_origin_xy, elevation.resolution_m)
    shape = config.shape
    minimum = np.zeros(shape, dtype=np.float32)
    maximum = np.zeros(shape, dtype=np.float32)
    valid = np.zeros(shape, dtype=np.uint8)
    inside = (rows >= 0) & (rows < elevation.valid.shape[0]) & (cols >= 0) & (cols < elevation.valid.shape[1])
    valid[inside] = elevation.valid[rows[inside], cols[inside]]
    valid_cells = valid == 1
    minimum[valid_cells] = elevation.height_min[rows[valid_cells], cols[valid_cells]] - np.float32(position[2])
    maximum[valid_cells] = elevation.height_max[rows[valid_cells], cols[valid_cells]] - np.float32(position[2])
    return minimum, maximum, valid


def validate_elevation_coverage(
    elevation: GlobalElevation, occupancy: Any, config: BEVConfig
) -> None:
    """Require every free occupancy cell to have a corresponding elevation interval."""

    labels = np.asarray(occupancy)
    if labels.shape != elevation.valid.shape:
        raise ValueError("occupancy and elevation must have matching shapes")
    if labels.ndim != 2:
        raise ValueError("occupancy must be a 2-D array")
    missing = (labels == config.free) & (elevation.valid != 1)
    if np.any(missing):
        raise ValueError("free occupancy cells without valid elevation")


def obstacle_clearance(local_bev: Any, ball_radius: float, config: BEVConfig) -> float:
    """Return nearest occupied cell-center distance minus the ball radius."""

    bev = np.asarray(local_bev)
    if bev.shape != config.shape:
        raise ValueError(f"local_bev must have shape {config.shape}, got {bev.shape}")
    radius = float(ball_radius)
    if radius < 0.0:
        raise ValueError("ball_radius must be non-negative")
    occupied = np.argwhere(bev == config.occupied)
    if occupied.size == 0:
        return np.inf
    center_row, center_col = config.center
    local_x = (center_row - occupied[:, 0]) * config.resolution_m
    local_y = (center_col - occupied[:, 1]) * config.resolution_m
    distances = np.hypot(local_x, local_y)
    return float(np.min(distances) - radius)


def is_spawn_free(
    global_map: Any,
    global_origin_xy: Any,
    base_position: Any,
    ball_radius: float,
    config: BEVConfig,
) -> bool:
    """Return whether a world XY position is collision-free in the global map.

    The ball footprint is sampled at half-cell spacing, plus a quarter-cell
    safety margin, and every sampled cell must be ``config.free``.  Cells
    outside the global raster are never considered free.
    """

    source = np.asarray(global_map)
    if source.ndim != 2:
        raise ValueError(f"global_map must have shape (H, W), got {source.shape}")
    radius = float(ball_radius)
    if radius < 0.0:
        raise ValueError("ball_radius must be non-negative")
    center = _base_xy(base_position)
    half_step = config.resolution_m / 2.0
    margin = config.resolution_m * 0.25
    offsets_1d = np.arange(-radius, radius + half_step, half_step)
    grid_x, grid_y = np.meshgrid(offsets_1d, offsets_1d, indexing="ij")
    inside = grid_x**2 + grid_y**2 <= (radius + margin) ** 2
    offsets = np.stack((grid_x[inside], grid_y[inside]), axis=-1)
    world = offsets + center
    rows, cols = _nearest_indices(world, global_origin_xy, config.resolution_m)
    valid = (
        (rows >= 0)
        & (rows < source.shape[0])
        & (cols >= 0)
        & (cols < source.shape[1])
    )
    if not np.all(valid):
        return False
    cells = source[rows[valid], cols[valid]]
    return bool(np.all(cells == config.free))


def rasterize_analytic_primitives(
    primitives: Sequence[Mapping[str, Any]],
    global_origin_xy: Any,
    grid_shape: tuple[int, int],
    resolution_m: float,
    occupied: int,
    scan_z: float,
) -> np.ndarray:
    """Rasterize exact Cylinder/Sphere cross-sections into a global map.

    Each primitive is a mapping with ``kind`` in ``{"cylinder", "sphere"}``,
    world ``center_xy`` (and ``center_z`` for spheres), ``radius``, and for
    cylinders ``z_min``/``z_max``.  A cell is occupied when its center lies
    inside the analytic cross-section at ``scan_z``.  This bypasses PhysX
    collision approximations, so upright cylinders and spheres produce
    perfectly circular labels.
    """

    origin = np.asarray(global_origin_xy, dtype=np.float64)
    if origin.shape != (2,):
        raise ValueError(f"global_origin_xy must have shape (2,), got {origin.shape}")
    if len(grid_shape) != 2 or any(int(side) <= 0 for side in grid_shape):
        raise ValueError(f"grid_shape must be a positive (H, W), got {grid_shape}")
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    grid = np.zeros(tuple(int(side) for side in grid_shape), dtype=np.uint8)
    height, width = grid.shape
    scan_z = float(scan_z)

    for primitive in primitives:
        kind = primitive.get("kind")
        center = np.asarray(primitive.get("center_xy"), dtype=np.float64)
        if center.shape != (2,):
            raise ValueError("center_xy must have shape (2,)")
        radius = float(primitive["radius"])
        if radius <= 0.0:
            continue
        if kind == "cylinder":
            if not (float(primitive["z_min"]) <= scan_z <= float(primitive["z_max"])):
                continue
        elif kind == "sphere":
            delta_z = scan_z - float(primitive["center_z"])
            if abs(delta_z) > radius:
                continue
            radius = math.sqrt(max(0.0, radius * radius - delta_z * delta_z))
        else:
            continue

        center_i = int(math.floor((center[0] - origin[0]) / resolution_m + 0.5))
        center_j = int(math.floor((center[1] - origin[1]) / resolution_m + 0.5))
        half = int(math.ceil(radius / resolution_m)) + 1
        rows = np.arange(max(0, center_i - half), min(height, center_i + half + 1))
        cols = np.arange(max(0, center_j - half), min(width, center_j + half + 1))
        if rows.size == 0 or cols.size == 0:
            continue
        cell_x = origin[0] + rows * resolution_m
        cell_y = origin[1] + cols * resolution_m
        grid_x, grid_y = np.meshgrid(cell_x, cell_y, indexing="ij")
        inside = (grid_x - center[0]) ** 2 + (grid_y - center[1]) ** 2 <= radius * radius
        subgrid = grid[np.ix_(rows, cols)]
        subgrid[inside] = occupied
        grid[np.ix_(rows, cols)] = subgrid
    return grid


def rasterize_analytic_primitives_in_height_band(
    primitives: Sequence[Mapping[str, Any]],
    global_origin_xy: Any,
    grid_shape: tuple[int, int],
    resolution_m: float,
    occupied: int,
    lower_z: float,
    upper_z: float,
) -> np.ndarray:
    """Project analytic collision geometry intersecting a vertical body band.

    The returned footprint is the maximum horizontal cross-section attained
    anywhere in the inclusive world-Z interval ``[lower_z, upper_z]``.
    """

    lower = float(lower_z)
    upper = float(upper_z)
    if not np.isfinite(lower) or not np.isfinite(upper) or upper < lower:
        raise ValueError("height band must contain finite lower_z <= upper_z")

    projected: list[dict[str, Any]] = []
    for primitive in primitives:
        kind = primitive.get("kind")
        candidate = dict(primitive)
        if kind == "cylinder":
            if float(primitive["z_max"]) < lower or float(primitive["z_min"]) > upper:
                continue
            candidate["z_min"] = 0.0
            candidate["z_max"] = 0.0
        elif kind == "sphere":
            center_z = float(primitive["center_z"])
            radius = float(primitive["radius"])
            nearest_z = min(max(center_z, lower), upper)
            delta_z = nearest_z - center_z
            if abs(delta_z) > radius:
                continue
            candidate["radius"] = math.sqrt(max(0.0, radius * radius - delta_z * delta_z))
            candidate["center_z"] = 0.0
        else:
            continue
        projected.append(candidate)

    # Each projected primitive has its largest in-band cross-section at the
    # synthetic scan plane. Cylinders retain their full footprint.
    return rasterize_analytic_primitives(
        projected,
        global_origin_xy,
        grid_shape,
        resolution_m,
        occupied,
        scan_z=0.0,
    )


def collect_analytic_obstacles(
    stage: Any,
    scene_root: str,
    usd: Any,
    usd_geom: Any,
    usd_physics: Any,
) -> list[dict[str, Any]]:
    """Return world-space Cylinder/Sphere primitives that carry collision.

    Only visible prims under ``scene_root`` are considered.  Cylinders must be
    approximately upright (local Z aligned with world Z); tilted analytic
    shapes raise so the caller can fall back to OMap instead of emitting
    silently wrong labels.
    """

    scene_prim = stage.GetPrimAtPath(scene_root)
    if not scene_prim or not scene_prim.IsValid():
        raise RuntimeError(f"scene root does not exist: {scene_root}")
    cache = usd_geom.XformCache()
    primitives: list[dict[str, Any]] = []
    for prim in usd.PrimRange(scene_prim, usd.TraverseInstanceProxies()):
        if not (prim.IsA(usd_geom.Cylinder) or prim.IsA(usd_geom.Sphere)):
            continue
        imageable = usd_geom.Imageable(prim)
        if imageable.ComputeVisibility(usd.TimeCode.Default()) == usd_geom.Tokens.invisible:
            continue
        applied = set(prim.GetAppliedSchemas())
        has_collision = prim.HasAPI(usd_physics.CollisionAPI) or "PhysicsCollisionAPI" in applied
        if not has_collision:
            continue
        transform = cache.GetLocalToWorldTransform(prim)
        matrix = np.asarray(transform).reshape(4, 4).T
        translation = matrix[:3, 3]
        if prim.IsA(usd_geom.Cylinder):
            z_axis = matrix[:3, 2]
            if abs(float(z_axis[2])) < 0.99:
                raise AnalyticElevationUnsupportedError(
                    f"tilted analytic Cylinder is unsupported: {prim.GetPath()}; "
                    "use --bev_teacher omap for this scene"
                )
            cylinder = usd_geom.Cylinder(prim)
            radius = cylinder.GetRadiusAttr().Get()
            height = cylinder.GetHeightAttr().Get()
            if radius is None or height is None:
                continue
            primitives.append(
                {
                    "kind": "cylinder",
                    "center_xy": (float(translation[0]), float(translation[1])),
                    "radius": float(radius),
                    "z_min": float(translation[2]) - float(height) / 2.0,
                    "z_max": float(translation[2]) + float(height) / 2.0,
                }
            )
        else:
            sphere = usd_geom.Sphere(prim)
            radius = sphere.GetRadiusAttr().Get()
            if radius is None:
                continue
            primitives.append(
                {
                    "kind": "sphere",
                    "center_xy": (float(translation[0]), float(translation[1])),
                    "center_z": float(translation[2]),
                    "radius": float(radius),
                }
            )
    return primitives


def build_analytic_global_map(
    stage: Any,
    scene_root: str,
    robot_root: str,
    config: BEVConfig,
    body_radius_m: float,
    usd: Any,
    usd_geom: Any,
    usd_physics: Any,
) -> GlobalOccupancy:
    """Build a maximum projection over the spherical robot's body height."""

    body_radius = float(body_radius_m)
    if not np.isfinite(body_radius) or body_radius <= 0.0:
        raise ValueError("body_radius_m must be finite and positive")

    provider = IsaacOccupancyProvider(
        stage=stage,
        scene_root=scene_root,
        robot_root=robot_root,
        config=config,
    )
    minimum, maximum = provider._scene_bounds(usd_geom, usd)
    scan_origin = provider._robot_origin(usd_geom)
    resolution = config.resolution_m
    grid_shape = (
        int(math.ceil((maximum[0] - minimum[0]) / resolution)),
        int(math.ceil((maximum[1] - minimum[1]) / resolution)),
    )
    grid = rasterize_analytic_primitives_in_height_band(
        collect_analytic_obstacles(stage, scene_root, usd, usd_geom, usd_physics),
        minimum[:2] + resolution / 2.0,
        grid_shape,
        resolution,
        config.occupied,
        lower_z=float(scan_origin[2] - body_radius),
        upper_z=float(scan_origin[2] + body_radius),
    )
    _validate_external_scene_occupancy(grid, config, scene_root)
    return GlobalOccupancy(
        grid=grid,
        global_origin_xy=minimum[:2] + resolution / 2.0,
        resolution_m=resolution,
    )


def _build_analytic_global_elevation(
    stage: Any,
    scene_root: str,
    occupancy: GlobalOccupancy,
    config: BEVConfig,
    usd: Any,
    usd_geom: Any,
    usd_physics: Any,
) -> GlobalElevation:
    elevation = rasterize_analytic_height_envelope(
        collect_analytic_obstacles(stage, scene_root, usd, usd_geom, usd_physics),
        occupancy.global_origin_xy,
        occupancy.grid.shape,
        occupancy.resolution_m,
    )
    try:
        validate_elevation_coverage(elevation, occupancy.grid, config)
    except ValueError as exc:
        if "free occupancy cells" in str(exc):
            raise AnalyticElevationCoverageError(str(exc)) from exc
        raise
    return elevation


def build_global_elevation(
    mode: str,
    stage: Any,
    scene_root: str,
    robot_root: str,
    occupancy: GlobalOccupancy,
    config: BEVConfig,
    usd: Any,
    usd_geom: Any,
    usd_physics: Any,
    physics_sync: Any | None = None,
) -> GlobalElevation:
    """Build a fully-covered elevation teacher, failing closed on every backend error."""

    if mode not in {"auto", "analytic", "raycast"}:
        raise ValueError(f"unsupported elevation teacher mode: {mode!r}")
    if not isinstance(occupancy, GlobalOccupancy):
        raise ValueError("occupancy must be a GlobalOccupancy")

    def raycast() -> GlobalElevation:
        return IsaacHeightProvider(
            stage=stage,
            scene_root=scene_root,
            robot_root=robot_root,
            occupancy=occupancy,
            config=config,
            physics_sync=physics_sync,
        ).build_global_map()

    if mode == "analytic":
        elevation = _build_analytic_global_elevation(
            stage, scene_root, occupancy, config, usd, usd_geom, usd_physics
        )
    elif mode == "raycast":
        elevation = raycast()
    else:
        try:
            elevation = _build_analytic_global_elevation(
                stage, scene_root, occupancy, config, usd, usd_geom, usd_physics
            )
        except (AnalyticElevationUnsupportedError, AnalyticElevationCoverageError):
            elevation = raycast()

    validate_elevation_coverage(elevation, occupancy.grid, config)
    return elevation


def _pose_xy_yaw(pose: Any, name: str) -> tuple[np.ndarray, float]:
    value = np.asarray(pose, dtype=np.float64)
    if value.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {value.shape}")
    return value[:2], float(value[2])


def warp_history_mask(
    previous_mask: Any,
    previous_pose: Any,
    current_pose: Any,
    config: BEVConfig,
) -> np.ndarray:
    """Warp a prior local mask into the current base frame by nearest neighbor."""

    source = np.asarray(previous_mask)
    if source.shape != config.shape:
        raise ValueError(f"previous_mask must have shape {config.shape}, got {source.shape}")
    previous_position, previous_yaw = _pose_xy_yaw(previous_pose, "previous_pose")
    current_position, current_yaw = _pose_xy_yaw(current_pose, "current_pose")
    current_local_grid = _local_grid_centers(config)
    world_grid = _local_to_world_xy(current_local_grid, current_position, current_yaw)
    previous_local_grid = _world_to_base_xy(world_grid, previous_position, previous_yaw)
    rows, cols = _nearest_local_bev_indices(previous_local_grid, config)
    return _sample_nearest(source, rows, cols, config.unknown)


class VisibilityHistory:
    """Bounded original-frame observations accumulated in the current base frame."""

    def __init__(self, max_frames: int, config: BEVConfig) -> None:
        if isinstance(max_frames, bool) or not isinstance(max_frames, (int, np.integer)):
            raise ValueError("max_frames must be a positive integer")
        if max_frames <= 0:
            raise ValueError("max_frames must be a positive integer")
        self.max_frames = int(max_frames)
        self.config = config
        self._observations: deque[tuple[np.ndarray, np.ndarray]] = deque(maxlen=self.max_frames)

    def update(self, current_mask: Any, current_pose: Any) -> np.ndarray:
        """Append one observation and directly warp the retained original masks."""

        mask = np.asarray(current_mask)
        if mask.shape != self.config.shape or mask.dtype != np.uint8:
            raise ValueError(
                f"current_mask must be uint8 with shape {self.config.shape}, "
                f"got {mask.dtype} {mask.shape}"
            )
        if not set(np.unique(mask).tolist()).issubset({0, 1}):
            raise ValueError("current_mask must be binary")
        position, yaw = _pose_xy_yaw(current_pose, "current_pose")
        pose = np.array([position[0], position[1], yaw], dtype=np.float64)
        self._observations.append((mask.copy(), pose))

        observed = np.zeros(self.config.shape, dtype=np.uint8)
        for observation_mask, observation_pose in self._observations:
            warped = warp_history_mask(
                observation_mask,
                observation_pose,
                pose,
                self.config,
            )
            observed[warped == 1] = 1
        return observed


def _as_matrix(value: Any, shape: tuple[int, int], name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != shape or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} must be a finite matrix with shape {shape}, got {matrix.shape}")
    return matrix


def depth_to_visible_mask(
    depth: Any,
    intrinsics: Any,
    camera_to_base: Any,
    config: BEVConfig,
) -> np.ndarray:
    """Rasterize valid camera depth rays into a binary local visibility mask.

    ``camera_to_base`` explicitly defines the camera-frame convention.  Depth
    pixels are back-projected as ``[(u-cx)d/fx, (v-cy)d/fy, d, 1]`` and then
    transformed into the base frame; this function makes no Isaac camera-axis
    assumption.  Every valid ray is sampled from camera origin to endpoint at
    half-cell spacing, so the output represents observed space, not occupancy.
    """

    depths = np.asarray(depth, dtype=np.float64)
    if depths.ndim != 2:
        raise ValueError(f"depth must have shape (H, W), got {depths.shape}")
    calibration = _as_matrix(intrinsics, (3, 3), "intrinsics")
    transform = _as_matrix(camera_to_base, (4, 4), "camera_to_base")
    if np.linalg.det(calibration) == 0.0:
        raise ValueError("intrinsics must be invertible")

    mask = np.zeros(config.shape, dtype=np.uint8)
    valid_rows, valid_cols = np.nonzero(np.isfinite(depths) & (depths > 0.0))
    if valid_rows.size == 0:
        return mask

    inverse_intrinsics = np.linalg.inv(calibration)
    camera_origin = transform[:3, 3]
    for row, col in zip(valid_rows, valid_cols):
        pixel = np.array([float(col), float(row), 1.0])
        endpoint_camera = inverse_intrinsics @ (pixel * depths[row, col])
        endpoint_homogeneous = transform @ np.append(endpoint_camera, 1.0)
        if endpoint_homogeneous[3] == 0.0:
            continue
        endpoint = endpoint_homogeneous[:3] / endpoint_homogeneous[3]
        direction = endpoint[:2] - camera_origin[:2]
        length = float(np.linalg.norm(direction))
        if length == 0.0:
            points = camera_origin[:2][None, :]
        else:
            spacing = config.resolution_m / 2.0
            fractions = np.minimum(np.arange(0.0, length + spacing, spacing) / length, 1.0)
            points = camera_origin[:2] + fractions[:, None] * direction
        rows, cols = _nearest_local_bev_indices(points, config)
        in_bounds = (
            (rows >= 0)
            & (rows < config.shape[0])
            & (cols >= 0)
            & (cols < config.shape[1])
        )
        mask[rows[in_bounds], cols[in_bounds]] = 1
    return mask


def _camera_path(camera: Any) -> str:
    path = getattr(camera, "prim_path", None)
    if path is None and hasattr(camera, "get_prim_path"):
        path = camera.get_prim_path()
    return str(path) if path is not None else "<unknown camera>"


def _extract_depth_from_frame(frame: Mapping[str, Any], camera_path: str) -> np.ndarray:
    """Validate and normalize one Isaac camera frame's depth annotator output."""

    keys = sorted(str(key) for key in frame.keys()) if isinstance(frame, Mapping) else []
    if not isinstance(frame, Mapping) or "distance_to_image_plane" not in frame:
        raise RuntimeError(
            f"camera {camera_path} has no distance_to_image_plane frame entry; available keys: {keys}"
        )
    depth = np.asarray(frame["distance_to_image_plane"], dtype=np.float32)
    if depth.ndim != 2 or 0 in depth.shape:
        raise RuntimeError(
            f"camera {camera_path} distance_to_image_plane must have a non-empty 2D shape; "
            f"got {depth.shape}; available keys: {keys}"
        )
    return depth


def enable_camera_depth(camera: Any) -> None:
    """Enable Isaac's image-plane-distance annotator after camera initialization."""

    camera.add_distance_to_image_plane_to_frame()


def read_camera_depth(camera: Any) -> np.ndarray:
    """Read a validated float32 depth image without advancing the simulation."""

    return _extract_depth_from_frame(camera.get_current_frame(), _camera_path(camera))


@contextmanager
def _robot_collisions_disabled(
    robot_prim: Any,
    collision_api: Any,
    prim_range: Any,
) -> Iterator[None]:
    """Disable existing robot collision attributes and restore them in ``finally``."""

    modified: list[tuple[Any, Any]] = []
    try:
        for prim in prim_range(robot_prim):
            if not prim.HasAPI(collision_api):
                continue
            attribute = collision_api(prim).GetCollisionEnabledAttr()
            if not attribute.IsValid():
                continue
            original = attribute.Get()
            attribute.Set(False)
            modified.append((attribute, original))
        yield
    finally:
        for attribute, original in reversed(modified):
            attribute.Set(original)


class IsaacHeightProvider:
    """Build static world-Z envelopes with lazy Isaac Sim vertical ray queries.

    ``raycast_heights`` is an optional test adapter taking ``(origin, direction,
    distance)`` and returning world-Z hit positions.  Production callers leave
    it unset so PhysX is imported only while building the static map.
    """

    def __init__(
        self,
        stage: Any,
        scene_root: str,
        robot_root: str,
        occupancy: GlobalOccupancy,
        config: BEVConfig,
        physics_sync: Any | None = None,
        raycast_heights: Any | None = None,
    ) -> None:
        if not isinstance(occupancy, GlobalOccupancy):
            raise ValueError("occupancy must be a GlobalOccupancy")
        if not np.isclose(occupancy.resolution_m, config.resolution_m):
            raise ValueError("occupancy resolution must match config resolution")
        self.stage = stage
        self.scene_root = scene_root
        self.robot_root = robot_root
        self.occupancy = occupancy
        self.config = config
        self.physics_sync = physics_sync
        self.raycast_heights = raycast_heights

    def _scene_bounds(self, usd_geom: Any, usd: Any) -> tuple[np.ndarray, np.ndarray]:
        scene_prim = self.stage.GetPrimAtPath(self.scene_root)
        if not scene_prim or not scene_prim.IsValid():
            raise RuntimeError(f"scene root does not exist: {self.scene_root}")
        world_range = _create_world_bbox_cache(usd_geom, usd).ComputeWorldBound(scene_prim).GetRange()
        minimum = np.asarray(world_range.GetMin(), dtype=np.float64)
        maximum = np.asarray(world_range.GetMax(), dtype=np.float64)
        if minimum.shape != (3,) or maximum.shape != (3,) or not np.all(np.isfinite([minimum, maximum])):
            raise RuntimeError(f"scene root has no finite world bounds: {self.scene_root}")
        if np.any(maximum <= minimum):
            raise RuntimeError(f"scene root has empty world bounds: {self.scene_root}")
        return minimum, maximum

    @staticmethod
    def _physx_raycast_heights(physx: Any) -> Any:
        """Adapt PhysX ``raycast_all`` callbacks to finite world-Z hit values."""

        import carb

        query = physx.get_physx_scene_query_interface()

        def query_heights(origin: Any, direction: Any, distance: float) -> list[float]:
            heights: list[float] = []

            def report_hit(hit: Any) -> bool:
                position = hit.get("position") if isinstance(hit, Mapping) else getattr(hit, "position", None)
                if position is None:
                    raise RuntimeError("PhysX raycast hit has no world position")
                z = float(position[2])
                if not np.isfinite(z):
                    raise RuntimeError("PhysX raycast hit has non-finite world Z")
                heights.append(z)
                return True

            query.raycast_all(
                carb.Float3(*[float(value) for value in origin]),
                carb.Float3(*[float(value) for value in direction]),
                float(distance),
                report_hit,
                True,
            )
            return heights

        return query_heights

    def _build_global_map(
        self,
        usd: Any,
        usd_geom: Any,
        usd_physics: Any,
        raycast_heights: Any,
    ) -> GlobalElevation:
        minimum, maximum = self._scene_bounds(usd_geom, usd)
        robot_prim = self.stage.GetPrimAtPath(self.robot_root)
        if not robot_prim or not robot_prim.IsValid():
            raise RuntimeError(f"robot root does not exist: {self.robot_root}")

        shape = self.occupancy.grid.shape
        height_min = np.zeros(shape, dtype=np.float32)
        height_max = np.zeros(shape, dtype=np.float32)
        valid = np.zeros(shape, dtype=np.uint8)
        ray_origin_z = float(maximum[2] + self.config.resolution_m)
        ray_lower_z = min(float(minimum[2]), 0.0)
        ray_distance = float(maximum[2] - ray_lower_z + 2.0 * self.config.resolution_m)

        with _robot_collisions_disabled(robot_prim, usd_physics.CollisionAPI, usd.PrimRange):
            if self.physics_sync is not None:
                self.physics_sync()
            for row in range(shape[0]):
                for col in range(shape[1]):
                    origin = np.array(
                        [
                            self.occupancy.global_origin_xy[0] + row * self.occupancy.resolution_m,
                            self.occupancy.global_origin_xy[1] + col * self.occupancy.resolution_m,
                            ray_origin_z,
                        ],
                        dtype=np.float64,
                    )
                    hits = list(raycast_heights(origin, (0.0, 0.0, -1.0), ray_distance))
                    if not hits:
                        continue
                    values = np.asarray(hits, dtype=np.float64)
                    if values.ndim != 1 or not np.all(np.isfinite(values)):
                        raise RuntimeError("PhysX raycast returned non-finite vertical hit heights")
                    height_min[row, col] = np.float32(np.min(values))
                    height_max[row, col] = np.float32(np.max(values))
                    valid[row, col] = 1

        return GlobalElevation(
            height_min=height_min,
            height_max=height_max,
            valid=valid,
            global_origin_xy=self.occupancy.global_origin_xy,
            resolution_m=self.occupancy.resolution_m,
        )

    def build_global_map(self) -> GlobalElevation:
        """Query all occupancy-aligned cells after temporarily excluding the robot."""

        # Deliberately lazy: ordinary test and geometry-only processes do not
        # need Isaac Sim, PhysX, carb, or the USD Python bindings installed.
        import omni.physx
        from pxr import Usd, UsdGeom, UsdPhysics

        raycast_heights = self.raycast_heights or self._physx_raycast_heights(omni.physx)
        return self._build_global_map(Usd, UsdGeom, UsdPhysics, raycast_heights)


class IsaacOccupancyProvider:
    """Generate one static global occupancy map through Isaac Sim 4.5 OMap.

    ``bounds_xy`` is optional explicit ``((min_x, min_y), (max_x, max_y))``.
    Without it, the provider takes the world bounding box of ``scene_root`` and
    adds a one metre XY pad.  The map origin is the center of cell ``[0, 0]``:
    ``(min_x + resolution / 2, min_y + resolution / 2)``.
    """

    def __init__(
        self,
        stage: Any,
        scene_root: str | None,
        robot_root: str,
        config: BEVConfig,
        bounds_xy: Any | None = None,
        physics_sync: Any | None = None,
    ) -> None:
        self.stage = stage
        self.scene_root = scene_root
        self.robot_root = robot_root
        self.config = config
        self.bounds_xy = self._normalize_bounds_xy(bounds_xy) if bounds_xy is not None else None
        self.physics_sync = physics_sync

    @staticmethod
    def _normalize_bounds_xy(bounds_xy: Any) -> tuple[np.ndarray, np.ndarray]:
        bounds = np.asarray(bounds_xy, dtype=np.float64)
        if bounds.shape != (2, 2) or not np.all(np.isfinite(bounds)):
            raise ValueError("bounds_xy must have shape ((min_x, min_y), (max_x, max_y))")
        if np.any(bounds[1] <= bounds[0]):
            raise ValueError("bounds_xy maximum must exceed minimum on both axes")
        return bounds[0], bounds[1]

    def _scene_bounds(self, usd_geom: Any, usd: Any) -> tuple[np.ndarray, np.ndarray]:
        if not self.scene_root:
            if self.bounds_xy is None:
                raise RuntimeError("scene_root or explicit bounds_xy is required for occupancy generation")
            raise RuntimeError("scene_root is required to determine OMap Z bounds")
        scene_prim = self.stage.GetPrimAtPath(self.scene_root)
        if not scene_prim or not scene_prim.IsValid():
            raise RuntimeError(f"scene root does not exist: {self.scene_root}")
        world_range = _create_world_bbox_cache(usd_geom, usd).ComputeWorldBound(scene_prim).GetRange()
        minimum = np.asarray(world_range.GetMin(), dtype=np.float64)
        maximum = np.asarray(world_range.GetMax(), dtype=np.float64)
        if minimum.shape != (3,) or maximum.shape != (3,) or not np.all(np.isfinite([minimum, maximum])):
            raise RuntimeError(f"scene root has no finite world bounds: {self.scene_root}")
        if np.any(maximum <= minimum):
            raise RuntimeError(f"scene root has empty world bounds: {self.scene_root}")
        if self.bounds_xy is None:
            minimum[:2] -= 1.0
            maximum[:2] += 1.0
        else:
            minimum[:2], maximum[:2] = self.bounds_xy
        return minimum, maximum

    def _robot_origin(self, usd_geom: Any) -> np.ndarray:
        robot_prim = self.stage.GetPrimAtPath(self.robot_root)
        if not robot_prim or not robot_prim.IsValid():
            raise RuntimeError(f"robot root does not exist: {self.robot_root}")
        # The robot root is frequently an identity/root XForm while the actual
        # ball pose lives on base_link.  OMap's scan plane is set by this
        # origin's Z, so using the root would slice at the ground (z=0) and
        # miss every obstacle.  Prefer base_link when it exists.
        origin_prim = robot_prim
        base_link_path = f"{self.robot_root}/base_link"
        base_link_prim = self.stage.GetPrimAtPath(base_link_path)
        if base_link_prim and base_link_prim.IsValid():
            origin_prim = base_link_prim
        transform = usd_geom.XformCache().GetLocalToWorldTransform(origin_prim)
        origin = np.asarray(transform.ExtractTranslation(), dtype=np.float64)
        if origin.shape != (3,) or not np.all(np.isfinite(origin)):
            raise RuntimeError(f"robot root has no finite reset position: {self.robot_root}")
        return origin

    def build_global_map(self) -> GlobalOccupancy:
        """Generate OMap with robot collision excluded, restoring it on every exit."""

        # Deliberately lazy: importing this module remains possible in ordinary
        # Python/pytest processes which have no Isaac Sim runtime.
        import omni.kit.app

        extension_manager = omni.kit.app.get_app().get_extension_manager()
        extension_manager.set_extension_enabled_immediate("isaacsim.asset.gen.omap", True)
        import omni.physx
        import omni.usd
        from pxr import Usd, UsdGeom, UsdPhysics

        _omap = _import_omap_binding()

        minimum, maximum = self._scene_bounds(UsdGeom, Usd)
        robot_prim = self.stage.GetPrimAtPath(self.robot_root)
        origin = self._robot_origin(UsdGeom)
        generator = _omap.Generator(
            omni.physx.acquire_physx_interface(), omni.usd.get_context().get_stage_id()
        )
        generator.update_settings(self.config.resolution_m, self.config.occupied, self.config.free, self.config.unknown)
        transform_origin, lower_bound, upper_bound = _omap_transform_arguments(
            origin,
            minimum,
            maximum,
        )
        generator.set_transform(transform_origin, lower_bound, upper_bound)
        with _robot_collisions_disabled(robot_prim, UsdPhysics.CollisionAPI, Usd.PrimRange):
            if self.physics_sync is not None:
                # collisionEnabled=False removes the shape from the tensor
                # view, but OMap queries the PhysX scene directly.  Let the
                # simulation rebuild its collision scene before generating.
                self.physics_sync()
            generator.generate2d()
        grid = omap_buffer_to_global_occupancy(generator.get_buffer(), generator.get_dimensions())
        values, counts = np.unique(grid, return_counts=True)
        print(
            "OMap: dimensions="
            f"{tuple(int(value) for value in generator.get_dimensions()[:2])} "
            f"bounds_xy=({tuple(float(value) for value in generator.get_min_bound()[:2])}, "
            f"{tuple(float(value) for value in generator.get_max_bound()[:2])}) "
            f"class_counts={dict(zip(values.tolist(), counts.tolist()))}",
            flush=True,
        )
        _validate_external_scene_occupancy(grid, self.config, self.scene_root)
        return GlobalOccupancy(
            grid=grid,
            global_origin_xy=(
                np.asarray(generator.get_min_bound(), dtype=np.float64)[:2]
                + self.config.resolution_m / 2.0
            ),
            resolution_m=self.config.resolution_m,
        )


def _import_omap_binding(import_module: Any = importlib.import_module) -> Any:
    """Load the Isaac Sim 4.5 OMap binary from its public bindings package."""

    return import_module("isaacsim.asset.gen.omap.bindings._omap")
