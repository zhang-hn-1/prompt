"""Offline body-band geometry labels for fixed local candidate centerlines."""

from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt


HORIZON_M = 4.0
STEP_M = 0.1
OFFSET_M = 2.3
RAMP_M = 3.0
REQUIRED_CLEARANCE_M = 0.6


def candidate_centerline(start_world_xy: np.ndarray, yaw_world_rad: float,
                         side: str) -> np.ndarray:
    if side not in ("straight", "left", "right"):
        raise ValueError("side must be straight, left, or right")
    start = np.asarray(start_world_xy, dtype=np.float64)
    if start.shape != (2,) or not np.isfinite(start).all() or not np.isfinite(yaw_world_rad):
        raise ValueError("start and yaw must be finite")
    x = np.linspace(0.0, HORIZON_M, round(HORIZON_M / STEP_M) + 1)
    t = np.clip(x / RAMP_M, 0, 1)
    lateral = OFFSET_M * t**2 * (3 - 2 * t)
    if side == "straight":
        lateral *= 0
    elif side == "right":
        lateral *= -1
    c, s = np.cos(yaw_world_rad), np.sin(yaw_world_rad)
    forward = np.array([c, s])
    left = np.array([-s, c])
    return start + x[:, None] * forward + lateral[:, None] * left


def clearance_grid(grid: np.ndarray, resolution_m: float) -> np.ndarray:
    """Distance to known occupied cells; no invented obstacle at map edge."""
    cells = np.asarray(grid)
    resolution = float(resolution_m)
    if cells.ndim != 2 or resolution <= 0 or not np.isfinite(resolution):
        raise ValueError("grid must be 2-D with positive finite resolution")
    if not np.isin(cells, (0, 1, 2)).all():
        raise ValueError("grid may only contain free/occupied/unknown labels 0/1/2")
    if not np.any(cells == 1):
        result = np.full(cells.shape, np.inf, dtype=np.float64)
    else:
        padding = int(np.ceil(REQUIRED_CLEARANCE_M / resolution)) + 2
        unoccupied = np.pad(cells != 1, padding, mode="constant", constant_values=True)
        distance = distance_transform_edt(unoccupied) * resolution
        result = distance[padding:-padding, padding:-padding]
    result[cells == 2] = np.nan
    return result


def classify_curve(points_world_xy: np.ndarray, clearance_m: np.ndarray,
                   global_origin_xy: np.ndarray, resolution_m: float,
                   required_clearance_m: float = REQUIRED_CLEARANCE_M) -> dict:
    points = np.asarray(points_world_xy, dtype=np.float64)
    clearance = np.asarray(clearance_m, dtype=np.float64)
    origin = np.asarray(global_origin_xy, dtype=np.float64)
    resolution = float(resolution_m)
    required = float(required_clearance_m)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points):
        raise ValueError("points must be nonempty N×2")
    if (clearance.ndim != 2 or origin.shape != (2,) or resolution <= 0
            or not np.isfinite(required) or required <= 0):
        raise ValueError("invalid map geometry")
    indices = np.rint((points - origin) / resolution).astype(np.int64)
    min_seen = np.inf
    for k, (i, j) in enumerate(indices):
        if i < 0 or j < 0 or i >= clearance.shape[0] or j >= clearance.shape[1]:
            return {"label": "unknown", "first_event_sample": k,
                    "min_known_clearance_m": float(min_seen) if np.isfinite(min_seen) else None}
        value = clearance[i, j]
        if np.isnan(value):
            return {"label": "unknown", "first_event_sample": k,
                    "min_known_clearance_m": float(min_seen) if np.isfinite(min_seen) else None}
        min_seen = min(min_seen, value)
        if value < required:
            return {"label": "blocked", "first_event_sample": k,
                    "min_known_clearance_m": float(min_seen)}
    return {"label": "clear", "first_event_sample": None,
            "min_known_clearance_m": float(min_seen) if np.isfinite(min_seen) else None}
