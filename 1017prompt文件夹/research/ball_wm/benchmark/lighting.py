"""Versioned Isaac lighting for fixed-camera visual benchmark runs."""

from __future__ import annotations

import math
from typing import Sequence


LEGACY_SUN_V1 = "benchmark_sun_v1"
FIXED_CAMERA_FILL_V1 = "fixed_camera_fill_v1"
LIGHTING_PROFILES = (LEGACY_SUN_V1, FIXED_CAMERA_FILL_V1)


def lamp_positions_world_m(
    start_position_world_m: Sequence[float],
    goal_position_world_m: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    """Place three lamps above the task axis at tested 3 m spacing."""
    start = tuple(float(value) for value in start_position_world_m)
    goal = tuple(float(value) for value in goal_position_world_m)
    if len(start) != 3 or len(goal) != 3 or not all(
        math.isfinite(value) for value in (*start, *goal)
    ):
        raise ValueError("lighting requires finite 3D start and goal positions")
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        raise ValueError("lighting task axis must have nonzero XY length")
    ux, uy = dx / length, dy / length
    cx, cy = (start[0] + goal[0]) / 2.0, (start[1] + goal[1]) / 2.0
    lamp_z = (start[2] + goal[2]) / 2.0 + 1.75
    return tuple((cx + offset * ux, cy + offset * uy, lamp_z)
                 for offset in (-3.0, 0.0, 3.0))


def install_fixed_camera_fill(
    stage: object,
    start_position_world_m: Sequence[float],
    goal_position_world_m: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    """Author lights before camera/render-product initialization."""
    from pxr import Gf, UsdGeom, UsdLux

    positions = lamp_positions_world_m(start_position_world_m, goal_position_world_m)
    for index, position in enumerate(positions):
        path = f"/World/BenchmarkLighting/FixedCameraFill_{index}"
        if stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"lighting prim already exists: {path}")
        light = UsdLux.SphereLight.Define(stage, path)
        light.CreateIntensityAttr(10000.0)
        light.CreateRadiusAttr(0.5)
        transform = UsdGeom.Xformable(light)
        transform.ClearXformOpOrder()
        transform.AddTranslateOp().Set(Gf.Vec3d(*position))
    return positions
