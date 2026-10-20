"""Offline failure diagnosis for closed-loop Ball PointGoal benchmark runs.

The benchmark bundle already contains enough measured telemetry to separate the
planner reference, dynamic proxy, tracker and termination signals without
starting Isaac Sim again.  This script deliberately reports approximations as
such: the frozen body-band occupancy is used for static reference checks, and
the measured drive profile is used for braking and yaw-rate proxies.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
from scipy.ndimage import distance_transform_edt

from benchmark.drive_model import DriveModel
from benchmark.grid_planning import inflate_obstacles
from benchmark.kinodynamic_planning import sample_motion_primitive


LOW_SPEED_THRESHOLD_M_S = 0.05
STALL_MIN_DURATION_S = 1.0
CURVATURE_TURN_THRESHOLD_1_M = 0.20
TRACKING_ERROR_THRESHOLD_M = 0.20
DEFAULT_BALL_RADIUS_M = 0.40
DEFAULT_SAFETY_MARGIN_M = 0.10

CSV_FIELDS = (
    "time_s",
    "x_m",
    "y_m",
    "yaw_rad",
    "goal_distance_m",
    "reference_x_m",
    "reference_y_m",
    "reference_yaw_rad",
    "cross_track_error_m",
    "heading_error_rad",
    "obstacle_clearance_m",
    "commanded_speed_m_s",
    "actual_speed_m_s",
    "auxiliary_joint_velocity_rad_s",
    "required_yaw_rate_rad_s",
    "reference_curvature_1_m",
    "waypoint_index",
    "contact_flag",
    "collision_flag",
    "control_saturation_flag",
    "braking_state_code",
    "stopped_cycles",
)


def _as_array(payload: Mapping[str, Any], name: str, *, dtype: Any = float) -> np.ndarray:
    if name not in payload:
        raise ValueError(f"episode bundle is missing {name!r}")
    array = np.asarray(payload[name], dtype=dtype)
    if array.size == 0:
        raise ValueError(f"episode bundle field {name!r} is empty")
    return array


def _wrap_angle(value: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(value) + math.pi) % (2.0 * math.pi) - math.pi


def _polyline_distances(points_xy: np.ndarray) -> np.ndarray:
    if points_xy.ndim != 2 or points_xy.shape[1] != 2 or len(points_xy) == 0:
        raise ValueError("points_xy must have shape (N, 2), N >= 1")
    if len(points_xy) == 1:
        return np.zeros(1, dtype=np.float64)
    return np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points_xy, axis=0), axis=1)))
    )


def _polyline_nearest(points_xy: np.ndarray, reference_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return nearest distance and segment index for each point."""

    if reference_xy.ndim != 2 or reference_xy.shape[1] != 2 or len(reference_xy) < 2:
        raise ValueError("reference_xy must have shape (N, 2), N >= 2")
    starts = reference_xy[:-1]
    vectors = reference_xy[1:] - starts
    lengths_sq = np.einsum("ij,ij->i", vectors, vectors)
    delta = points_xy[:, None, :] - starts[None, :, :]
    projection = np.divide(
        np.einsum("nsi,si->ns", delta, vectors),
        lengths_sq[None, :],
        out=np.zeros((len(points_xy), len(starts)), dtype=np.float64),
        where=lengths_sq[None, :] > 1e-15,
    )
    projection = np.clip(projection, 0.0, 1.0)
    nearest = starts[None, :, :] + projection[:, :, None] * vectors[None, :, :]
    distances = np.linalg.norm(points_xy[:, None, :] - nearest, axis=2)
    indices = np.argmin(distances, axis=1)
    return distances[np.arange(len(points_xy)), indices], indices


def _reference_curvature(reference_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xy = reference_pose[:, :2]
    yaw = np.unwrap(reference_pose[:, 2])
    spacing = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    curvature = np.zeros(len(reference_pose), dtype=np.float64)
    if len(reference_pose) > 1:
        delta_yaw = np.diff(yaw)
        segment_curvature = np.divide(
            delta_yaw,
            spacing,
            out=np.zeros_like(delta_yaw),
            where=spacing > 1e-9,
        )
        curvature[1:] = segment_curvature
        curvature[0] = segment_curvature[0]
    return curvature, spacing


def _sample_polyline(points_xy: np.ndarray, spacing_m: float = 0.025) -> np.ndarray:
    samples: list[np.ndarray] = []
    for left, right in zip(points_xy[:-1], points_xy[1:]):
        distance = float(np.linalg.norm(right - left))
        count = max(1, int(math.ceil(distance / spacing_m)))
        samples.extend(np.linspace(left, right, count, endpoint=False))
    samples.append(points_xy[-1])
    return np.asarray(samples, dtype=np.float64)


def _sample_reference_motion(
    reference_pose: np.ndarray,
    reference_command: np.ndarray,
    drive_model: DriveModel,
) -> np.ndarray:
    """Reconstruct the planner's constant-curvature primitives.

    Linear interpolation between Hybrid-A* waypoints can cut through an
    obstacle even when every constant-curvature primitive used by the planner
    was free.  The command at waypoint ``i`` is the primitive from waypoint
    ``i`` to ``i + 1``; the final row is the terminal zero command.
    """

    if len(reference_pose) < 2 or reference_command.shape != (len(reference_pose), 2):
        return _sample_polyline(reference_pose[:, :2])
    samples: list[np.ndarray] = []
    for index in range(len(reference_pose) - 1):
        main, auxiliary = (float(value) for value in reference_command[index])
        if abs(main) <= 1e-12:
            segment = np.asarray((reference_pose[index], reference_pose[index + 1]), dtype=np.float64)
        else:
            direction = 1.0 if main > 0.0 else -1.0
            segment = sample_motion_primitive(
                start_pose=reference_pose[index],
                signed_arc_length_m=direction * 0.3,
                curvature_1_m=drive_model.curvature(main, auxiliary),
                # Match the planner's collision sampling spacing.  A finer
                # linearized check would test a different reference than the
                # one actually accepted by Hybrid A*.
                sample_spacing_m=0.05,
            )
        if index:
            segment = segment[1:]
        samples.extend(segment)
    samples.append(reference_pose[-1])
    return np.asarray(samples, dtype=np.float64)


def _world_cells(points_xy: np.ndarray, origin_xy: np.ndarray, resolution_m: float) -> np.ndarray:
    return np.floor((points_xy - origin_xy[None, :]) / resolution_m + 0.5).astype(np.int64)


def _map_reference_metrics(
    reference_xy: np.ndarray,
    *,
    grid: np.ndarray,
    origin_xy: np.ndarray,
    resolution_m: float,
    safety_margin_m: float,
    reference_samples_xy: np.ndarray | None = None,
) -> dict[str, Any]:
    planning_grid = np.where(np.asarray(grid) == 0, 0, 1).astype(np.uint8)
    inflated = inflate_obstacles(
        planning_grid,
        radius_m=DEFAULT_BALL_RADIUS_M + safety_margin_m,
        resolution_m=resolution_m,
        occupied=1,
    )
    samples = (
        _sample_polyline(reference_xy)
        if reference_samples_xy is None
        else np.asarray(reference_samples_xy, dtype=np.float64)
    )
    cells = _world_cells(samples, origin_xy, resolution_m)
    inside = (
        (cells[:, 0] >= 0)
        & (cells[:, 0] < inflated.shape[0])
        & (cells[:, 1] >= 0)
        & (cells[:, 1] < inflated.shape[1])
    )
    occupied = np.zeros(len(cells), dtype=bool)
    occupied[inside] = inflated[cells[inside, 0], cells[inside, 1]] == 1
    free_distance = distance_transform_edt(inflated == 0) * resolution_m
    map_clearance = np.zeros(len(cells), dtype=np.float64)
    map_clearance[inside] = free_distance[cells[inside, 0], cells[inside, 1]]
    if np.any(inside):
        min_clearance = float(np.min(map_clearance[inside]))
    else:
        min_clearance = 0.0
    return {
        "reference_map_collision_free": bool(np.all(inside) and not np.any(occupied)),
        "reference_out_of_bounds_samples": int(np.count_nonzero(~inside)),
        "reference_occupied_samples": int(np.count_nonzero(occupied)),
        "reference_min_map_clearance_m": min_clearance,
        "reference_map_clearance_definition": (
            "distance to a grid inflated by ball_radius + safety_margin; "
            "discrete conservative proxy, not a continuous contact distance"
        ),
    }


def _effective_deceleration(drive_model: DriveModel) -> float:
    accelerations = [
        sample.speed_m_s**2 / (2.0 * sample.stopping_distance_m)
        for sample in drive_model.braking_samples
        if sample.speed_m_s > 1e-9 and sample.stopping_distance_m > 1e-9
    ]
    if not accelerations:
        return float("nan")
    return float(min(accelerations))


def _longest_true_duration(mask: np.ndarray, time_s: np.ndarray) -> tuple[float, float]:
    if len(mask) != len(time_s):
        raise ValueError("mask and time_s must have equal length")
    if len(mask) < 2:
        return (0.0, 0.0)
    longest = 0.0
    total = 0.0
    start: float | None = None
    for index, active in enumerate(mask):
        if active and start is None:
            start = float(time_s[index])
        if (not active or index == len(mask) - 1) and start is not None:
            end_index = index if active and index == len(mask) - 1 else index - 1
            end = float(time_s[end_index])
            duration = max(0.0, end - start)
            total += duration
            longest = max(longest, duration)
            start = None
    return total, longest


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def _load_occupancy(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    with np.load(path, allow_pickle=False) as archive:
        grid = np.asarray(archive["grid"], dtype=np.uint8)
        origin = np.asarray(archive["global_origin_xy"], dtype=np.float64)
        resolution = float(np.asarray(archive["resolution_m"]).reshape(-1)[0])
    if grid.ndim != 2 or origin.shape != (2,) or not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError(f"invalid occupancy map: {path}")
    return grid, origin, resolution


def _pad_action(values: np.ndarray, length: int, *, initial: float = math.nan) -> np.ndarray:
    result = np.full(length, initial, dtype=np.float64)
    if len(values):
        result[1 : min(length, len(values) + 1)] = np.asarray(values[: length - 1], dtype=np.float64)
    return result


def _read_action_metric(metrics: Mapping[str, Any], name: str, length: int) -> np.ndarray:
    values = np.asarray(metrics.get(name, []), dtype=np.float64)
    if values.size == 0:
        return np.full(length, np.nan, dtype=np.float64)
    return _pad_action(values, length)


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in CSV_FIELDS} for row in rows)


def _classify_failure(
    *,
    metrics: Mapping[str, Any],
    summary: Mapping[str, Any],
) -> tuple[str, str]:
    if bool(summary["initial_contact"]) or bool(summary["initial_collision"]):
        return "scene_initialization", "episode starts with contact or collision truth"
    if not bool(summary["reference_map_collision_free"]):
        return "planner_reference_infeasible", "reference path intersects the inflated occupancy map"
    termination = str(metrics.get("termination_reason", "unknown"))
    if termination == "collision":
        if float(summary["max_cross_track_error_m"]) > TRACKING_ERROR_THRESHOLD_M:
            return "trajectory_tracking_collision", "reference is map-feasible but actual tracking error grows before collision"
        return "dynamic_or_clearance_collision", "reference is map-feasible but actual footprint clearance reaches zero"
    if termination == "timeout":
        if bool(summary["persistent_contact"]):
            return "contact_stall_timeout", "timeout includes persistent physical contact"
        if float(summary["longest_stalled_duration_s"]) >= STALL_MIN_DURATION_S:
            return "tracking_stall_timeout", "long low-speed interval prevents useful progress"
        if float(summary["max_cross_track_error_m"]) > TRACKING_ERROR_THRESHOLD_M:
            return "tracking_progress_timeout", "timeout follows large reference-tracking error without contact"
        return "speed_or_dynamic_timeout", "path is feasible but progress is too slow for the budget"
    if termination == "runtime_error":
        return "runtime_error", "runtime terminated before a navigation conclusion"
    if termination == "success":
        return "none", "episode reached the goal"
    return "unknown", f"unrecognized termination reason: {termination}"


def analyze_episode(
    episode_dir: Path,
    *,
    occupancy_path: Path,
    drive_profile_path: Path,
    safety_margin_m: float = DEFAULT_SAFETY_MARGIN_M,
    low_speed_threshold_m_s: float = LOW_SPEED_THRESHOLD_M_S,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metrics = _load_json(episode_dir / "metrics.json")
    manifest = _load_json(episode_dir / "manifest.json")
    episode_id = episode_dir.name.removeprefix("episode_")
    matching = [
        item for item in manifest.get("episodes", [])
        if isinstance(item, dict) and item.get("episode_id") == episode_id
    ]
    if len(matching) != 1:
        raise ValueError(f"episode manifest does not uniquely resolve {episode_id!r}")
    episode = matching[0]
    with np.load(episode_dir / "states.npz", allow_pickle=False) as archive:
        states = {name: np.asarray(archive[name]) for name in archive.files}
    grid, origin, resolution = _load_occupancy(occupancy_path)
    drive_model = DriveModel.from_json(drive_profile_path)

    position = _as_array(states, "position_world_xy", dtype=np.float64)
    time_s = _as_array(states, "simulation_time_s", dtype=np.float64)
    yaw = _as_array(states, "yaw_world_rad", dtype=np.float64)
    linear_velocity = _as_array(states, "linear_velocity_world_m_s", dtype=np.float64)
    joint_velocity = _as_array(states, "joint_velocity_rad_s", dtype=np.float64)
    applied_action = _as_array(states, "applied_action", dtype=np.float64)
    clearance = _as_array(states, "clearance_m", dtype=np.float64)
    physical_contact = _as_array(states, "physical_contact", dtype=bool)
    collision = _as_array(states, "collision", dtype=bool)
    if position.shape[0] != time_s.shape[0]:
        raise ValueError("position and simulation time must be state-aligned")
    reference_pose = _as_array(states, "kinodynamic_reference_pose", dtype=np.float64)
    reference_xy = reference_pose[:, :2]
    reference_curvature, reference_spacing = _reference_curvature(reference_pose)
    reference_s = _polyline_distances(reference_xy)
    reference_command = np.asarray(states.get("kinodynamic_reference_command", []), dtype=np.float64)
    reference_motion = _sample_reference_motion(reference_pose, reference_command, drive_model)
    map_metrics = _map_reference_metrics(
        reference_xy,
        grid=grid,
        origin_xy=origin,
        resolution_m=resolution,
        safety_margin_m=safety_margin_m,
        reference_samples_xy=reference_motion[:, :2],
    )
    nearest_distance, nearest_segment = _polyline_nearest(position, reference_xy)

    action_count = max(0, len(position) - 1)
    reference_index = np.asarray(states.get("policy_reference_index", np.zeros(action_count)), dtype=np.int64)
    reference_index = np.clip(reference_index[:action_count], 0, len(reference_pose) - 1)
    cross_track = np.asarray(states.get("policy_cross_track_error_m", nearest_distance[1:]), dtype=np.float64)
    cross_track = cross_track[:action_count]
    cross_track = np.where(np.isfinite(cross_track), cross_track, nearest_distance[1 : action_count + 1])
    # Recompute heading error from the persisted world yaw and reference pose.
    # The legacy metrics field is policy-aligned and was zero-filled for this
    # reference policy, so using it would hide the actual turn error.
    heading_error = np.zeros(len(position), dtype=np.float64)
    for index in range(len(position)):
        reference_row = reference_pose[0] if index == 0 else reference_pose[reference_index[index - 1]]
        heading_error[index] = float(_wrap_angle(yaw[index] - reference_row[2]))
    command_curvature = _read_action_metric(metrics, "command_curvature_1_m", len(position))
    actual_speed = np.linalg.norm(linear_velocity[:, :2], axis=1)
    command_speed = np.zeros(len(position), dtype=np.float64)
    required_yaw_rate = np.zeros(len(position), dtype=np.float64)
    saturation = np.zeros(len(position), dtype=bool)
    for index, command in enumerate(applied_action):
        main = float(command[0])
        auxiliary = float(command[1])
        if abs(main) > 1e-12:
            command_speed[index] = drive_model.speed(main, auxiliary)
            curvature = drive_model.curvature(main, auxiliary)
            required_yaw_rate[index] = abs(command_speed[index] * curvature)
        saturation[index] = (
            abs(main) >= 2.0 - 1e-6
            or abs(auxiliary) >= math.radians(30.0) - 1e-6
            or abs(float(joint_velocity[index, 1])) >= 0.30 - 1e-3
        )
    planner_curvature_for_rows = np.zeros(len(position), dtype=np.float64)
    planner_curvature_for_rows[0] = reference_curvature[0]
    for index in range(1, len(position)):
        planner_curvature_for_rows[index] = reference_curvature[reference_index[index - 1]]
    if np.any(np.isfinite(command_curvature[1:])):
        planner_curvature_for_rows[1:] = np.where(
            np.isfinite(command_curvature[1:]), command_curvature[1:], planner_curvature_for_rows[1:]
        )

    goal_xy = np.asarray(episode["goal_position_world_m"], dtype=np.float64)[:2]
    goal_distance = np.linalg.norm(position - goal_xy[None, :], axis=1)
    low_speed = actual_speed < float(low_speed_threshold_m_s)
    stalled_total, stalled_longest = _longest_true_duration(low_speed, time_s)
    persistent_contact = bool(np.count_nonzero(physical_contact) >= max(5, int(0.1 * len(physical_contact))))
    initial_clearance = float(clearance[0])
    if reference_command.ndim == 2 and len(reference_command):
        reference_speeds = np.asarray(
            [drive_model.speed(float(main), float(aux)) if abs(float(main)) > 1e-12 else 0.0 for main, aux in reference_command],
            dtype=np.float64,
        )
        reference_braking = np.asarray(
            [drive_model.stopping_distance(float(speed), "reverse" if main < 0.0 else "forward") if abs(float(main)) > 1e-12 else 0.0 for speed, (main, _) in zip(reference_speeds, reference_command)],
            dtype=np.float64,
        )
        reference_yaw_rate = np.asarray(
            [abs(speed * drive_model.curvature(float(main), float(aux))) if abs(float(main)) > 1e-12 else 0.0 for speed, (main, aux) in zip(reference_speeds, reference_command)],
            dtype=np.float64,
        )
    else:
        reference_speeds = np.zeros(0, dtype=np.float64)
        reference_braking = np.zeros(0, dtype=np.float64)
        reference_yaw_rate = np.zeros(0, dtype=np.float64)
    turn_indices = np.flatnonzero(np.abs(reference_curvature) >= CURVATURE_TURN_THRESHOLD_1_M)
    first_turn_s = float(reference_s[turn_indices[0]]) if len(turn_indices) else float("inf")
    effective_deceleration = _effective_deceleration(drive_model)
    max_reference_braking = float(np.max(reference_braking)) if len(reference_braking) else 0.0
    max_reference_yaw_rate = float(np.max(reference_yaw_rate)) if len(reference_yaw_rate) else 0.0
    max_observed_yaw_rate = float(np.max(np.abs(_as_array(states, "angular_velocity_world_rad_s", dtype=np.float64)[:, 2])))
    initial_collision = bool(collision[0])
    initial_contact = bool(physical_contact[0])
    max_cross_track = float(np.max(cross_track)) if len(cross_track) else float(np.max(nearest_distance))
    rms_cross_track = float(np.sqrt(np.mean(np.square(cross_track)))) if len(cross_track) else float("nan")
    max_heading_error = float(np.nanmax(np.abs(heading_error))) if np.any(np.isfinite(heading_error)) else float("nan")
    rms_heading_error = float(np.sqrt(np.nanmean(np.square(heading_error)))) if np.any(np.isfinite(heading_error)) else float("nan")
    last_window_start = max(0.0, float(time_s[-1]) - 5.0)
    last_window = time_s >= last_window_start
    summary: dict[str, Any] = {
        "episode_id": str(metrics.get("episode_id", episode.get("episode_id", episode_dir.name))),
        "difficulty": str(metrics.get("difficulty", episode.get("difficulty", "unknown"))),
        "termination_reason": str(metrics.get("termination_reason", "unknown")),
        "success": bool(float(metrics.get("success", 0.0))),
        "initial_clearance_m": initial_clearance,
        "initial_contact": initial_contact,
        "initial_collision": initial_collision,
        "reference_length_m": float(reference_s[-1]),
        "reference_waypoint_count": int(len(reference_pose)),
        "reference_min_waypoint_spacing_m": float(np.min(reference_spacing)) if len(reference_spacing) else 0.0,
        "reference_max_waypoint_spacing_m": float(np.max(reference_spacing)) if len(reference_spacing) else 0.0,
        "reference_max_abs_curvature_1_m": float(np.max(np.abs(reference_curvature))),
        "reference_max_heading_change_rad": float(np.max(np.abs(np.diff(np.unwrap(reference_pose[:, 2]))))) if len(reference_pose) > 1 else 0.0,
        "reference_first_turn_distance_m": first_turn_s,
        "reference_required_braking_distance_m": max_reference_braking,
        "reference_required_yaw_rate_rad_s": max_reference_yaw_rate,
        "effective_deceleration_proxy_m_s2": effective_deceleration,
        "available_distance_before_first_turn_m": first_turn_s,
        "braking_distance_exceeds_first_turn_distance": bool(math.isfinite(first_turn_s) and max_reference_braking > first_turn_s),
        "reference_max_observed_command_yaw_rate_rad_s": max_reference_yaw_rate,
        "actual_max_yaw_rate_rad_s": max_observed_yaw_rate,
        "max_cross_track_error_m": max_cross_track,
        "rms_cross_track_error_m": rms_cross_track,
        "max_heading_error_rad": max_heading_error,
        "rms_heading_error_rad": rms_heading_error,
        "control_saturation_ratio": float(np.mean(saturation)),
        "actual_min_clearance_m": float(np.min(clearance)),
        "actual_min_clearance_time_s": float(time_s[int(np.argmin(clearance))]),
        "physical_contact_count": int(np.count_nonzero(physical_contact)),
        "collision_count": int(np.count_nonzero(collision)),
        "first_contact_time_s": float(time_s[np.flatnonzero(physical_contact)[0]]) if np.any(physical_contact) else None,
        "contact_duration_s": float(np.sum(np.diff(time_s)[physical_contact[:-1]])) if np.any(physical_contact[:-1]) else 0.0,
        "persistent_contact": persistent_contact,
        "final_goal_distance_m": float(goal_distance[-1]),
        "last_5s_mean_speed_m_s": float(np.mean(actual_speed[last_window])) if np.any(last_window) else 0.0,
        "last_5s_min_speed_m_s": float(np.min(actual_speed[last_window])) if np.any(last_window) else 0.0,
        "stalled_duration_s": stalled_total,
        "longest_stalled_duration_s": stalled_longest,
        "low_speed_threshold_m_s": float(low_speed_threshold_m_s),
        "reference_path_collision_free": bool(map_metrics["reference_map_collision_free"]),
        **map_metrics,
    }
    category, evidence = _classify_failure(metrics=metrics, summary=summary)
    summary["suspected_failure_category"] = category
    summary["classification_evidence"] = evidence
    summary["contacted_object"] = None
    summary["contacted_object_note"] = "contact pair/object names are not persisted in this benchmark bundle"

    rows: list[dict[str, Any]] = []
    state_reference_index = np.zeros(len(position), dtype=np.int64)
    if action_count:
        state_reference_index[1:] = reference_index
    for index in range(len(position)):
        reference_row = reference_pose[state_reference_index[index]]
        row = {
            "time_s": float(time_s[index]),
            "x_m": float(position[index, 0]),
            "y_m": float(position[index, 1]),
            "yaw_rad": float(yaw[index]),
            "goal_distance_m": float(goal_distance[index]),
            "reference_x_m": float(reference_row[0]),
            "reference_y_m": float(reference_row[1]),
            "reference_yaw_rad": float(reference_row[2]),
            "cross_track_error_m": float(cross_track[index - 1]) if index > 0 and index - 1 < len(cross_track) else float(nearest_distance[index]),
            "heading_error_rad": float(heading_error[index]) if np.isfinite(heading_error[index]) else float(_wrap_angle(yaw[index] - reference_row[2])),
            "obstacle_clearance_m": float(clearance[index]),
            "commanded_speed_m_s": float(command_speed[index]),
            "actual_speed_m_s": float(actual_speed[index]),
            "auxiliary_joint_velocity_rad_s": float(joint_velocity[index, 1]),
            "required_yaw_rate_rad_s": float(required_yaw_rate[index]),
            "reference_curvature_1_m": float(planner_curvature_for_rows[index]),
            "waypoint_index": int(state_reference_index[index]),
            "contact_flag": int(physical_contact[index]),
            "collision_flag": int(collision[index]),
            "control_saturation_flag": int(saturation[index]),
            "braking_state_code": int(states.get("policy_braking_state_code", np.zeros(action_count, dtype=np.int8))[index - 1]) if index > 0 and action_count else 0,
            "stopped_cycles": int(states.get("policy_stopped_cycles", np.zeros(action_count, dtype=np.int64))[index - 1]) if index > 0 and action_count else 0,
        }
        rows.append(row)
    return summary, rows


def _find_occupancy(scene_uri: str, *, data_root: Path, scene_index: Path) -> Path:
    index = _load_json(scene_index)
    records = index.get("scenes", [])
    matches = [item for item in records if isinstance(item, dict) and item.get("scene_uri") == scene_uri]
    if len(matches) != 1:
        raise ValueError(f"scene index does not uniquely resolve {scene_uri!r}")
    uri = str(matches[0]["occupancy_uri"])
    prefix = "${DATA_ROOT}/"
    if uri.startswith(prefix):
        return (data_root / uri[len(prefix) :]).resolve()
    return Path(uri).expanduser().resolve()


def analyze_run(
    run_root: Path,
    *,
    data_root: Path,
    scene_index: Path,
    drive_profile_path: Path,
    output_root: Path | None = None,
) -> dict[str, Any]:
    output = (output_root or run_root / "diagnostics").resolve()
    output.mkdir(parents=True, exist_ok=True)
    episode_summaries: list[dict[str, Any]] = []
    for episode_dir in sorted(run_root.glob("episode_*/")):
        if not (episode_dir / "states.npz").is_file():
            continue
        manifest = _load_json(episode_dir / "manifest.json")
        episodes = manifest.get("episodes", [])
        episode_id = episode_dir.name.removeprefix("episode_")
        matching = [
            item for item in episodes
            if isinstance(item, dict) and item.get("episode_id") == episode_id
        ]
        if len(matching) != 1:
            raise ValueError(
                f"episode manifest does not uniquely resolve {episode_id!r}"
            )
        scene_uri = str(matching[0]["scene_uri"])
        occupancy = _find_occupancy(scene_uri, data_root=data_root, scene_index=scene_index)
        summary, rows = analyze_episode(
            episode_dir,
            occupancy_path=occupancy,
            drive_profile_path=drive_profile_path,
        )
        _write_csv(output / episode_dir.name / "episode_diagnostics.csv", rows)
        (output / episode_dir.name / "diagnostics_summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        episode_summaries.append(summary)
    result = {
        "schema_version": "ball_pointgoal_failure_diagnostics_v1",
        "run_root": str(run_root.resolve()),
        "episode_count": len(episode_summaries),
        "episodes": episode_summaries,
        "category_counts": {
            category: sum(item["suspected_failure_category"] == category for item in episode_summaries)
            for category in sorted({item["suspected_failure_category"] for item in episode_summaries})
        },
    }
    (output / "diagnostics_summary.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return result


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument(
        "--scene-index",
        type=Path,
        default=Path("research/ball_wm/configs/benchmark_v1/scene_index.json"),
    )
    parser.add_argument(
        "--drive-profile",
        type=Path,
        default=Path("research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json"),
    )
    parser.add_argument("--output-root", type=Path, default=None)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    result = analyze_run(
        args.run_root.resolve(),
        data_root=args.data_root.resolve(),
        scene_index=args.scene_index.resolve(),
        drive_profile_path=args.drive_profile.resolve(),
        output_root=None if args.output_root is None else args.output_root.resolve(),
    )
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
