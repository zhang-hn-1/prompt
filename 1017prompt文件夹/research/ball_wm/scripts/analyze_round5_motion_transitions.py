"""Round 5 motion measurement, finite-horizon tables, and one stateful model."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Iterable, Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from benchmark.drive_model import DriveModel  # noqa: E402


PROTOCOL_SCHEMA = "round5_measurement_protocol_v1"
M1_SCHEMA = "round5_linear_state_delta_v1"
STATE_NAMES = (
    "main_joint_velocity_rad_s",
    "auxiliary_joint_rad",
    "auxiliary_joint_velocity_rad_s",
    "body_forward_velocity_m_s",
    "body_lateral_velocity_m_s",
    "world_z_angular_velocity_rad_s",
    "final_main_rad_s",
    "final_auxiliary_rad",
)
REQUEST_NAMES = ("requested_main_rad_s", "requested_auxiliary_rad")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"could not load JSON: {path}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def _json_value(value: object) -> object:
    if isinstance(value, np.ndarray):
        return [_json_value(item) for item in value.tolist()]
    if isinstance(value, (np.floating, float)):
        scalar = float(value)
        return scalar if math.isfinite(scalar) else None
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def _atomic_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(_json_value(dict(payload)), ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            encoded = {}
            for key in fields:
                value = _json_value(row.get(key))
                encoded[key] = (
                    json.dumps(value, ensure_ascii=False, sort_keys=True)
                    if isinstance(value, (dict, list))
                    else value
                )
            writer.writerow(encoded)


def _finite_array(
    value: object, *, name: str, columns: int | None = None, minimum: int = 2
) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    expected = 2 if columns is not None else 1
    if array.ndim != expected or (columns is not None and array.shape[1] != columns):
        raise ValueError(f"{name} has invalid shape")
    if array.shape[0] < minimum or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain finite samples")
    return array


def _angle_wrap(value: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(value) + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_wxyz(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion.T
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_measurement_protocol(path: str | Path) -> dict[str, object]:
    payload = _load_json(Path(path))
    if payload.get("schema_version") != PROTOCOL_SCHEMA:
        raise ValueError(f"measurement protocol must use {PROTOCOL_SCHEMA}")
    if payload.get("round4_decision_is_immutable") is not True:
        raise ValueError("Round 4 decision must remain immutable")
    horizons = np.asarray(payload.get("finite_horizons_s"), dtype=np.float64)
    if horizons.ndim != 1 or horizons.size < 1 or not np.isfinite(horizons).all():
        raise ValueError("finite horizons must be a finite vector")
    if np.any(horizons <= 0.0) or np.any(np.diff(horizons) <= 0.0):
        raise ValueError("finite horizons must be positive and increasing")
    return payload


def _valid_segments(mask: np.ndarray) -> Iterable[np.ndarray]:
    indices = np.flatnonzero(mask)
    if not indices.size:
        return
    starts = np.r_[0, np.flatnonzero(np.diff(indices) > 1) + 1]
    ends = np.r_[starts[1:], indices.size]
    for start, end in zip(starts, ends):
        yield indices[start:end]


def measure_motion_quantities(
    *,
    time_s: object,
    position_xy_m: object,
    linear_velocity_world_xy_m_s: object,
    root_quaternion_world_wxyz: object,
    angular_velocity_world_rad_s: object,
    minimum_planar_speed_m_s: float,
) -> dict[str, np.ndarray]:
    """Compute distinct orientation, angular-rate, course, and geometry signals."""

    time = _finite_array(time_s, name="time_s")
    position = _finite_array(position_xy_m, name="position_xy_m", columns=2)
    velocity = _finite_array(
        linear_velocity_world_xy_m_s,
        name="linear_velocity_world_xy_m_s",
        columns=2,
    )
    quaternion = _finite_array(
        root_quaternion_world_wxyz,
        name="root_quaternion_world_wxyz",
        columns=4,
    )
    angular = _finite_array(
        angular_velocity_world_rad_s,
        name="angular_velocity_world_rad_s",
        columns=3,
    )
    lengths = {len(time), len(position), len(velocity), len(quaternion), len(angular)}
    if len(lengths) != 1 or np.any(np.diff(time) <= 0.0):
        raise ValueError("motion arrays must align on strictly increasing time")
    minimum_speed = float(minimum_planar_speed_m_s)
    if not math.isfinite(minimum_speed) or minimum_speed <= 0.0:
        raise ValueError("minimum_planar_speed_m_s must be positive")

    yaw = np.unwrap(_yaw_from_wxyz(quaternion))
    yaw_derivative = np.gradient(yaw, time, edge_order=2)
    world_z = angular[:, 2].copy()
    speed = np.linalg.norm(velocity, axis=1)
    valid = speed >= minimum_speed
    course = np.full_like(speed, np.nan)
    course_rate = np.full_like(speed, np.nan)
    course_curvature = np.full_like(speed, np.nan)
    raw_course = np.arctan2(velocity[:, 1], velocity[:, 0])
    for segment in _valid_segments(valid):
        if segment.size < 2:
            continue
        segment_course = np.unwrap(raw_course[segment])
        course[segment] = segment_course
        edge_order = 2 if segment.size >= 3 else 1
        course_rate[segment] = np.gradient(
            segment_course, time[segment], edge_order=edge_order
        )
        course_curvature[segment] = course_rate[segment] / speed[segment]

    acceleration = np.column_stack(
        (
            np.gradient(velocity[:, 0], time, edge_order=2),
            np.gradient(velocity[:, 1], time, edge_order=2),
        )
    )
    kinematic = np.full_like(speed, np.nan)
    numerator = velocity[:, 0] * acceleration[:, 1] - velocity[:, 1] * acceleration[:, 0]
    kinematic[valid] = numerator[valid] / np.power(speed[valid], 3)

    three_point = np.full_like(speed, np.nan)
    for index in range(1, len(position) - 1):
        if not valid[index]:
            continue
        first = position[index] - position[index - 1]
        second = position[index + 1] - position[index]
        chord = first + second
        denominator = float(
            np.linalg.norm(first) * np.linalg.norm(second) * np.linalg.norm(chord)
        )
        if denominator <= 1e-10:
            continue
        cross = float(first[0] * second[1] - first[1] * second[0])
        three_point[index] = 2.0 * cross / denominator
    return {
        "root_yaw_rad": yaw,
        "yaw_derivative_rad_s": yaw_derivative,
        "world_z_angular_velocity_rad_s": world_z,
        "planar_speed_m_s": speed,
        "course_valid": valid,
        "course_rad": course,
        "course_rate_rad_s": course_rate,
        "course_curvature_1_m": course_curvature,
        "kinematic_curvature_1_m": kinematic,
        "three_point_curvature_1_m": three_point,
    }


def _samples_to_columns(samples: Sequence[Mapping[str, object]]) -> dict[str, np.ndarray]:
    if not samples:
        raise ValueError("trace requires samples")
    keys = (
        "time_s",
        "phase",
        "requested_auxiliary_rad",
        "auxiliary_joint_rad",
        "auxiliary_joint_velocity_rad_s",
        "position_xy_m",
        "linear_velocity_world_xy_m_s",
        "root_quaternion_world_wxyz",
        "angular_velocity_world_rad_s",
    )
    missing = [key for key in keys if any(key not in sample for sample in samples)]
    if missing:
        raise ValueError(f"trace samples are missing: {', '.join(missing)}")
    return {key: np.asarray([sample[key] for sample in samples]) for key in keys}


def classify_measurement_window(
    samples_or_columns: Mapping[str, object] | Sequence[Mapping[str, object]],
    protocol: Mapping[str, object],
) -> dict[str, object]:
    """Apply one uniform spatial/time window and return independent flags."""

    if isinstance(samples_or_columns, Mapping):
        columns = {key: np.asarray(value) for key, value in samples_or_columns.items()}
    else:
        columns = _samples_to_columns(samples_or_columns)
    config = protocol.get("analysis_window")
    if not isinstance(config, Mapping):
        raise ValueError("analysis_window configuration is required")
    time = np.asarray(columns["time_s"], dtype=np.float64)
    phases = np.asarray(columns["phase"], dtype=str)
    preferred = str(config.get("preferred_phase", "steer"))
    phase_indices = np.flatnonzero(phases == preferred)
    if not phase_indices.size:
        phase_indices = np.arange(len(time))
    end_time = float(time[phase_indices[-1]])
    maximum_duration = float(config["maximum_duration_s"])
    selected = phase_indices[time[phase_indices] >= end_time - maximum_duration - 1e-12]
    minimum_duration = float(config["minimum_duration_s"])
    window_duration = float(time[selected[-1]] - time[selected[0]]) if selected.size > 1 else 0.0

    motion = measure_motion_quantities(
        time_s=time,
        position_xy_m=columns["position_xy_m"],
        linear_velocity_world_xy_m_s=columns["linear_velocity_world_xy_m_s"],
        root_quaternion_world_wxyz=columns["root_quaternion_world_wxyz"],
        angular_velocity_world_rad_s=columns["angular_velocity_world_rad_s"],
        minimum_planar_speed_m_s=float(config["minimum_planar_speed_m_s"]),
    )
    requested_aux = np.asarray(columns["requested_auxiliary_rad"], dtype=np.float64)
    actual_aux = np.asarray(columns["auxiliary_joint_rad"], dtype=np.float64)
    aux_velocity = np.asarray(columns["auxiliary_joint_velocity_rad_s"], dtype=np.float64)
    position = np.asarray(columns["position_xy_m"], dtype=np.float64)
    speed = motion["planar_speed_m_s"]
    world_z = motion["world_z_angular_velocity_rad_s"]
    target_error = float(np.median(np.abs(actual_aux[selected] - requested_aux[selected])))
    target_reached = target_error <= float(config["maximum_auxiliary_median_target_error_rad"])
    aux_p95 = float(np.percentile(np.abs(aux_velocity[selected]), 95.0))
    median_speed = float(np.median(speed[selected]))
    speed_cv = float(np.std(speed[selected]) / max(median_speed, 1e-12))
    relative_time = time[selected] - time[selected[0]]
    world_z_trend = float(
        np.polyfit(relative_time, world_z[selected], 1)[0] if selected.size >= 3 else math.inf
    )
    state_stationary = bool(
        window_duration >= minimum_duration
        and aux_p95 <= float(config["maximum_auxiliary_p95_abs_velocity_rad_s"])
        and median_speed >= float(config["minimum_planar_speed_m_s"])
        and speed_cv <= float(config["maximum_speed_coefficient_of_variation"])
        and abs(world_z_trend) <= float(config["maximum_abs_world_z_rate_trend_rad_s2"])
    )
    path_length = float(
        np.linalg.norm(np.diff(position[selected], axis=0), axis=1).sum()
        if selected.size >= 2
        else 0.0
    )
    local_geometry = motion["three_point_curvature_1_m"][selected]
    finite_geometry = local_geometry[np.isfinite(local_geometry)]
    geometry_identifiable = bool(
        window_duration >= minimum_duration
        and path_length >= float(config["minimum_path_length_m"])
        and finite_geometry.size >= 3
    )
    local_course_rate = motion["course_rate_rad_s"][selected]
    finite_course_rate = local_course_rate[np.isfinite(local_course_rate)]
    course_peak_to_peak = (
        float(np.ptp(finite_course_rate)) if finite_course_rate.size else None
    )
    curvature_idr = (
        float(np.percentile(finite_geometry, 90) - np.percentile(finite_geometry, 10))
        if finite_geometry.size
        else None
    )
    oscillatory = bool(
        (course_peak_to_peak is not None and course_peak_to_peak >= float(config["oscillation_min_course_rate_peak_to_peak_rad_s"]))
        or (curvature_idr is not None and curvature_idr >= float(config["oscillation_min_curvature_interdecile_range_1_m"]))
    )
    transition_valid = bool(
        window_duration >= minimum_duration
        and selected.size >= 2
        and np.isfinite(position[selected]).all()
        and np.isfinite(actual_aux[selected]).all()
    )
    return {
        "window_start_time_s": float(time[selected[0]]),
        "window_end_time_s": float(time[selected[-1]]),
        "window_duration_s": window_duration,
        "window_sample_count": int(selected.size),
        "path_length_m": path_length,
        "target_reached": bool(target_reached),
        "state_stationary": state_stationary,
        "geometry_identifiable": geometry_identifiable,
        "oscillatory": oscillatory,
        "transition_valid": transition_valid,
        "median_auxiliary_target_error_rad": target_error,
        "auxiliary_p95_abs_velocity_rad_s": aux_p95,
        "median_planar_speed_m_s": median_speed,
        "speed_coefficient_of_variation": speed_cv,
        "world_z_rate_trend_rad_s2": world_z_trend,
        "course_rate_peak_to_peak_rad_s": course_peak_to_peak,
        "curvature_interdecile_range_1_m": curvature_idr,
        "median_three_point_curvature_1_m": (
            float(np.median(finite_geometry)) if finite_geometry.size else None
        ),
    }


def _first_change(values: np.ndarray) -> int | None:
    changed = np.flatnonzero(np.abs(values - values[0]) > 1e-9)
    return int(changed[0]) if changed.size else None


def _first_fraction_time(
    time: np.ndarray,
    value: np.ndarray,
    *,
    start: int,
    initial: float,
    target: float,
    fraction: float,
) -> float | None:
    change = target - initial
    if abs(change) <= 1e-12:
        return 0.0
    progress = (value[start:] - initial) / change
    indices = np.flatnonzero(progress >= fraction)
    return None if not indices.size else float(time[start + int(indices[0])] - time[start])


def measure_t90(
    *,
    time_s: object,
    requested_target: object,
    final_command: object,
    measured: object,
    fraction: float = 0.9,
) -> dict[str, object]:
    """Keep request-target and observed-final t90 definitions explicit."""

    time = _finite_array(time_s, name="time_s")
    requested = _finite_array(requested_target, name="requested_target")
    final = _finite_array(final_command, name="final_command")
    actual = _finite_array(measured, name="measured")
    if len({len(time), len(requested), len(final), len(actual)}) != 1:
        raise ValueError("t90 signals must align")
    request_start = _first_change(requested)
    if request_start is None:
        request_start = 0
    final_start = _first_change(final)
    if final_start is None:
        final_start = request_start
    requested_value = float(requested[-1])
    observed_final = float(np.median(actual[max(request_start, len(actual) - max(3, len(actual) // 10)) :]))
    initial_actual = float(actual[max(0, request_start - 1)])
    initial_final = float(final[max(0, request_start - 1)])
    return {
        "fraction": float(fraction),
        "request_change_time_s": float(time[request_start]),
        "final_command_change_time_s": float(time[final_start]),
        "requested_target": requested_value,
        "observed_final": observed_final,
        "measured_t90_requested_target_from_request_s": _first_fraction_time(
            time, actual, start=request_start, initial=initial_actual, target=requested_value, fraction=fraction
        ),
        "measured_t90_observed_final_from_request_s": _first_fraction_time(
            time, actual, start=request_start, initial=initial_actual, target=observed_final, fraction=fraction
        ),
        "measured_t90_requested_target_from_final_command_change_s": _first_fraction_time(
            time, actual, start=final_start, initial=float(actual[max(0, final_start - 1)]), target=requested_value, fraction=fraction
        ),
        "final_command_t90_requested_target_from_request_s": _first_fraction_time(
            time, final, start=request_start, initial=initial_final, target=requested_value, fraction=fraction
        ),
    }


def phase_step_samples(
    samples: Sequence[Mapping[str, object]], phase: str
) -> list[Mapping[str, object]]:
    """Return one pre-step sample plus one contiguous requested phase."""

    indices = [index for index, sample in enumerate(samples) if sample.get("phase") == phase]
    if not indices:
        raise ValueError(f"trace has no {phase!r} phase")
    if indices != list(range(indices[0], indices[-1] + 1)):
        raise ValueError(f"trace phase {phase!r} is not contiguous")
    first = max(0, indices[0] - 1)
    return list(samples[first : indices[-1] + 1])


def _trace_samples(trace: Mapping[str, object]) -> list[Mapping[str, object]]:
    samples = trace.get("samples")
    if not isinstance(samples, list) or not samples or not all(isinstance(item, Mapping) for item in samples):
        raise ValueError("trace requires a non-empty sample list")
    normalized: list[Mapping[str, object]] = []
    for raw_sample in samples:
        sample = dict(raw_sample)
        if "requested_main_rad_s" not in sample and "main_request_rad_s" in sample:
            sample["requested_main_rad_s"] = sample["main_request_rad_s"]
        if "requested_auxiliary_rad" not in sample and "auxiliary_request_rad" in sample:
            sample["requested_auxiliary_rad"] = sample["auxiliary_request_rad"]
        normalized.append(sample)
    return normalized


def _transition_boundary(samples: Sequence[Mapping[str, object]]) -> tuple[int, int]:
    transition = [index for index, sample in enumerate(samples) if sample.get("phase") == "transition"]
    if not transition or transition[0] == 0:
        raise ValueError("trace needs precondition samples before transition")
    return transition[0] - 1, transition[0]


def _rotate_world_to_local(vector: np.ndarray, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([[c, s], [-s, c]], dtype=np.float64) @ vector


def extract_transition_windows(
    trace: Mapping[str, object], protocol: Mapping[str, object]
) -> list[dict[str, object]]:
    """Extract fixed-start-frame measured transitions from one complete sequence."""

    samples = _trace_samples(trace)
    start_index, first_transition = _transition_boundary(samples)
    start = samples[start_index]
    start_time = float(start["time_s"])
    start_position = np.asarray(start["position_xy_m"], dtype=np.float64)
    start_yaw = float(start["navigation_heading_rad"])
    sequence = trace.get("sequence")
    sequence_id = str(sequence.get("sequence_id")) if isinstance(sequence, Mapping) else "unknown"
    role = str(sequence.get("dataset_role")) if isinstance(sequence, Mapping) else "unknown"
    rows: list[dict[str, object]] = []
    time = np.asarray([sample["time_s"] for sample in samples], dtype=np.float64)
    for horizon in protocol["finite_horizons_s"]:  # type: ignore[index]
        horizon = float(horizon)
        target_time = start_time + horizon
        candidates = np.arange(first_transition, len(samples))
        end_index = int(candidates[np.argmin(np.abs(time[candidates] - target_time))])
        if time[end_index] + 0.5 * np.median(np.diff(time)) < target_time:
            continue
        end = samples[end_index]
        world_delta = np.asarray(end["position_xy_m"], dtype=np.float64) - start_position
        local_delta = _rotate_world_to_local(world_delta, start_yaw)
        start_course = math.atan2(
            float(start["linear_velocity_world_xy_m_s"][1]),
            float(start["linear_velocity_world_xy_m_s"][0]),
        )
        end_velocity = np.asarray(end["linear_velocity_world_xy_m_s"], dtype=np.float64)
        start_speed = float(np.linalg.norm(start["linear_velocity_world_xy_m_s"]))
        end_speed = float(np.linalg.norm(end_velocity))
        minimum_speed = float(protocol["analysis_window"]["minimum_planar_speed_m_s"])  # type: ignore[index]
        subset = samples[first_transition : end_index + 1]
        window_speeds = np.asarray(
            [
                start_speed,
                *(
                    float(np.linalg.norm(item["linear_velocity_world_xy_m_s"]))
                    for item in subset
                ),
            ],
            dtype=float,
        )
        course_valid = bool(np.all(window_speeds >= minimum_speed))
        end_course = math.atan2(float(end_velocity[1]), float(end_velocity[0])) if course_valid else None
        rows.append(
            {
                "sequence_id": sequence_id,
                "dataset_role": role,
                "horizon_s": horizon,
                "start_sample_index": start_index,
                "first_transition_sample_index": first_transition,
                "end_sample_index": end_index,
                "start_time_s": start_time,
                "end_time_s": float(end["time_s"]),
                "start_navigation_heading_rad": start_yaw,
                "start_main_joint_velocity_rad_s": float(start["main_joint_velocity_rad_s"]),
                "start_auxiliary_joint_rad": float(start["auxiliary_joint_rad"]),
                "start_auxiliary_joint_velocity_rad_s": float(start["auxiliary_joint_velocity_rad_s"]),
                "start_linear_velocity_world_xy_m_s": list(map(float, start["linear_velocity_world_xy_m_s"])),
                "start_world_z_angular_velocity_rad_s": float(start["angular_velocity_world_rad_s"][2]),
                "start_final_main_rad_s": float(start["final_main_rad_s"]),
                "start_final_auxiliary_rad": float(start["final_auxiliary_rad"]),
                "future_request_main_rad_s": [float(item["requested_main_rad_s"]) for item in subset],
                "future_request_auxiliary_rad": [float(item["requested_auxiliary_rad"]) for item in subset],
                "world_delta_xy_m": world_delta.tolist(),
                "local_delta_xy_m": local_delta.tolist(),
                "root_heading_change_rad": float(_angle_wrap(float(end["navigation_heading_rad"]) - start_yaw)),
                "course_change_rad": None if not course_valid else float(_angle_wrap(float(end_course) - start_course)),
                "course_change_valid": bool(course_valid),
                "minimum_planar_speed_in_window_m_s": float(np.min(window_speeds)),
                "end_main_joint_velocity_rad_s": float(end["main_joint_velocity_rad_s"]),
                "end_auxiliary_joint_rad": float(end["auxiliary_joint_rad"]),
                "end_auxiliary_joint_velocity_rad_s": float(end["auxiliary_joint_velocity_rad_s"]),
                "end_linear_velocity_world_xy_m_s": end_velocity.tolist(),
                "supervisor_active_fraction": float(np.mean([bool(item["auxiliary_velocity_supervisor_active"]) for item in subset])),
                "auxiliary_tracking_error_abs_max_rad": float(max(abs(float(item.get("auxiliary_tracking_error_rad", 0.0))) for item in subset)),
                "transition_valid": True,
            }
        )
    return rows


def validate_predictor_payload(payload: Mapping[str, object]) -> None:
    if not isinstance(payload.get("current_state"), Mapping):
        raise ValueError("predictor requires current_state")
    if not isinstance(payload.get("future_requests"), Sequence):
        raise ValueError("predictor requires future_requests")
    forbidden = [
        key
        for key in payload
        if key.startswith("future_measured") or key.startswith("future_final_supervisor")
    ]
    if forbidden:
        raise ValueError("future measured state or supervisor output is forbidden")


def _state_from_sample(sample: Mapping[str, object]) -> np.ndarray:
    yaw = float(sample["navigation_heading_rad"])
    body_velocity = _rotate_world_to_local(
        np.asarray(sample["linear_velocity_world_xy_m_s"], dtype=np.float64), yaw
    )
    return np.asarray(
        [
            sample["main_joint_velocity_rad_s"],
            sample["auxiliary_joint_rad"],
            sample["auxiliary_joint_velocity_rad_s"],
            body_velocity[0],
            body_velocity[1],
            sample["angular_velocity_world_rad_s"][2],
            sample["final_main_rad_s"],
            sample["final_auxiliary_rad"],
        ],
        dtype=np.float64,
    )


def _features(state: np.ndarray, request: np.ndarray) -> np.ndarray:
    return np.concatenate(
        (
            [1.0],
            state,
            request,
            [
                state[0] * state[1],
                state[3] * state[1],
                request[0] - state[6],
                request[1] - state[7],
            ],
        )
    )


def fit_m1_model(
    traces: Sequence[tuple[Path, Mapping[str, object]]],
    protocol: Mapping[str, object],
) -> dict[str, object]:
    """Fit the sole preregistered M1 using only complete fit-core sequences."""

    design: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    identities: list[dict[str, object]] = []
    for path, trace in traces:
        sequence = trace.get("sequence")
        if not isinstance(sequence, Mapping) or sequence.get("dataset_role") != "fit_core":
            raise ValueError("M1 fit accepts fit_core traces only")
        samples = _trace_samples(trace)
        for current, future in zip(samples, samples[1:]):
            state = _state_from_sample(current)
            future_state = _state_from_sample(future)
            request = np.asarray(
                [future["requested_main_rad_s"], future["requested_auxiliary_rad"]],
                dtype=np.float64,
            )
            design.append(_features(state, request))
            targets.append(future_state - state)
        identities.append(
            {
                "sequence_id": sequence["sequence_id"],
                "trace_path": str(path.resolve()),
                "trace_sha256": _sha256(path),
            }
        )
    x = np.vstack(design)
    y = np.vstack(targets)
    ridge = float(protocol["candidate"]["ridge_lambda"])  # type: ignore[index]
    penalty = np.eye(x.shape[1]) * ridge
    penalty[0, 0] = 0.0
    coefficients = np.linalg.solve(x.T @ x + penalty, x.T @ y)
    residual = y - x @ coefficients
    return {
        "schema_version": M1_SCHEMA,
        "candidate_name": protocol["candidate"]["name"],  # type: ignore[index]
        "state_names": list(STATE_NAMES),
        "request_names": list(REQUEST_NAMES),
        "feature_names": [
            "bias",
            *STATE_NAMES,
            *REQUEST_NAMES,
            "main_times_aux",
            "body_forward_times_aux",
            "main_request_minus_final_main",
            "aux_request_minus_final_aux",
        ],
        "dt_s": float(protocol["time_alignment"]["dt_s"]),  # type: ignore[index]
        "ridge_lambda": ridge,
        "coefficient_feature_by_state": coefficients.tolist(),
        "fit_step_count": int(x.shape[0]),
        "fit_sequence_count": len(identities),
        "fit_residual_state_rmse": np.sqrt(np.mean(np.square(residual), axis=0)).tolist(),
        "fit_trace_identities": identities,
        "causal_contract": {
            "initialized_from_current_measured_state_only": True,
            "future_requested_commands_only": True,
            "future_measured_state_used": False,
            "future_supervisor_output_used": False,
            "truth_reset_inside_window": False,
        },
        "heading_update": {
            "method": "integrate recursively predicted world-z angular velocity",
            "label": "root navigation yaw change",
            "world_z_is_not_assumed_identical_to_euler_yaw_derivative": True,
            "proxy_error_must_be_reported_against_root_yaw": True,
        },
    }


def _rollout_m1(
    model: Mapping[str, object],
    start: Mapping[str, object],
    future_samples: Sequence[Mapping[str, object]],
) -> dict[str, np.ndarray | float]:
    coefficients = np.asarray(model["coefficient_feature_by_state"], dtype=np.float64)
    state = _state_from_sample(start)
    yaw = float(start["navigation_heading_rad"])
    position = np.zeros(2, dtype=np.float64)
    dt = float(model["dt_s"])
    trajectory = [position.copy()]
    headings = [yaw]
    state_trace = [state.copy()]
    for sample in future_samples:
        request = np.asarray(
            [sample["requested_main_rad_s"], sample["requested_auxiliary_rad"]],
            dtype=np.float64,
        )
        state = state + _features(state, request) @ coefficients
        if not np.isfinite(state).all():
            raise ValueError("M1 recursive rollout became non-finite")
        state[6] = float(np.clip(state[6], -2.0, 2.0))
        state[7] = float(np.clip(state[7], -math.radians(30.0), math.radians(30.0)))
        yaw += float(state[5]) * dt
        c, s = math.cos(yaw), math.sin(yaw)
        world_velocity = np.asarray(
            [c * state[3] - s * state[4], s * state[3] + c * state[4]]
        )
        position += world_velocity * dt
        trajectory.append(position.copy())
        headings.append(yaw)
        state_trace.append(state.copy())
    start_yaw = float(start["navigation_heading_rad"])
    local_position = _rotate_world_to_local(position, start_yaw)
    return {
        "local_delta_xy_m": local_position,
        "root_heading_change_rad": float(_angle_wrap(yaw - start_yaw)),
        "terminal_state": state,
        "state_trace": np.asarray(state_trace),
        "world_trajectory_delta_xy_m": np.asarray(trajectory),
        "heading_trace_rad": np.asarray(headings),
        "heading_estimator": "integral_of_recursively_predicted_world_z_angular_velocity",
    }


def _rollout_m0(
    model: DriveModel,
    start: Mapping[str, object],
    future_samples: Sequence[Mapping[str, object]],
    *,
    dt_s: float,
) -> dict[str, object]:
    yaw = float(start["navigation_heading_rad"])
    start_yaw = yaw
    position = np.zeros(2, dtype=np.float64)
    trajectory = [position.copy()]
    out_of_support = False
    for sample in future_samples:
        main = float(sample["requested_main_rad_s"])
        auxiliary = float(sample["requested_auxiliary_rad"])
        if main == 0.0:
            signed_speed = 0.0
            curvature = 0.0
        else:
            out_of_support = out_of_support or abs(main) < 0.5 or abs(main) > 1.5
            speed = model.speed(main, auxiliary)
            signed_speed = math.copysign(speed, main)
            curvature = model.curvature(main, auxiliary)
        yaw += signed_speed * curvature * dt_s
        position += signed_speed * np.asarray([math.cos(yaw), math.sin(yaw)]) * dt_s
        trajectory.append(position.copy())
    return {
        "local_delta_xy_m": _rotate_world_to_local(position, start_yaw),
        "root_heading_change_rad": float(_angle_wrap(yaw - start_yaw)),
        "world_trajectory_delta_xy_m": np.asarray(trajectory),
        "out_of_support": out_of_support,
        "supported_outputs": ["position_xy", "kinematic_heading"],
        "unsupported_outputs": ["joint_terminal_state", "smoother_state", "supervisor_state"],
    }


def _prediction_rows(
    trace_path: Path,
    trace: Mapping[str, object],
    protocol: Mapping[str, object],
    drive_model: DriveModel,
    m1: Mapping[str, object],
) -> list[dict[str, object]]:
    samples = _trace_samples(trace)
    start_index, first_transition = _transition_boundary(samples)
    measured = extract_transition_windows(trace, protocol)
    dt = float(protocol["time_alignment"]["dt_s"])  # type: ignore[index]
    rows: list[dict[str, object]] = []
    for measured_row in measured:
        end_index = int(measured_row["end_sample_index"])
        future = samples[first_transition : end_index + 1]
        payload = {
            "current_state": {name: value for name, value in zip(STATE_NAMES, _state_from_sample(samples[start_index]))},
            "future_requests": [
                {
                    "main_request_rad_s": item["requested_main_rad_s"],
                    "auxiliary_request_rad": item["requested_auxiliary_rad"],
                }
                for item in future
            ],
        }
        validate_predictor_payload(payload)
        m0 = _rollout_m0(drive_model, samples[start_index], future, dt_s=dt)
        m1_result = _rollout_m1(m1, samples[start_index], future)
        actual = np.asarray(measured_row["local_delta_xy_m"], dtype=np.float64)
        actual_heading = float(measured_row["root_heading_change_rad"])
        actual_world_trajectory = np.asarray(
            [sample["position_xy_m"] for sample in [samples[start_index], *future]],
            dtype=np.float64,
        )
        actual_world_trajectory -= actual_world_trajectory[0]
        actual_local_trajectory = np.vstack(
            [_rotate_world_to_local(point, float(samples[start_index]["navigation_heading_rad"])) for point in actual_world_trajectory]
        )
        row = dict(measured_row)
        for name, result in (("m0", m0), ("m1", m1_result)):
            predicted = np.asarray(result["local_delta_xy_m"], dtype=np.float64)
            error = predicted - actual
            predicted_world_trajectory = np.asarray(
                result["world_trajectory_delta_xy_m"], dtype=np.float64
            )
            predicted_local_trajectory = np.vstack(
                [
                    _rotate_world_to_local(
                        point, float(samples[start_index]["navigation_heading_rad"])
                    )
                    for point in predicted_world_trajectory
                ]
            )
            trajectory_error = np.linalg.norm(
                predicted_local_trajectory - actual_local_trajectory, axis=1
            )
            row.update(
                {
                    f"{name}_predicted_local_delta_xy_m": predicted.tolist(),
                    f"{name}_endpoint_error_xy_m": error.tolist(),
                    f"{name}_endpoint_position_error_m": float(np.linalg.norm(error)),
                    f"{name}_root_heading_change_rad": float(result["root_heading_change_rad"]),
                    f"{name}_root_heading_change_error_rad": float(
                        _angle_wrap(float(result["root_heading_change_rad"]) - actual_heading)
                    ),
                    f"{name}_trajectory_rms_error_m": float(
                        np.sqrt(np.mean(np.square(trajectory_error)))
                    ),
                    f"{name}_trajectory_peak_error_m": float(np.max(trajectory_error)),
                }
            )
        actual_terminal_state = _state_from_sample(samples[end_index])
        predicted_terminal_state = np.asarray(m1_result["terminal_state"], dtype=float)
        terminal_error = predicted_terminal_state - actual_terminal_state
        row["actual_terminal_state"] = dict(zip(STATE_NAMES, actual_terminal_state.tolist()))
        row["m1_predicted_terminal_state"] = dict(
            zip(STATE_NAMES, predicted_terminal_state.tolist())
        )
        row["m1_terminal_state_error"] = dict(zip(STATE_NAMES, terminal_error.tolist()))
        actual_state_trace = np.vstack(
            [_state_from_sample(sample) for sample in [samples[start_index], *future]]
        )
        predicted_state_trace = np.asarray(m1_result["state_trace"], dtype=float)
        elapsed = np.arange(len(actual_state_trace), dtype=float) * dt
        main_target = float(future[-1]["requested_main_rad_s"])
        auxiliary_target = float(future[-1]["requested_auxiliary_rad"])
        for label, state_index, target in (
            ("main", 0, main_target),
            ("auxiliary", 1, auxiliary_target),
        ):
            required = bool(
                abs(target) > 1e-9
                and actual_state_trace[0, state_index] * target < 0.0
            )
            actual_crossing = _target_sign_zero_crossing_time(
                actual_state_trace[:, state_index], elapsed, target
            )
            predicted_crossing = _target_sign_zero_crossing_time(
                predicted_state_trace[:, state_index], elapsed, target
            )
            row[f"{label}_reversal_required"] = required
            row[f"actual_{label}_zero_crossing_s"] = actual_crossing
            row[f"m1_{label}_zero_crossing_s"] = predicted_crossing
            row[f"m1_{label}_zero_crossing_error_s"] = (
                None
                if actual_crossing is None or predicted_crossing is None
                else float(predicted_crossing - actual_crossing)
            )
        measured_world_z_integral = float(
            np.sum([sample["angular_velocity_world_rad_s"][2] for sample in future]) * dt
        )
        row["measured_world_z_integral_rad"] = measured_world_z_integral
        row["measured_world_z_integral_minus_root_heading_change_rad"] = float(
            _angle_wrap(measured_world_z_integral - actual_heading)
        )
        row["m1_heading_estimator"] = m1_result["heading_estimator"]
        row["m0_out_of_support"] = bool(m0["out_of_support"])
        row["trace_path"] = str(trace_path.resolve())
        row["trace_sha256"] = _sha256(trace_path)
        rows.append(row)
    return rows


def _target_sign_zero_crossing_time(
    values: object, elapsed_time_s: object, target: float
) -> float | None:
    values_array = np.asarray(values, dtype=float)
    time_array = np.asarray(elapsed_time_s, dtype=float)
    if values_array.shape != time_array.shape or values_array.ndim != 1:
        raise ValueError("zero-crossing values and times must be aligned vectors")
    if abs(target) <= 1e-9 or values_array[0] * target >= 0.0:
        return None
    signed = values_array * math.copysign(1.0, target)
    candidates = np.flatnonzero(signed >= 0.0)
    candidates = candidates[candidates > 0]
    if not candidates.size:
        return None
    index = int(candidates[0])
    before, after = float(signed[index - 1]), float(signed[index])
    fraction = 1.0 if math.isclose(after, before) else float(
        np.clip(-before / (after - before), 0.0, 1.0)
    )
    return float(time_array[index - 1] + fraction * (time_array[index] - time_array[index - 1]))


def _aggregate_prediction(rows: Sequence[Mapping[str, object]], prefix: str) -> dict[str, object]:
    endpoint = np.asarray([row[f"{prefix}_endpoint_position_error_m"] for row in rows], dtype=float)
    heading = np.asarray([abs(float(row[f"{prefix}_root_heading_change_error_rad"])) for row in rows], dtype=float)
    six = np.asarray(
        [row[f"{prefix}_endpoint_position_error_m"] for row in rows if math.isclose(float(row["horizon_s"]), 6.0)],
        dtype=float,
    )
    return {
        "sample_count": len(rows),
        "endpoint_position_mae_m": float(np.mean(endpoint)),
        "endpoint_position_rmse_m": float(np.sqrt(np.mean(np.square(endpoint)))),
        "endpoint_position_p90_m": float(np.percentile(endpoint, 90)),
        "endpoint_position_p90_m_at_6s": float(np.percentile(six, 90)) if six.size else None,
        "root_heading_change_mae_rad": float(np.mean(heading)),
    }


def _repeatability(
    all_rows: Sequence[Mapping[str, object]], manifest: Mapping[str, object]
) -> dict[str, object]:
    sequences = manifest["sequences"]
    pairs = [item for item in sequences if item.get("dataset_role") == "repeat_validation"]
    results = []
    for pair in pairs:
        repeat_id = pair["sequence_id"]
        core_id = pair["repeat_of"]
        repeat = next(row for row in all_rows if row["sequence_id"] == repeat_id and math.isclose(float(row["horizon_s"]), 6.0))
        core = next(row for row in all_rows if row["sequence_id"] == core_id and math.isclose(float(row["horizon_s"]), 6.0))
        position_difference = float(
            np.linalg.norm(
                np.asarray(repeat["local_delta_xy_m"], dtype=float)
                - np.asarray(core["local_delta_xy_m"], dtype=float)
            )
        )
        heading_difference = abs(
            float(_angle_wrap(float(repeat["root_heading_change_rad"]) - float(core["root_heading_change_rad"])))
        )
        results.append(
            {
                "repeat_sequence_id": repeat_id,
                "fit_core_sequence_id": core_id,
                "endpoint_difference_m_at_6s": position_difference,
                "heading_difference_rad_at_6s": heading_difference,
            }
        )
    return {
        "pairs": results,
        "maximum_endpoint_difference_m_at_6s": max(item["endpoint_difference_m_at_6s"] for item in results),
        "maximum_heading_difference_rad_at_6s": max(item["heading_difference_rad_at_6s"] for item in results),
    }


def evaluate_candidate(
    rows: Sequence[Mapping[str, object]],
    protocol: Mapping[str, object],
    repeatability: Mapping[str, object],
) -> dict[str, object]:
    heldout = [row for row in rows if row["dataset_role"] == "heldout_validation" and row["transition_valid"]]
    valid_sequences = len({str(row["sequence_id"]) for row in heldout})
    m0 = _aggregate_prediction(heldout, "m0")
    m1 = _aggregate_prediction(heldout, "m1")
    improvement = (
        (float(m0["endpoint_position_mae_m"]) - float(m1["endpoint_position_mae_m"]))
        / float(m0["endpoint_position_mae_m"])
        if float(m0["endpoint_position_mae_m"]) > 0.0
        else -math.inf
    )
    gates = protocol["candidate_gates"]
    checks = {
        "minimum_valid_heldout_sequences": valid_sequences >= int(gates["minimum_valid_heldout_sequences"]),
        "minimum_endpoint_mae_improvement": improvement >= float(gates["minimum_endpoint_mae_improvement_fraction_over_m0"]),
        "maximum_endpoint_position_mae": float(m1["endpoint_position_mae_m"]) <= float(gates["maximum_endpoint_position_mae_m_all_horizons"]),
        "maximum_endpoint_position_p90_at_6s": float(m1["endpoint_position_p90_m_at_6s"]) <= float(gates["maximum_endpoint_position_p90_m_at_6s"]),
        "maximum_root_heading_change_mae": float(m1["root_heading_change_mae_rad"]) <= float(gates["maximum_root_heading_change_mae_rad"]),
        "maximum_repeat_endpoint_difference": float(repeatability["maximum_endpoint_difference_m_at_6s"]) <= float(gates["maximum_repeat_endpoint_difference_m_at_6s"]),
        "maximum_repeat_heading_difference": float(repeatability["maximum_heading_difference_rad_at_6s"]) <= float(gates["maximum_repeat_heading_difference_rad_at_6s"]),
    }
    return {
        "valid_heldout_sequence_count": valid_sequences,
        "m0": m0,
        "m1": m1,
        "endpoint_mae_improvement_fraction_over_m0": improvement,
        "gate_checks": checks,
        "accepted": all(checks.values()),
        "decision": "accepted_under_frozen_endpoint_and_heading_gates" if all(checks.values()) else "rejected",
        "acceptance_scope": (
            "finite-horizon endpoint position and empirically evaluated root-heading estimate; "
            "terminal-state, trajectory-shape, and reversal diagnostics are reported separately"
        ),
    }


def _supplementary_prediction_diagnostics(
    rows: Sequence[Mapping[str, object]],
) -> dict[str, object]:
    heldout = [row for row in rows if row["dataset_role"] == "heldout_validation"]
    terminal_mae = {
        name: float(
            np.mean([abs(float(row["m1_terminal_state_error"][name])) for row in heldout])
        )
        for name in STATE_NAMES
    }
    six_second = [
        row for row in heldout if math.isclose(float(row["horizon_s"]), 6.0)
    ]
    reversals: dict[str, object] = {}
    for label in ("main", "auxiliary"):
        required = [row for row in six_second if row[f"{label}_reversal_required"]]
        measurable = [
            row
            for row in required
            if row[f"m1_{label}_zero_crossing_error_s"] is not None
        ]
        errors = np.asarray(
            [abs(float(row[f"m1_{label}_zero_crossing_error_s"])) for row in measurable],
            dtype=float,
        )
        reversals[label] = {
            "required_sequence_count": len(required),
            "both_crossings_measurable_count": len(measurable),
            "zero_crossing_mae_s": float(np.mean(errors)) if errors.size else None,
            "zero_crossing_max_abs_error_s": float(np.max(errors)) if errors.size else None,
        }
    proxy_error = np.asarray(
        [
            abs(float(row["measured_world_z_integral_minus_root_heading_change_rad"]))
            for row in heldout
        ],
        dtype=float,
    )
    return {
        "heldout_window_count": len(heldout),
        "m0_trajectory_rms_error_mean_m": float(
            np.mean([row["m0_trajectory_rms_error_m"] for row in heldout])
        ),
        "m1_trajectory_rms_error_mean_m": float(
            np.mean([row["m1_trajectory_rms_error_m"] for row in heldout])
        ),
        "m1_terminal_state_mae": terminal_mae,
        "six_second_reversal_zero_crossing": reversals,
        "measured_world_z_integral_vs_root_heading": {
            "mae_rad": float(np.mean(proxy_error)),
            "max_abs_error_rad": float(np.max(proxy_error)),
            "note": "world-z integral is a tested proxy input to the estimator, not an identity with Euler-yaw derivative",
        },
    }


def _project_point_to_polyline(point: np.ndarray, line: np.ndarray, cumulative: np.ndarray) -> tuple[float, float]:
    best_distance = math.inf
    best_progress = float(cumulative[0])
    if len(line) == 1:
        return best_progress, float(np.linalg.norm(point - line[0]))
    for index, (left, right) in enumerate(zip(line, line[1:])):
        delta = right - left
        length2 = float(delta @ delta)
        fraction = 0.0 if length2 <= 1e-12 else float(np.clip((point - left) @ delta / length2, 0.0, 1.0))
        projected = left + fraction * delta
        distance = float(np.linalg.norm(point - projected))
        if distance < best_distance:
            best_distance = distance
            best_progress = float(cumulative[index] + fraction * math.sqrt(length2))
    return best_progress, best_distance


def audit_reference_progress(states_path: Path, protocol: Mapping[str, object]) -> dict[str, object]:
    with np.load(states_path, allow_pickle=False) as data:
        arrays = {name: np.asarray(data[name]) for name in data.files}
    position = arrays["position_world_xy"]
    reference = arrays["kinodynamic_reference_pose"][:, :2]
    direction = arrays["kinodynamic_reference_drive_direction"].astype(int)
    reference_index = arrays["policy_reference_index"].astype(int)
    recorded = arrays["policy_reference_progress_s_m"].astype(float)
    action_count = min(len(reference_index), len(recorded), len(position) - 1)
    reference_step = np.linalg.norm(np.diff(reference, axis=0), axis=1)
    cumulative = np.r_[0.0, np.cumsum(reference_step)]
    boundaries = [0, *(np.flatnonzero(np.diff(direction) != 0) + 1).tolist(), len(reference)]
    independent: list[float] = []
    projection_error: list[float] = []
    for action in range(action_count):
        index = int(np.clip(reference_index[action], 0, len(reference) - 1))
        segment_number = max(i for i in range(len(boundaries) - 1) if boundaries[i] <= index)
        left, right = boundaries[segment_number], boundaries[segment_number + 1]
        line = reference[left:right]
        progress, error = _project_point_to_polyline(position[action + 1], line, cumulative[left:right])
        independent.append(progress)
        projection_error.append(error)
    independent_array = np.asarray(independent)
    recorded = recorded[:action_count]
    threshold = float(protocol["reference_progress"]["abnormal_jump_threshold_m_per_control_tick"])  # type: ignore[index]
    return {
        "states_path": str(states_path.resolve()),
        "actual_cumulative_path_length_m": float(np.linalg.norm(np.diff(position, axis=0), axis=1).sum()),
        "recorded_progress_final_m": float(recorded[-1]),
        "recorded_progress_peak_m": float(np.max(recorded)),
        "recorded_backward_step_count": int(np.count_nonzero(np.diff(recorded) < -1e-9)),
        "recorded_abnormal_jump_count": int(np.count_nonzero(np.diff(recorded) > threshold)),
        "recorded_max_jump_m": float(np.max(np.diff(recorded))) if len(recorded) > 1 else 0.0,
        "independent_segment_projection_final_m": float(independent_array[-1]),
        "independent_segment_projection_backward_step_count": int(np.count_nonzero(np.diff(independent_array) < -1e-9)),
        "independent_segment_projection_abnormal_jump_count": int(np.count_nonzero(np.abs(np.diff(independent_array)) > threshold)),
        "projection_error_rms_m": float(np.sqrt(np.mean(np.square(projection_error)))),
        "projection_error_peak_m": float(np.max(projection_error)),
        "recorded_minus_independent_rms_m": float(np.sqrt(np.mean(np.square(recorded - independent_array)))),
        "note": "independent projection is restricted to the recorded active direction segment and is not monotonicized",
    }


def run_p0(
    *,
    protocol_path: Path,
    round4_root: Path,
    round4_analysis_path: Path,
    fixed_root: Path,
    output_dir: Path,
) -> dict[str, object]:
    protocol = load_measurement_protocol(protocol_path)
    old_analysis = _load_json(round4_analysis_path)
    old_by_id = {row["case_id"]: row for row in old_analysis["response_rows"]}
    rows: list[dict[str, object]] = []
    course_alignment_errors: list[float] = []
    curvature_pairs: list[tuple[float, float]] = []
    derived_dir = output_dir / "round4_reanalysis_derived_traces"
    for trace_path in sorted((round4_root / "traces").glob("*.json")):
        trace = _load_json(trace_path)
        samples = _trace_samples(trace)
        case = trace["case"]
        case_id = str(case["case_id"])
        columns = _samples_to_columns(samples)
        flags = classify_measurement_window(columns, protocol)
        motion = measure_motion_quantities(
            time_s=columns["time_s"],
            position_xy_m=columns["position_xy_m"],
            linear_velocity_world_xy_m_s=columns["linear_velocity_world_xy_m_s"],
            root_quaternion_world_wxyz=columns["root_quaternion_world_wxyz"],
            angular_velocity_world_rad_s=columns["angular_velocity_world_rad_s"],
            minimum_planar_speed_m_s=float(protocol["analysis_window"]["minimum_planar_speed_m_s"]),  # type: ignore[index]
        )
        time = np.asarray(columns["time_s"], dtype=float)
        position = np.asarray(columns["position_xy_m"], dtype=float)
        velocity = np.asarray(columns["linear_velocity_world_xy_m_s"], dtype=float)
        position_derivative = np.column_stack(
            [np.gradient(position[:, axis], time) for axis in range(2)]
        )
        velocity_alignment_rms = float(
            np.sqrt(np.mean(np.square(position_derivative - velocity)))
        )
        displacement = np.diff(position, axis=0)
        displacement_course = np.arctan2(displacement[:, 1], displacement[:, 0])
        course_valid = motion["course_valid"][1:] & (
            np.linalg.norm(displacement, axis=1) > 1e-3
        )
        local_course_errors = np.abs(
            _angle_wrap(motion["course_rad"][1:][course_valid] - displacement_course[course_valid])
        )
        course_alignment_errors.extend(map(float, local_course_errors))
        curvature_valid = np.isfinite(motion["kinematic_curvature_1_m"]) & np.isfinite(
            motion["three_point_curvature_1_m"]
        )
        curvature_pairs.extend(
            zip(
                map(float, motion["kinematic_curvature_1_m"][curvature_valid]),
                map(float, motion["three_point_curvature_1_m"][curvature_valid]),
            )
        )
        if any(sample.get("phase") == "steer" for sample in samples):
            t90_samples = phase_step_samples(samples, "steer")
            t90 = measure_t90(
                time_s=np.asarray([sample["time_s"] for sample in t90_samples]),
                requested_target=np.asarray(
                    [sample["requested_auxiliary_rad"] for sample in t90_samples]
                ),
                final_command=np.asarray(
                    [sample["final_auxiliary_rad"] for sample in t90_samples]
                ),
                measured=np.asarray([sample["auxiliary_joint_rad"] for sample in t90_samples]),
                fraction=float(protocol["t90"]["fraction"]),  # type: ignore[index]
            )
            t90["not_applicable_reason"] = None
        else:
            t90 = {
                "fraction": float(protocol["t90"]["fraction"]),  # type: ignore[index]
                "request_change_time_s": None,
                "final_command_change_time_s": None,
                "requested_target": 0.0,
                "observed_final": float(np.median(columns["auxiliary_joint_rad"])),
                "measured_t90_requested_target_from_request_s": None,
                "measured_t90_observed_final_from_request_s": None,
                "measured_t90_requested_target_from_final_command_change_s": None,
                "final_command_t90_requested_target_from_request_s": None,
                "not_applicable_reason": "zero-amplitude auxiliary drive trace has no steer phase",
            }
        old = old_by_id[case_id]
        rows.append(
            {
                "case_id": case_id,
                "dataset_role": case["dataset_role"],
                "requested_main_rad_s": case["main_command_rad_s"],
                "requested_auxiliary_rad": case["auxiliary_command_rad"],
                "round4_steady_state_valid": old["steady_state_valid"],
                "round4_invalid_reasons": old["invalid_reasons"],
                **flags,
                **t90,
                "median_yaw_derivative_rad_s": float(np.median(motion["yaw_derivative_rad_s"])),
                "median_world_z_angular_velocity_rad_s": float(np.median(motion["world_z_angular_velocity_rad_s"])),
                "yaw_derivative_minus_world_z_rms_rad_s": float(np.sqrt(np.mean(np.square(motion["yaw_derivative_rad_s"] - motion["world_z_angular_velocity_rad_s"])))),
                "position_derivative_minus_recorded_velocity_rms_m_s": velocity_alignment_rms,
                "velocity_course_minus_displacement_course_median_abs_rad": (
                    float(np.median(local_course_errors)) if local_course_errors.size else None
                ),
            }
        )
        derived_samples = []
        for index, sample in enumerate(samples):
            derived_samples.append(
                {
                    "time_s": sample["time_s"],
                    "phase": sample["phase"],
                    **{key: value[index] for key, value in motion.items()},
                }
            )
        _atomic_json(
            derived_dir / trace_path.name,
            {
                "schema_version": "round5_round4_reanalysis_trace_v1",
                "source_path": str(trace_path.resolve()),
                "source_sha256": _sha256(trace_path),
                "samples": derived_samples,
            },
        )
    reference = {
        episode: audit_reference_progress(
            fixed_root / f"episode_{episode}" / "states.npz", protocol
        )
        for episode in ("hard_0_e1_0002", "hard_0_e1_0004")
    }
    count = lambda key: sum(bool(row[key]) for row in rows)
    curvature_array = np.asarray(curvature_pairs, dtype=float)
    result = {
        "schema_version": "round5_measurement_comparison_v1",
        "protocol_path": str(protocol_path.resolve()),
        "protocol_sha256": _sha256(protocol_path),
        "round4_source_count": len(rows),
        "round4_decision_preserved": old_analysis["candidate_decision"],
        "old_data_role": protocol["round4_reanalysis_role"],
        "new_flag_counts": {
            key: count(key)
            for key in (
                "target_reached",
                "state_stationary",
                "geometry_identifiable",
                "oscillatory",
                "transition_valid",
            )
        },
        "coordinate_and_time_alignment": {
            "position_derivative_minus_recorded_velocity_rms_mean_m_s": float(
                np.mean([row["position_derivative_minus_recorded_velocity_rms_m_s"] for row in rows])
            ),
            "position_derivative_minus_recorded_velocity_rms_max_m_s": float(
                np.max([row["position_derivative_minus_recorded_velocity_rms_m_s"] for row in rows])
            ),
            "velocity_course_minus_displacement_course_median_abs_rad": float(
                np.median(course_alignment_errors)
            ),
            "velocity_course_minus_displacement_course_p90_abs_rad": float(
                np.percentile(course_alignment_errors, 90.0)
            ),
            "kinematic_vs_three_point_curvature_correlation": float(
                np.corrcoef(curvature_array[:, 0], curvature_array[:, 1])[0, 1]
            ),
            "kinematic_vs_three_point_curvature_sign_agreement_fraction": float(
                np.mean(np.sign(curvature_array[:, 0]) == np.sign(curvature_array[:, 1]))
            ),
            "yaw_derivative_minus_world_z_rms_mean_rad_s": float(
                np.mean([row["yaw_derivative_minus_world_z_rms_rad_s"] for row in rows])
            ),
            "yaw_derivative_minus_world_z_rms_max_rad_s": float(
                np.max([row["yaw_derivative_minus_world_z_rms_rad_s"] for row in rows])
            ),
        },
        "rows": rows,
        "fixed_reference_progress": reference,
    }
    _atomic_json(output_dir / "round5_measurement_comparison.json", result)
    _write_csv(output_dir / "round5_round4_measurement_comparison.csv", rows)
    return result


def _load_manifest_payload(path: Path) -> dict[str, object]:
    payload = _load_json(path)
    if payload.get("schema_version") != "round5_transition_manifest_v1":
        raise ValueError("Round 5 transition manifest is required")
    return payload


def _trace_for_sequence(root: Path, sequence: Mapping[str, object]) -> tuple[Path, dict[str, object]]:
    path = root / "traces" / f"{sequence['sequence_id']}.json"
    trace = _load_json(path)
    identity = trace.get("sequence")
    if not isinstance(identity, Mapping) or identity.get("sequence_id") != sequence["sequence_id"]:
        raise ValueError(f"trace identity mismatch: {path}")
    return path, trace


def run_fit(
    *, protocol_path: Path, manifest_path: Path, transition_root: Path, output_path: Path
) -> dict[str, object]:
    protocol = load_measurement_protocol(protocol_path)
    manifest = _load_manifest_payload(manifest_path)
    # Deliberately construct paths for fit_core only. Held-out files are not opened here.
    core = [item for item in manifest["sequences"] if item["dataset_role"] == "fit_core"]
    traces = [_trace_for_sequence(transition_root, item) for item in core]
    model = fit_m1_model(traces, protocol)
    model["protocol_path"] = str(protocol_path.resolve())
    model["protocol_sha256"] = _sha256(protocol_path)
    model["manifest_path"] = str(manifest_path.resolve())
    model["manifest_sha256"] = _sha256(manifest_path)
    model["heldout_opened_during_fit"] = False
    _atomic_json(output_path, model)
    return model


def run_evaluate(
    *,
    protocol_path: Path,
    manifest_path: Path,
    transition_root: Path,
    drive_profile_path: Path,
    model_path: Path,
    output_dir: Path,
) -> dict[str, object]:
    protocol = load_measurement_protocol(protocol_path)
    manifest = _load_manifest_payload(manifest_path)
    if _sha256(drive_profile_path) != "2de28cdfe98ecd6d45d5158261efb436acabaf31c1df4774f75b708a9086ddfd":
        raise RuntimeError("frozen V1 hash mismatch")
    drive_model = DriveModel.from_json(drive_profile_path)
    m1 = _load_json(model_path)
    if m1.get("schema_version") != M1_SCHEMA or m1.get("heldout_opened_during_fit") is not False:
        raise ValueError("invalid or contaminated M1 model")
    rows: list[dict[str, object]] = []
    for sequence in manifest["sequences"]:
        path, trace = _trace_for_sequence(transition_root, sequence)
        rows.extend(_prediction_rows(path, trace, protocol, drive_model, m1))
    repeatability = _repeatability(rows, manifest)
    decision = evaluate_candidate(rows, protocol, repeatability)
    supplementary = _supplementary_prediction_diagnostics(rows)
    result = {
        "schema_version": "round5_transition_evaluation_v1",
        "protocol_path": str(protocol_path.resolve()),
        "protocol_sha256": _sha256(protocol_path),
        "manifest_path": str(manifest_path.resolve()),
        "manifest_sha256": _sha256(manifest_path),
        "drive_profile_path": str(drive_profile_path.resolve()),
        "drive_profile_sha256": _sha256(drive_profile_path),
        "m1_model_path": str(model_path.resolve()),
        "m1_model_sha256": _sha256(model_path),
        "rows": rows,
        "repeatability": repeatability,
        "supplementary_prediction_diagnostics": supplementary,
        "candidate_decision": decision,
    }
    _atomic_json(output_dir / "round5_transition_evaluation.json", result)
    _write_csv(output_dir / "round5_transition_samples.csv", rows)
    return result


def physics_diagnostics_to_samples(
    arrays: Mapping[str, object], *, auxiliary_velocity_limit_rad_s: float = 0.30
) -> list[dict[str, object]]:
    """Convert the frozen 50 Hz diagnostic trace to the causal predictor schema."""

    required = (
        "simulation_time_s",
        "raw_command_target",
        "applied_action",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "position_world_xy",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "yaw_world_rad",
    )
    converted = {name: np.asarray(arrays[name]) for name in required}
    count = len(converted["simulation_time_s"])
    if count < 2 or any(len(value) != count for value in converted.values()):
        raise ValueError("physics diagnostics must be aligned and contain at least two rows")
    samples: list[dict[str, object]] = []
    for index in range(count):
        request = converted["raw_command_target"][index]
        applied = converted["applied_action"][index]
        joint_position = converted["joint_position_rad"][index]
        joint_velocity = converted["joint_velocity_rad_s"][index]
        linear_velocity = converted["linear_velocity_world_m_s"][index]
        angular_velocity = converted["angular_velocity_world_rad_s"][index]
        samples.append(
            {
                "time_s": float(converted["simulation_time_s"][index]),
                "requested_main_rad_s": float(request[0]),
                "requested_auxiliary_rad": float(request[1]),
                "final_main_rad_s": float(applied[0]),
                "final_auxiliary_rad": float(applied[1]),
                "main_joint_velocity_rad_s": float(joint_velocity[0]),
                "auxiliary_joint_rad": float(joint_position[1]),
                "auxiliary_joint_velocity_rad_s": float(joint_velocity[1]),
                "navigation_heading_rad": float(converted["yaw_world_rad"][index]),
                "position_xy_m": list(map(float, converted["position_world_xy"][index])),
                "linear_velocity_world_xy_m_s": list(map(float, linear_velocity[:2])),
                "angular_velocity_world_rad_s": list(map(float, angular_velocity)),
                "auxiliary_velocity_supervisor_active": bool(
                    abs(float(joint_velocity[1])) > auxiliary_velocity_limit_rad_s
                ),
            }
        )
    return samples


def _fit_support(
    manifest: Mapping[str, object], transition_root: Path
) -> dict[str, object]:
    core = [item for item in manifest["sequences"] if item["dataset_role"] == "fit_core"]
    request_rows = [stage for item in core for stage in item["stages"]]
    request_main = np.asarray([row["main_request_rad_s"] for row in request_rows], dtype=float)
    request_aux = np.asarray([row["auxiliary_request_rad"] for row in request_rows], dtype=float)
    states: list[np.ndarray] = []
    for item in core:
        _, trace = _trace_for_sequence(transition_root, item)
        states.extend(_state_from_sample(sample) for sample in _trace_samples(trace))
    state_matrix = np.vstack(states)
    return {
        "request_main_min_max_rad_s": [float(request_main.min()), float(request_main.max())],
        "request_auxiliary_min_max_rad": [float(request_aux.min()), float(request_aux.max())],
        "state_min": state_matrix.min(axis=0).tolist(),
        "state_max": state_matrix.max(axis=0).tolist(),
        "state_names": list(STATE_NAMES),
    }


def _inside_bounds(value: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> bool:
    tolerance = 1e-9
    return bool(np.all(value >= lower - tolerance) and np.all(value <= upper + tolerance))


def _reference_transition_audit(
    states: Mapping[str, np.ndarray], support: Mapping[str, object]
) -> list[dict[str, object]]:
    reference_index = np.asarray(states["policy_reference_index"], dtype=int)
    main = np.asarray(states["policy_baseline_main_command_rad_s"], dtype=float)
    auxiliary = np.asarray(states["policy_auxiliary_target_angle_rad"], dtype=float)
    direction = np.asarray(states["kinodynamic_reference_drive_direction"], dtype=int)
    time = np.asarray(states["simulation_time_s"], dtype=float)
    joint_position = np.asarray(states["joint_position_rad"], dtype=float)
    joint_velocity = np.asarray(states["joint_velocity_rad_s"], dtype=float)
    linear_velocity = np.asarray(states["linear_velocity_world_m_s"], dtype=float)
    action_count = min(len(reference_index), len(main), len(auxiliary), len(time) - 1)
    events = [0]
    for index in range(1, action_count):
        if (
            reference_index[index] != reference_index[index - 1]
            or not math.isclose(main[index], main[index - 1], abs_tol=1e-12)
            or not math.isclose(auxiliary[index], auxiliary[index - 1], abs_tol=1e-12)
        ):
            events.append(index)
    main_bounds = np.asarray(support["request_main_min_max_rad_s"], dtype=float)
    aux_bounds = np.asarray(support["request_auxiliary_min_max_rad"], dtype=float)
    rows: list[dict[str, object]] = []
    previous_event = 0
    for action in events:
        state_index = action
        before_ref = int(reference_index[max(0, action - 1)])
        after_ref = int(reference_index[action])
        before_command = np.asarray(
            [main[max(0, action - 1)], auxiliary[max(0, action - 1)]], dtype=float
        )
        after_command = np.asarray([main[action], auxiliary[action]], dtype=float)
        command_changed = bool(np.linalg.norm(after_command - before_command, ord=np.inf) > 1e-12)
        snapshots: dict[str, object] = {}
        for label, offset in (("start", 0), ("after_0p5s", 5), ("after_1p5s", 15)):
            index = min(state_index + offset, len(time) - 1)
            snapshots[label] = {
                "time_s": float(time[index]),
                "main_joint_velocity_rad_s": float(joint_velocity[index, 0]),
                "auxiliary_joint_rad": float(joint_position[index, 1]),
                "auxiliary_joint_velocity_rad_s": float(joint_velocity[index, 1]),
                "planar_speed_m_s": float(np.linalg.norm(linear_velocity[index, :2])),
            }
        rows.append(
            {
                "action_index": action,
                "time_s": float(time[state_index]),
                "dwell_since_previous_event_s": float(time[state_index] - time[previous_event]),
                "reference_index_before": before_ref,
                "reference_index_after": after_ref,
                "reference_index_changed": before_ref != after_ref,
                "command_before_main_aux": before_command.tolist(),
                "command_after_main_aux": after_command.tolist(),
                "command_changed": command_changed,
                "same_target_index_change": bool(before_ref != after_ref and not command_changed),
                "drive_direction_before": int(direction[np.clip(before_ref, 0, len(direction) - 1)]),
                "drive_direction_after": int(direction[np.clip(after_ref, 0, len(direction) - 1)]),
                "drive_direction_handoff": bool(
                    direction[np.clip(before_ref, 0, len(direction) - 1)]
                    != direction[np.clip(after_ref, 0, len(direction) - 1)]
                ),
                "post_request_within_fit_core_box": bool(
                    main_bounds[0] <= after_command[0] <= main_bounds[1]
                    and aux_bounds[0] <= after_command[1] <= aux_bounds[1]
                ),
                "measured_state": snapshots,
            }
        )
        previous_event = action
    return rows


def run_shadow(
    *,
    protocol_path: Path,
    manifest_path: Path,
    transition_root: Path,
    drive_profile_path: Path,
    model_path: Path,
    fixed_root: Path,
    output_dir: Path,
) -> dict[str, object]:
    protocol = load_measurement_protocol(protocol_path)
    manifest = _load_manifest_payload(manifest_path)
    if _sha256(drive_profile_path) != "2de28cdfe98ecd6d45d5158261efb436acabaf31c1df4774f75b708a9086ddfd":
        raise RuntimeError("frozen V1 hash mismatch")
    drive_model = DriveModel.from_json(drive_profile_path)
    m1 = _load_json(model_path)
    if m1.get("schema_version") != M1_SCHEMA:
        raise ValueError("Round 5 M1 model is required")
    support = _fit_support(manifest, transition_root)
    request_main_bounds = np.asarray(support["request_main_min_max_rad_s"], dtype=float)
    request_aux_bounds = np.asarray(support["request_auxiliary_min_max_rad"], dtype=float)
    state_lower = np.asarray(support["state_min"], dtype=float)
    state_upper = np.asarray(support["state_max"], dtype=float)
    dt = float(protocol["time_alignment"]["dt_s"])
    stride = max(1, int(round(0.5 / dt)))
    rows: list[dict[str, object]] = []
    episodes: dict[str, object] = {}
    for states_path in sorted(fixed_root.glob("episode_*/states.npz")):
        episode_id = states_path.parent.name.removeprefix("episode_")
        diagnostics_path = states_path.parent / "physics_diagnostics.npz"
        with np.load(diagnostics_path, allow_pickle=False) as data:
            diagnostics = {name: np.asarray(data[name]) for name in data.files}
        with np.load(states_path, allow_pickle=False) as data:
            states = {name: np.asarray(data[name]) for name in data.files}
        samples = physics_diagnostics_to_samples(diagnostics)
        command_changes = set((np.flatnonzero(np.any(np.diff(diagnostics["raw_command_target"], axis=0) != 0.0, axis=1)) + 1).tolist())
        starts = set(range(0, len(samples) - 1, stride))
        starts.update(max(0, index - 1) for index in command_changes)
        for start_index in sorted(starts):
            start = samples[start_index]
            start_position = np.asarray(start["position_xy_m"], dtype=float)
            start_yaw = float(start["navigation_heading_rad"])
            for horizon in protocol["finite_horizons_s"]:
                step_count = int(round(float(horizon) / dt))
                end_index = start_index + step_count
                if end_index >= len(samples):
                    continue
                future = samples[start_index + 1 : end_index + 1]
                payload = {
                    "current_state": dict(zip(STATE_NAMES, _state_from_sample(start))),
                    "future_requests": [
                        {
                            "main_request_rad_s": sample["requested_main_rad_s"],
                            "auxiliary_request_rad": sample["requested_auxiliary_rad"],
                        }
                        for sample in future
                    ],
                }
                validate_predictor_payload(payload)
                m0 = _rollout_m0(drive_model, start, future, dt_s=dt)
                m1_result = _rollout_m1(m1, start, future)
                actual_world = np.asarray(samples[end_index]["position_xy_m"], dtype=float) - start_position
                actual_local = _rotate_world_to_local(actual_world, start_yaw)
                actual_heading = float(
                    _angle_wrap(float(samples[end_index]["navigation_heading_rad"]) - start_yaw)
                )
                request_matrix = np.asarray(
                    [[sample["requested_main_rad_s"], sample["requested_auxiliary_rad"]] for sample in future],
                    dtype=float,
                )
                row: dict[str, object] = {
                    "episode_id": episode_id,
                    "start_sample_index": start_index,
                    "end_sample_index": end_index,
                    "start_time_s": float(start["time_s"]),
                    "horizon_s": float(horizon),
                    "actual_local_delta_xy_m": actual_local.tolist(),
                    "actual_root_heading_change_rad": actual_heading,
                    "request_within_fit_core_box": bool(
                        np.all(request_matrix[:, 0] >= request_main_bounds[0] - 1e-9)
                        and np.all(request_matrix[:, 0] <= request_main_bounds[1] + 1e-9)
                        and np.all(request_matrix[:, 1] >= request_aux_bounds[0] - 1e-9)
                        and np.all(request_matrix[:, 1] <= request_aux_bounds[1] + 1e-9)
                    ),
                    "start_state_within_fit_core_box": _inside_bounds(
                        _state_from_sample(start), state_lower, state_upper
                    ),
                    "m0_out_of_support": bool(m0["out_of_support"]),
                }
                for name, prediction in (("m0", m0), ("m1", m1_result)):
                    predicted = np.asarray(prediction["local_delta_xy_m"], dtype=float)
                    row[f"{name}_predicted_local_delta_xy_m"] = predicted.tolist()
                    row[f"{name}_endpoint_position_error_m"] = float(
                        np.linalg.norm(predicted - actual_local)
                    )
                    row[f"{name}_root_heading_change_error_rad"] = float(
                        _angle_wrap(float(prediction["root_heading_change_rad"]) - actual_heading)
                    )
                rows.append(row)
        episodes[episode_id] = {
            "states_path": str(states_path.resolve()),
            "states_sha256": _sha256(states_path),
            "physics_diagnostics_path": str(diagnostics_path.resolve()),
            "physics_diagnostics_sha256": _sha256(diagnostics_path),
            "reference_transition_audit": _reference_transition_audit(states, support),
        }
    def aggregate(selected: Sequence[Mapping[str, object]], prefix: str) -> dict[str, object]:
        if not selected:
            return {"window_count": 0}
        endpoint = np.asarray([row[f"{prefix}_endpoint_position_error_m"] for row in selected], dtype=float)
        heading = np.asarray([abs(float(row[f"{prefix}_root_heading_change_error_rad"])) for row in selected], dtype=float)
        return {
            "window_count": len(selected),
            "endpoint_position_mae_m": float(endpoint.mean()),
            "endpoint_position_p90_m": float(np.percentile(endpoint, 90.0)),
            "root_heading_change_mae_rad": float(heading.mean()),
        }
    aggregates: dict[str, object] = {}
    for episode_id in episodes:
        episode_rows = [row for row in rows if row["episode_id"] == episode_id]
        in_support = [
            row for row in episode_rows
            if row["request_within_fit_core_box"] and row["start_state_within_fit_core_box"]
        ]
        aggregates[episode_id] = {
            "all_windows": {name: aggregate(episode_rows, name) for name in ("m0", "m1")},
            "fit_box_windows": {name: aggregate(in_support, name) for name in ("m0", "m1")},
            "fit_box_window_count": len(in_support),
            "total_window_count": len(episode_rows),
        }
    result = {
        "schema_version": "round5_fixed_reference_shadow_v1",
        "interpretation": "action-conditioned finite-horizon replay; not a closed-loop navigation result",
        "causal_contract": {
            "current_measured_state_at_window_start": True,
            "recorded_future_requested_actions": True,
            "future_measured_state_as_input": False,
            "future_supervisor_output_as_input": False,
            "truth_reset_inside_window": False,
        },
        "protocol_sha256": _sha256(protocol_path),
        "manifest_sha256": _sha256(manifest_path),
        "m1_model_sha256": _sha256(model_path),
        "fit_support": support,
        "episodes": episodes,
        "aggregates": aggregates,
        "rows": rows,
    }
    _atomic_json(output_dir / "round5_fixed_reference_shadow.json", result)
    _write_csv(output_dir / "round5_fixed_reference_shadow_windows.csv", rows)
    transition_rows = [
        {"episode_id": episode_id, **row}
        for episode_id, payload in episodes.items()
        for row in payload["reference_transition_audit"]
    ]
    _write_csv(output_dir / "round5_fixed_reference_transition_audit.csv", transition_rows)
    return result


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    p0 = sub.add_parser("p0")
    p0.add_argument("--protocol", type=Path, required=True)
    p0.add_argument("--round4-root", type=Path, required=True)
    p0.add_argument("--round4-analysis", type=Path, required=True)
    p0.add_argument("--fixed-root", type=Path, required=True)
    p0.add_argument("--out", type=Path, required=True)
    fit = sub.add_parser("fit")
    fit.add_argument("--protocol", type=Path, required=True)
    fit.add_argument("--manifest", type=Path, required=True)
    fit.add_argument("--transition-root", type=Path, required=True)
    fit.add_argument("--out", type=Path, required=True)
    evaluate = sub.add_parser("evaluate")
    evaluate.add_argument("--protocol", type=Path, required=True)
    evaluate.add_argument("--manifest", type=Path, required=True)
    evaluate.add_argument("--transition-root", type=Path, required=True)
    evaluate.add_argument("--drive-profile", type=Path, required=True)
    evaluate.add_argument("--model", type=Path, required=True)
    evaluate.add_argument("--out", type=Path, required=True)
    shadow = sub.add_parser("shadow")
    shadow.add_argument("--protocol", type=Path, required=True)
    shadow.add_argument("--manifest", type=Path, required=True)
    shadow.add_argument("--transition-root", type=Path, required=True)
    shadow.add_argument("--drive-profile", type=Path, required=True)
    shadow.add_argument("--model", type=Path, required=True)
    shadow.add_argument("--fixed-root", type=Path, required=True)
    shadow.add_argument("--out", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.mode == "p0":
        result = run_p0(
            protocol_path=args.protocol.resolve(),
            round4_root=args.round4_root.resolve(),
            round4_analysis_path=args.round4_analysis.resolve(),
            fixed_root=args.fixed_root.resolve(),
            output_dir=args.out.resolve(),
        )
        print(args.out.resolve() / "round5_measurement_comparison.json")
    elif args.mode == "fit":
        result = run_fit(
            protocol_path=args.protocol.resolve(),
            manifest_path=args.manifest.resolve(),
            transition_root=args.transition_root.resolve(),
            output_path=args.out.resolve(),
        )
        print(args.out.resolve())
    elif args.mode == "evaluate":
        result = run_evaluate(
            protocol_path=args.protocol.resolve(),
            manifest_path=args.manifest.resolve(),
            transition_root=args.transition_root.resolve(),
            drive_profile_path=args.drive_profile.resolve(),
            model_path=args.model.resolve(),
            output_dir=args.out.resolve(),
        )
        print(args.out.resolve() / "round5_transition_evaluation.json")
    else:
        result = run_shadow(
            protocol_path=args.protocol.resolve(),
            manifest_path=args.manifest.resolve(),
            transition_root=args.transition_root.resolve(),
            drive_profile_path=args.drive_profile.resolve(),
            model_path=args.model.resolve(),
            fixed_root=args.fixed_root.resolve(),
            output_dir=args.out.resolve(),
        )
        print(args.out.resolve() / "round5_fixed_reference_shadow.json")
    return 0 if result else 1


if __name__ == "__main__":
    raise SystemExit(main())
