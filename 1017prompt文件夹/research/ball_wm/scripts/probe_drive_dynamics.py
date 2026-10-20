"""Coupled main/auxiliary drive-dynamics calibration for the ball robot.

Isaac Sim is imported only by the runtime entry point so that experiment
construction and trace analysis remain available to CPU-only tests and tools.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Sequence

import numpy as np

from action_smoothing import (
    ActionSmoother,
    DEFAULT_AUXILIARY_KD,
    DEFAULT_AUXILIARY_KP,
    DEFAULT_AUXILIARY_RATE_RAD_S,
    DEFAULT_MAIN_ACCELERATION_RAD_S2,
    DEFAULT_MAIN_DERIVATIVE_GAIN,
)


EXPECTED_PROTOCOL_ID = "ball_action_v2_aux30"
EXPECTED_ACTION_NAMES = (
    "main_joint_velocity_cmd",
    "auxiliary_joint_angle_cmd",
)
STRAIGHT_MAIN_COMMANDS_RAD_S = (-1.5, -1.0, -0.5, 0.5, 1.0, 1.5)
TURN_MAIN_COMMANDS_RAD_S = (-1.5, -1.0, -0.5, 0.5, 1.0, 1.5)
TURN_AUXILIARY_DEG = (-30.0, -20.0, -10.0, 10.0, 20.0, 30.0)
BRAKING_MAIN_COMMANDS_RAD_S = (-1.5, -1.0, -0.5, 0.5, 1.0, 1.5)
STEERING_STEP_MAIN_COMMANDS_RAD_S = (0.5, 1.0, 1.5)
STEERING_STEP_AUXILIARY_DEG = (-20.0, 20.0)
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTRACT = SCRIPT_DIR.parent / "configs" / "data_contract.yaml"
ROBOT_START = np.asarray([0.0, 0.0, 0.45], dtype=np.float64)
ROBOT_USD = (
    SCRIPT_DIR.parents[2]
    / "project"
    / "rl_env_0.4"
    / "rl_env"
    / "rotunbot_wheel_sim_model_0.4"
    / "Rotunbot_wheel.usd"
)
ROBOT_ROOT = "/World/BallRobot"
SETTLE_STEPS = 50


@dataclass(frozen=True)
class CalibrationCase:
    """One deterministic command condition in the calibration matrix."""

    case_id: str
    mode: str
    main_command_rad_s: float
    auxiliary_command_rad: float

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class TrialStage:
    phase: str
    duration_s: float
    main_target_rad_s: float
    auxiliary_target_rad: float


def _case_number(value: float) -> str:
    sign = "neg" if value < 0.0 else "pos"
    magnitude = f"{abs(value):.1f}".replace(".", "p")
    return f"{sign}{magnitude}"


def _finite_vector(
    values: object,
    *,
    name: str,
    columns: int | None = None,
    minimum_length: int = 2,
) -> np.ndarray:
    try:
        array = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain finite numeric samples") from error
    expected_ndim = 2 if columns is not None else 1
    if array.ndim != expected_ndim or (
        columns is not None and array.shape[1] != columns
    ):
        shape = "(N,)" if columns is None else f"(N, {columns})"
        raise ValueError(f"{name} must have shape {shape}")
    if array.shape[0] < minimum_length or not np.isfinite(array).all():
        raise ValueError(
            f"{name} must contain at least {minimum_length} finite samples"
        )
    return array


def _validate_equal_lengths(**arrays: Sequence[object]) -> int:
    lengths = {len(value) for value in arrays.values()}
    if len(lengths) != 1:
        raise ValueError("all calibration traces must have equal lengths")
    return lengths.pop()


def _first_response_time(
    time_s: np.ndarray,
    measured: np.ndarray,
    target: float,
    *,
    fraction: float = 0.9,
) -> float | None:
    threshold = abs(target) * fraction
    if threshold <= np.finfo(np.float64).eps:
        return 0.0
    indices = np.flatnonzero(np.abs(measured) >= threshold)
    if not indices.size:
        return None
    return float(time_s[int(indices[0])] - time_s[0])


def calibration_cases_from_contract(contract: object) -> tuple[CalibrationCase, ...]:
    """Validate the V2 action contract and build the fixed 54-case matrix."""

    if getattr(contract, "action_protocol_id", None) != EXPECTED_PROTOCOL_ID:
        raise ValueError(f"calibration requires {EXPECTED_PROTOCOL_ID}")
    if getattr(contract, "action_names", None) != EXPECTED_ACTION_NAMES:
        raise ValueError("calibration requires the two-field ball action protocol")
    minimum = np.asarray(getattr(contract, "action_minimum", ()), dtype=np.float64)
    maximum = np.asarray(getattr(contract, "action_maximum", ()), dtype=np.float64)
    if (
        minimum.shape != (2,)
        or maximum.shape != (2,)
        or not np.isfinite([minimum, maximum]).all()
    ):
        raise ValueError("action bounds must be two finite vectors")
    required_minimum = np.asarray((-1.5, math.radians(-30.0)))
    required_maximum = np.asarray((1.5, math.radians(30.0)))
    if np.any(minimum > required_minimum + 1e-6) or np.any(
        maximum < required_maximum - 1e-6
    ):
        raise ValueError("action bounds do not cover the calibration matrix")

    cases: list[CalibrationCase] = []
    for main in STRAIGHT_MAIN_COMMANDS_RAD_S:
        cases.append(
            CalibrationCase(
                case_id=f"straight_main_{_case_number(main)}",
                mode="straight",
                main_command_rad_s=main,
                auxiliary_command_rad=0.0,
            )
        )
    for main in TURN_MAIN_COMMANDS_RAD_S:
        for auxiliary_deg in TURN_AUXILIARY_DEG:
            cases.append(
                CalibrationCase(
                    case_id=(
                        f"turn_main_{_case_number(main)}_"
                        f"aux_{_case_number(auxiliary_deg)}deg"
                    ),
                    mode="steady_turn",
                    main_command_rad_s=main,
                    auxiliary_command_rad=math.radians(auxiliary_deg),
                )
            )
    for main in BRAKING_MAIN_COMMANDS_RAD_S:
        cases.append(
            CalibrationCase(
                case_id=f"brake_main_{_case_number(main)}",
                mode="braking",
                main_command_rad_s=main,
                auxiliary_command_rad=0.0,
            )
        )
    for main in STEERING_STEP_MAIN_COMMANDS_RAD_S:
        for auxiliary_deg in STEERING_STEP_AUXILIARY_DEG:
            cases.append(
                CalibrationCase(
                    case_id=(
                        f"steering_step_main_{_case_number(main)}_"
                        f"aux_{_case_number(auxiliary_deg)}deg"
                    ),
                    mode="steering_step",
                    main_command_rad_s=main,
                    auxiliary_command_rad=math.radians(auxiliary_deg),
                )
            )
    return tuple(cases)


def summarize_motion_trial(
    *,
    case_id: str,
    mode: str,
    time_s: Sequence[float],
    phase: Sequence[str],
    main_target_rad_s: Sequence[float],
    auxiliary_target_rad: Sequence[float],
    main_joint_velocity_rad_s: Sequence[float],
    auxiliary_joint_rad: Sequence[float],
    yaw_rad: Sequence[float],
    position_xy_m: Sequence[Sequence[float]],
    linear_velocity_world_xy_m_s: Sequence[Sequence[float]],
) -> dict[str, object]:
    """Summarize one command trial without imposing guessed physical limits."""

    if not isinstance(case_id, str) or not case_id:
        raise ValueError("case_id must be a non-empty string")
    if mode not in {"straight", "steady_turn", "braking", "steering_step"}:
        raise ValueError("unsupported calibration mode")
    sample_count = _validate_equal_lengths(
        time_s=time_s,
        phase=phase,
        main_target_rad_s=main_target_rad_s,
        auxiliary_target_rad=auxiliary_target_rad,
        main_joint_velocity_rad_s=main_joint_velocity_rad_s,
        auxiliary_joint_rad=auxiliary_joint_rad,
        yaw_rad=yaw_rad,
        position_xy_m=position_xy_m,
        linear_velocity_world_xy_m_s=linear_velocity_world_xy_m_s,
    )
    time = _finite_vector(time_s, name="time_s")
    if np.any(np.diff(time) <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    phases = np.asarray(phase, dtype=str)
    if phases.shape != (sample_count,) or np.any(np.char.str_len(phases) == 0):
        raise ValueError("phase must contain one non-empty label per sample")
    main_target = _finite_vector(main_target_rad_s, name="main_target_rad_s")
    auxiliary_target = _finite_vector(
        auxiliary_target_rad, name="auxiliary_target_rad"
    )
    main_actual = _finite_vector(
        main_joint_velocity_rad_s, name="main_joint_velocity_rad_s"
    )
    auxiliary_actual = _finite_vector(
        auxiliary_joint_rad, name="auxiliary_joint_rad"
    )
    yaw = np.unwrap(_finite_vector(yaw_rad, name="yaw_rad"))
    positions = _finite_vector(position_xy_m, name="position_xy_m", columns=2)
    world_velocity = _finite_vector(
        linear_velocity_world_xy_m_s,
        name="linear_velocity_world_xy_m_s",
        columns=2,
    )

    steady_count = max(2, int(math.ceil(sample_count * 0.4)))
    steady = slice(sample_count - steady_count, sample_count)
    planar_speed = np.linalg.norm(world_velocity, axis=1)
    signed_speed = np.copysign(
        planar_speed,
        np.where(np.abs(main_actual) > 1e-6, main_actual, main_target),
    )
    yaw_rate = np.gradient(yaw, time)
    steady_main = float(np.median(main_actual[steady]))
    steady_auxiliary = float(np.median(auxiliary_actual[steady]))
    steady_speed = float(np.median(signed_speed[steady]))
    steady_yaw_rate = float(np.median(yaw_rate[steady]))
    if abs(steady_speed) <= 1e-4:
        curvature = None
        turning_radius = None
    else:
        curvature = steady_yaw_rate / steady_speed
        turning_radius = None if abs(curvature) <= 1e-4 else 1.0 / curvature

    main_command = float(main_target[-1])
    auxiliary_command = float(auxiliary_target[-1])
    samples = [
        {
            "time_s": float(time[index]),
            "phase": str(phases[index]),
            "main_target_rad_s": float(main_target[index]),
            "auxiliary_target_rad": float(auxiliary_target[index]),
            "main_joint_velocity_rad_s": float(main_actual[index]),
            "auxiliary_joint_rad": float(auxiliary_actual[index]),
            "yaw_rad": float(yaw[index]),
            "position_xy_m": positions[index].tolist(),
            "linear_velocity_world_xy_m_s": world_velocity[index].tolist(),
            "planar_speed_m_s": float(planar_speed[index]),
        }
        for index in range(sample_count)
    ]
    return {
        "case_id": case_id,
        "mode": mode,
        "main_command_rad_s": main_command,
        "auxiliary_command_rad": auxiliary_command,
        "steady_main_joint_velocity_rad_s": steady_main,
        "steady_auxiliary_joint_rad": steady_auxiliary,
        "steady_forward_speed_m_s": steady_speed,
        "steady_yaw_rate_rad_s": steady_yaw_rate,
        "steady_curvature_1_m": curvature,
        "signed_turning_radius_m": turning_radius,
        "main_time_to_90_percent_s": _first_response_time(
            time, main_actual, main_command
        ),
        "auxiliary_time_to_90_percent_s": _first_response_time(
            time, auxiliary_actual, auxiliary_command
        ),
        "path_length_m": float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
        "samples": samples,
    }


def summarize_braking_trial(
    *,
    time_s: Sequence[float],
    phase: Sequence[str],
    position_xy_m: Sequence[Sequence[float]],
    planar_speed_m_s: Sequence[float],
    stop_speed_threshold_m_s: float = 0.05,
) -> dict[str, object]:
    """Measure stopping time and distance from the first braking sample."""

    _validate_equal_lengths(
        time_s=time_s,
        phase=phase,
        position_xy_m=position_xy_m,
        planar_speed_m_s=planar_speed_m_s,
    )
    time = _finite_vector(time_s, name="time_s")
    positions = _finite_vector(position_xy_m, name="position_xy_m", columns=2)
    speed = _finite_vector(planar_speed_m_s, name="planar_speed_m_s")
    phases = np.asarray(phase, dtype=str)
    threshold = float(stop_speed_threshold_m_s)
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ValueError("stop_speed_threshold_m_s must be finite and positive")
    brake_indices = np.flatnonzero(phases == "brake")
    if not brake_indices.size:
        raise ValueError("braking trial requires a brake phase")
    start = int(brake_indices[0])
    stopped = brake_indices[speed[brake_indices] <= threshold]
    stopped_within_trial = bool(stopped.size)
    end = int(stopped[0]) if stopped_within_trial else int(brake_indices[-1])
    distance = float(
        np.linalg.norm(np.diff(positions[start : end + 1], axis=0), axis=1).sum()
    )
    return {
        "brake_start_time_s": float(time[start]),
        "initial_braking_speed_m_s": float(speed[start]),
        "stop_time_s": float(time[end] - time[start]),
        "stopping_distance_m": distance,
        "stop_speed_threshold_m_s": threshold,
        "stopped_within_trial": stopped_within_trial,
    }


def summarize_steering_transient(
    *,
    time_s: Sequence[float],
    phase: Sequence[str],
    auxiliary_command_rad: float,
    auxiliary_joint_rad: Sequence[float],
    yaw_rate_rad_s: Sequence[float],
) -> dict[str, object]:
    """Measure auxiliary rise/release timing and the resulting yaw-rate peak."""

    _validate_equal_lengths(
        time_s=time_s,
        phase=phase,
        auxiliary_joint_rad=auxiliary_joint_rad,
        yaw_rate_rad_s=yaw_rate_rad_s,
    )
    time = _finite_vector(time_s, name="time_s")
    if np.any(np.diff(time) <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    phases = np.asarray(phase, dtype=str)
    auxiliary = _finite_vector(auxiliary_joint_rad, name="auxiliary_joint_rad")
    yaw_rate = _finite_vector(yaw_rate_rad_s, name="yaw_rate_rad_s")
    command = float(auxiliary_command_rad)
    if not math.isfinite(command) or abs(command) <= 1e-6:
        raise ValueError("auxiliary_command_rad must be finite and non-zero")
    steer_indices = np.flatnonzero(phases == "steer")
    release_indices = np.flatnonzero(phases == "release")
    if not steer_indices.size or not release_indices.size:
        raise ValueError("steering transient requires steer and release phases")
    steer_start = int(steer_indices[0])
    release_start = int(release_indices[0])
    risen = steer_indices[np.abs(auxiliary[steer_indices]) >= 0.9 * abs(command)]
    released = release_indices[
        np.abs(auxiliary[release_indices]) <= 0.1 * abs(command)
    ]
    rise_time = (
        float(time[int(risen[0])] - time[steer_start]) if risen.size else None
    )
    release_time = (
        float(time[int(released[0])] - time[release_start])
        if released.size
        else None
    )
    return {
        "auxiliary_command_rad": command,
        "auxiliary_rise_time_90_percent_s": rise_time,
        "auxiliary_release_time_10_percent_s": release_time,
        "peak_abs_yaw_rate_rad_s": float(np.max(np.abs(yaw_rate[steer_indices]))),
        "returned_to_neutral": bool(
            abs(float(auxiliary[-1])) <= 0.1 * abs(command)
        ),
    }


def trial_stages(
    case: CalibrationCase,
    *,
    main_acceleration_rad_s2: float,
    auxiliary_rate_rad_s: float,
) -> tuple[TrialStage, ...]:
    """Build physical-duration command stages for one calibration case."""

    if not isinstance(case, CalibrationCase):
        raise TypeError("case must be a CalibrationCase")
    main_acceleration = float(main_acceleration_rad_s2)
    auxiliary_rate = float(auxiliary_rate_rad_s)
    if not np.isfinite([main_acceleration, auxiliary_rate]).all() or (
        main_acceleration <= 0.0 or auxiliary_rate <= 0.0
    ):
        raise ValueError("command rate limits must be finite and positive")
    main_ramp_s = abs(case.main_command_rad_s) / main_acceleration
    auxiliary_ramp_s = abs(case.auxiliary_command_rad) / auxiliary_rate
    if case.mode in {"straight", "steady_turn"}:
        return (
            TrialStage(
                phase="drive",
                duration_s=max(main_ramp_s, auxiliary_ramp_s) + 2.0,
                main_target_rad_s=case.main_command_rad_s,
                auxiliary_target_rad=case.auxiliary_command_rad,
            ),
        )
    if case.mode == "braking":
        return (
            TrialStage(
                phase="drive",
                duration_s=main_ramp_s + 1.5,
                main_target_rad_s=case.main_command_rad_s,
                auxiliary_target_rad=0.0,
            ),
            TrialStage(
                phase="brake",
                duration_s=main_ramp_s + 1.5,
                main_target_rad_s=0.0,
                auxiliary_target_rad=0.0,
            ),
        )
    if case.mode == "steering_step":
        return (
            TrialStage(
                phase="baseline",
                duration_s=main_ramp_s + 1.0,
                main_target_rad_s=case.main_command_rad_s,
                auxiliary_target_rad=0.0,
            ),
            TrialStage(
                phase="steer",
                duration_s=auxiliary_ramp_s + 1.5,
                main_target_rad_s=case.main_command_rad_s,
                auxiliary_target_rad=case.auxiliary_command_rad,
            ),
            TrialStage(
                phase="release",
                duration_s=auxiliary_ramp_s + 1.0,
                main_target_rad_s=case.main_command_rad_s,
                auxiliary_target_rad=0.0,
            ),
        )
    raise ValueError(f"unsupported calibration mode: {case.mode!r}")


def _yaw_from_wxyz(quaternion: object) -> float:
    orientation = np.asarray(quaternion, dtype=np.float64)
    if orientation.shape != (4,) or not np.isfinite(orientation).all():
        raise RuntimeError("robot quaternion must be a finite vector with shape (4,)")
    norm = float(np.linalg.norm(orientation))
    if norm <= np.finfo(np.float64).eps:
        raise RuntimeError("robot quaternion must have non-zero norm")
    w, x, y, z = orientation / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write("\n")
    temporary.replace(path)


def _finite_float(value: object, *, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _drive_direction(main_command_rad_s: float) -> str:
    if main_command_rad_s > 0.0:
        return "forward"
    if main_command_rad_s < 0.0:
        return "reverse"
    raise ValueError("main command must be non-zero")


def compact_drive_profile(report: Mapping[str, object]) -> dict[str, object]:
    """Export only complete, measured operating points used by the Oracle."""

    if report.get("probe") != "coupled_drive_dynamics":
        raise ValueError("report must be a coupled_drive_dynamics object")
    if report.get("action_protocol_id") != EXPECTED_PROTOCOL_ID:
        raise ValueError(f"report must use {EXPECTED_PROTOCOL_ID}")
    timestamp = report.get("measurement_timestamp_utc")
    if not isinstance(timestamp, str):
        raise ValueError("measurement_timestamp_utc must be an ISO-8601 string")
    try:
        parsed_timestamp = datetime.fromisoformat(timestamp)
    except ValueError as error:
        raise ValueError("measurement_timestamp_utc must be an ISO-8601 string") from error
    if parsed_timestamp.tzinfo is None:
        raise ValueError("measurement_timestamp_utc must include a timezone")

    controller = report.get("controller")
    if not isinstance(controller, Mapping):
        raise ValueError("controller metadata is required")
    expected_controller = {
        "main_acceleration_limit_rad_s2": DEFAULT_MAIN_ACCELERATION_RAD_S2,
        "main_derivative_gain": DEFAULT_MAIN_DERIVATIVE_GAIN,
        "auxiliary_kp": DEFAULT_AUXILIARY_KP,
        "auxiliary_kd": DEFAULT_AUXILIARY_KD,
        "auxiliary_target_rate_limit_rad_s": DEFAULT_AUXILIARY_RATE_RAD_S,
    }
    normalized_controller: dict[str, float] = {}
    for name, expected in expected_controller.items():
        actual = _finite_float(controller.get(name), name=f"controller.{name}")
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("controller metadata does not match the accepted settings")
        normalized_controller[name] = actual

    cases = report.get("cases")
    if not isinstance(cases, list):
        raise ValueError("report cases must be a list")
    if report.get("case_count") != len(cases):
        raise ValueError("case_count does not match report cases")

    expected_main = set(STRAIGHT_MAIN_COMMANDS_RAD_S)
    expected_auxiliary = {0.0, *(math.radians(value) for value in TURN_AUXILIARY_DEG)}
    steering_by_key: dict[tuple[float, float], dict[str, object]] = {}
    braking_by_main: dict[float, dict[str, object]] = {}
    for item in cases:
        if not isinstance(item, Mapping):
            raise ValueError("each calibration case must be an object")
        mode = item.get("mode")
        if mode not in {"straight", "steady_turn", "braking", "steering_step"}:
            raise ValueError("calibration case has unsupported mode")
        if mode == "steering_step":
            continue
        main = _finite_float(item.get("main_command_rad_s"), name="main_command_rad_s")
        auxiliary = _finite_float(
            item.get("auxiliary_command_rad"), name="auxiliary_command_rad"
        )
        direction = _drive_direction(main)
        if mode in {"straight", "steady_turn"}:
            key = (main, auxiliary)
            if key in steering_by_key:
                raise ValueError("duplicate steering operating point")
            steady_main = _finite_float(
                item.get("steady_main_joint_velocity_rad_s"),
                name="steady_main_joint_velocity_rad_s",
            )
            speed = abs(
                _finite_float(
                    item.get("steady_forward_speed_m_s"),
                    name="steady_forward_speed_m_s",
                )
            )
            curvature = _finite_float(
                item.get("steady_curvature_1_m"), name="steady_curvature_1_m"
            )
            radius_value = item.get("signed_turning_radius_m")
            radius = (
                None
                if radius_value is None
                else _finite_float(radius_value, name="signed_turning_radius_m")
            )
            steering_by_key[key] = {
                "main_command_rad_s": main,
                "drive_direction": direction,
                "auxiliary_angle_rad": auxiliary,
                "measured_main_joint_velocity_rad_s": steady_main,
                "speed_m_s": speed,
                "curvature_1_m": curvature,
                "signed_turning_radius_m": radius,
            }
        elif mode == "braking":
            if main in braking_by_main:
                raise ValueError("duplicate braking operating point")
            braking = item.get("braking")
            if not isinstance(braking, Mapping):
                raise ValueError("braking case requires braking measurements")
            if braking.get("stopped_within_trial") is not True:
                raise ValueError("braking measurement must reach the stop threshold")
            braking_by_main[main] = {
                "main_command_rad_s": main,
                "drive_direction": direction,
                "speed_m_s": abs(
                    _finite_float(
                        braking.get("initial_braking_speed_m_s"),
                        name="initial_braking_speed_m_s",
                    )
                ),
                "stopping_distance_m": _finite_float(
                    braking.get("stopping_distance_m"), name="stopping_distance_m"
                ),
                "stop_time_s": _finite_float(
                    braking.get("stop_time_s"), name="stop_time_s"
                ),
            }

    expected_steering = {
        (main, auxiliary)
        for main in expected_main
        for auxiliary in expected_auxiliary
    }
    if set(steering_by_key) != expected_steering:
        raise ValueError("report must contain the complete signed steering matrix")
    if set(braking_by_main) != expected_main:
        raise ValueError("report must contain the complete signed braking matrix")

    return {
        "schema_version": "drive_dynamics_v1",
        "action_protocol_id": EXPECTED_PROTOCOL_ID,
        "measurement_timestamp_utc": timestamp,
        "physics_hz": int(report.get("physics_hz", 0)),
        "controller": normalized_controller,
        "steering_samples": [steering_by_key[key] for key in sorted(steering_by_key)],
        "braking_samples": [braking_by_main[key] for key in sorted(braking_by_main)],
    }


def write_compact_drive_profile(
    output_path: str | Path, report: Mapping[str, object]
) -> Path:
    """Atomically write the validated compact drive profile."""

    output = Path(output_path)
    _atomic_json(output, compact_drive_profile(report))
    return output


def run_calibration(
    contract_path: Path,
    output_dir: Path,
    *,
    gui: bool,
    main_acceleration_rad_s2: float,
    auxiliary_kp: float,
    auxiliary_kd: float,
    auxiliary_rate_rad_s: float,
    profile_output_path: Path | None = None,
) -> dict[str, object]:
    """Run all ground-only calibration cases using the accepted controller."""

    controller_parameters = np.asarray(
        [main_acceleration_rad_s2, auxiliary_kp, auxiliary_kd, auxiliary_rate_rad_s],
        dtype=np.float64,
    )
    if not np.isfinite(controller_parameters).all() or np.any(
        controller_parameters <= 0.0
    ):
        raise ValueError(
            "main acceleration, auxiliary Kp, Kd, and rate limit must be "
            "positive and finite"
        )

    # Keep Isaac imports below input validation: CPU-only callers can construct
    # and analyse experiments without initializing Kit or a GPU context.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {"headless": not gui, "width": 1280, "height": 720}
    )
    try:
        from collection_schema import load_contract

        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction

        contract = load_contract(contract_path)
        cases = calibration_cases_from_contract(contract)
        physics_hz = int(contract.schedule.physics_hz)
        physics_dt = 1.0 / physics_hz
        action_minimum = np.asarray(contract.action_minimum, dtype=np.float64)
        action_maximum = np.asarray(contract.action_maximum, dtype=np.float64)

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=physics_dt,
            rendering_dt=1.0 / contract.schedule.stereo_hz,
        )
        world.scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_ROOT)
        robot = world.scene.add(
            Articulation(
                prim_path=ROBOT_ROOT,
                name="drive_calibration_ball_robot",
                position=ROBOT_START,
            )
        )
        world.reset()
        if robot.num_dof < 2:
            raise RuntimeError("drive calibration requires at least two robot joints")
        controller = robot.get_articulation_controller()
        kps = np.asarray([0.0, auxiliary_kp, 0.0], dtype=np.float32)
        kds = np.asarray(
            [DEFAULT_MAIN_DERIVATIVE_GAIN, auxiliary_kd, 0.0],
            dtype=np.float32,
        )
        zero_action = ArticulationAction(
            joint_velocities=np.asarray([0.0, np.nan, np.nan], dtype=np.float32),
            joint_positions=np.asarray([np.nan, 0.0, np.nan], dtype=np.float32),
        )

        case_summaries: list[dict[str, object]] = []
        for case_index, case in enumerate(cases, start=1):
            if gui and not simulation_app.is_running():
                raise RuntimeError(
                    "Isaac GUI was closed before drive calibration completed"
                )
            print(
                f"[{case_index:02d}/{len(cases):02d}] {case.case_id}",
                flush=True,
            )
            robot.set_world_pose(
                position=ROBOT_START.copy(),
                orientation=np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            )
            robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
            robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
            robot.set_joint_positions(np.zeros(robot.num_dof, dtype=np.float32))
            robot.set_joint_velocities(np.zeros(robot.num_dof, dtype=np.float32))
            controller.set_gains(kps=kps, kds=kds)
            for _ in range(SETTLE_STEPS):
                controller.apply_action(zero_action)
                world.step(render=gui)

            smoother = ActionSmoother(
                dt_s=physics_dt,
                max_main_acceleration_rad_s2=main_acceleration_rad_s2,
                max_auxiliary_rate_rad_s=auxiliary_rate_rad_s,
            )
            smoother.reset()
            traces: dict[str, list[object]] = {
                "time_s": [],
                "phase": [],
                "main_target_rad_s": [],
                "auxiliary_target_rad": [],
                "main_joint_velocity_rad_s": [],
                "auxiliary_joint_rad": [],
                "yaw_rad": [],
                "position_xy_m": [],
                "linear_velocity_world_xy_m_s": [],
            }
            elapsed_steps = 0
            stages = trial_stages(
                case,
                main_acceleration_rad_s2=main_acceleration_rad_s2,
                auxiliary_rate_rad_s=auxiliary_rate_rad_s,
            )
            for stage in stages:
                stage_steps = max(1, int(round(stage.duration_s * physics_hz)))
                for _ in range(stage_steps):
                    if gui and not simulation_app.is_running():
                        raise RuntimeError(
                            "Isaac GUI was closed before drive calibration completed"
                        )
                    applied = smoother.step(
                        [stage.main_target_rad_s, stage.auxiliary_target_rad],
                        action_minimum,
                        action_maximum,
                    )
                    controller.apply_action(
                        ArticulationAction(
                            joint_velocities=np.asarray(
                                [applied[0], np.nan, np.nan], dtype=np.float32
                            ),
                            joint_positions=np.asarray(
                                [np.nan, applied[1], np.nan], dtype=np.float32
                            ),
                        )
                    )
                    world.step(render=gui)
                    elapsed_steps += 1

                    position, quaternion = robot.get_world_pose()
                    joint_positions = np.asarray(
                        robot.get_joint_positions(), dtype=np.float64
                    )
                    joint_velocities = np.asarray(
                        robot.get_joint_velocities(), dtype=np.float64
                    )
                    linear_velocity = np.asarray(
                        robot.get_linear_velocity(), dtype=np.float64
                    )
                    if (
                        joint_positions.shape[0] < 2
                        or joint_velocities.shape[0] < 1
                        or linear_velocity.shape != (3,)
                    ):
                        raise RuntimeError("robot returned an incomplete state vector")
                    traces["time_s"].append(elapsed_steps * physics_dt)
                    traces["phase"].append(stage.phase)
                    traces["main_target_rad_s"].append(float(applied[0]))
                    traces["auxiliary_target_rad"].append(float(applied[1]))
                    traces["main_joint_velocity_rad_s"].append(
                        float(joint_velocities[0])
                    )
                    traces["auxiliary_joint_rad"].append(float(joint_positions[1]))
                    traces["yaw_rad"].append(_yaw_from_wxyz(quaternion))
                    traces["position_xy_m"].append(
                        [float(position[0]), float(position[1])]
                    )
                    traces["linear_velocity_world_xy_m_s"].append(
                        [float(linear_velocity[0]), float(linear_velocity[1])]
                    )

            summary = summarize_motion_trial(
                case_id=case.case_id,
                mode=case.mode,
                **traces,
            )
            # Braking and steering-step trials end at a zero target, so retain
            # the experimental condition explicitly at the top level.
            summary["main_command_rad_s"] = case.main_command_rad_s
            summary["auxiliary_command_rad"] = case.auxiliary_command_rad
            if case.mode == "braking":
                summary["braking"] = summarize_braking_trial(
                    time_s=traces["time_s"],
                    phase=traces["phase"],
                    position_xy_m=traces["position_xy_m"],
                    planar_speed_m_s=[
                        float(np.linalg.norm(velocity))
                        for velocity in traces["linear_velocity_world_xy_m_s"]
                    ],
                )
            if case.mode == "steering_step":
                time_s = np.asarray(traces["time_s"], dtype=np.float64)
                yaw_rad = np.unwrap(np.asarray(traces["yaw_rad"], dtype=np.float64))
                summary["steering_transient"] = summarize_steering_transient(
                    time_s=time_s,
                    phase=traces["phase"],
                    auxiliary_command_rad=case.auxiliary_command_rad,
                    auxiliary_joint_rad=traces["auxiliary_joint_rad"],
                    yaw_rate_rad_s=np.gradient(yaw_rad, time_s),
                )
            case_summaries.append(summary)

        report: dict[str, object] = {
            "schema_version": "1.0",
            "probe": "coupled_drive_dynamics",
            "measurement_timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "action_protocol_id": contract.action_protocol_id,
            "physics_hz": physics_hz,
            "settle_steps": SETTLE_STEPS,
            "case_count": len(case_summaries),
            "controller": {
                "main_acceleration_limit_rad_s2": main_acceleration_rad_s2,
                "main_derivative_gain": DEFAULT_MAIN_DERIVATIVE_GAIN,
                "auxiliary_kp": auxiliary_kp,
                "auxiliary_kd": auxiliary_kd,
                "auxiliary_target_rate_limit_rad_s": auxiliary_rate_rad_s,
            },
            "cases": case_summaries,
        }
        output_dir.mkdir(parents=True, exist_ok=True)
        report_path = output_dir / "drive_dynamics_calibration.json"
        _atomic_json(report_path, report)
        _render_report_in_clean_process(report_path)
        if profile_output_path is not None:
            write_compact_drive_profile(profile_output_path, report)
        return report
    finally:
        simulation_app.close()


def render_calibration_report(
    output_path: str | Path, report: object
) -> Path:
    """Render longitudinal, turning, and braking summaries to one PNG."""

    from PIL import Image, ImageDraw

    if not isinstance(report, dict) or report.get("probe") != "coupled_drive_dynamics":
        raise ValueError("report must be a coupled_drive_dynamics object")
    cases = report.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ValueError("calibration report requires a non-empty cases list")

    width, height = 1200, 760
    canvas = Image.new("RGB", (width, height), (249, 250, 252))
    draw = ImageDraw.Draw(canvas)
    draw.text((48, 24), "Ball robot coupled drive-dynamics calibration", fill=(30, 35, 42))
    draw.text(
        (48, 48),
        "main response | steering curvature | braking distance",
        fill=(85, 90, 98),
    )

    panels = (
        (48, 100, 376, 650),
        (436, 100, 764, 650),
        (824, 100, 1152, 650),
    )
    for bounds in panels:
        draw.rectangle(bounds, fill=(255, 255, 255), outline=(190, 196, 202), width=2)

    def finite_points(
        records: Sequence[dict[str, object]], x_key: str, y_key: str
    ) -> list[tuple[float, float]]:
        points: list[tuple[float, float]] = []
        for record in records:
            try:
                x_value = float(record[x_key])
                y_value = float(record[y_key])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(x_value) and math.isfinite(y_value):
                points.append((x_value, y_value))
        return points

    def scatter(
        bounds: tuple[int, int, int, int],
        points: Sequence[tuple[float, float]],
        *,
        title: str,
        x_label: str,
        y_label: str,
        color: tuple[int, int, int],
    ) -> None:
        left, top, right, bottom = bounds
        draw.text((left + 16, top + 14), title, fill=(35, 40, 48))
        plot = (left + 52, top + 54, right - 18, bottom - 54)
        draw.rectangle(plot, outline=(210, 214, 220), width=1)
        draw.text((left + 56, bottom - 36), x_label, fill=(80, 85, 92))
        draw.text((left + 8, top + 42), y_label, fill=(80, 85, 92))
        if not points:
            draw.text((left + 75, top + 250), "no finite records", fill=(150, 80, 80))
            return
        values = np.asarray(points, dtype=np.float64)
        minimum = values.min(axis=0)
        maximum = values.max(axis=0)
        span = maximum - minimum
        span[span < 1e-9] = 1.0
        padding = span * 0.08
        minimum -= padding
        maximum += padding
        span = maximum - minimum
        for x_value, y_value in points:
            px = plot[0] + (x_value - minimum[0]) / span[0] * (plot[2] - plot[0])
            py = plot[3] - (y_value - minimum[1]) / span[1] * (plot[3] - plot[1])
            draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=color)
        draw.text(
            (plot[0], plot[3] + 8),
            f"x [{minimum[0]:.2f}, {maximum[0]:.2f}]",
            fill=(110, 115, 122),
        )
        draw.text(
            (plot[0], plot[1] - 20),
            f"y [{minimum[1]:.2f}, {maximum[1]:.2f}]",
            fill=(110, 115, 122),
        )

    typed_cases = [case for case in cases if isinstance(case, dict)]
    straight = [case for case in typed_cases if case.get("mode") == "straight"]
    turning = [case for case in typed_cases if case.get("mode") == "steady_turn"]
    braking = [case for case in typed_cases if case.get("mode") == "braking"]
    braking_points: list[tuple[float, float]] = []
    for case in braking:
        record = case.get("braking")
        if not isinstance(record, dict):
            continue
        braking_points.extend(
            finite_points(
                [record], "initial_braking_speed_m_s", "stopping_distance_m"
            )
        )
    scatter(
        panels[0],
        finite_points(
            straight, "steady_main_joint_velocity_rad_s", "steady_forward_speed_m_s"
        ),
        title="Longitudinal response",
        x_label="main joint rad/s",
        y_label="speed m/s",
        color=(34, 112, 204),
    )
    turning_points = [
        (math.degrees(float(case["auxiliary_command_rad"])), float(case["steady_curvature_1_m"]))
        for case in turning
        if case.get("steady_curvature_1_m") is not None
    ]
    scatter(
        panels[1],
        turning_points,
        title="Steering response",
        x_label="auxiliary deg",
        y_label="curvature 1/m",
        color=(224, 112, 34),
    )
    scatter(
        panels[2],
        braking_points,
        title="Braking response",
        x_label="initial speed m/s",
        y_label="distance m",
        color=(32, 150, 86),
    )

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp")
    canvas.save(temporary, format="PNG")
    temporary.replace(output)
    return output


def _render_report_file(report_path: Path) -> Path:
    with report_path.open(encoding="utf-8") as handle:
        report = json.load(handle)
    return render_calibration_report(
        report_path.with_name("drive_dynamics_calibration.png"), report
    )


def _render_report_in_clean_process(report_path: str | Path) -> Path:
    path = Path(report_path)
    completed = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), "--render-report", str(path)],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"clean calibration renderer failed: {detail}")
    return path.with_name("drive_dynamics_calibration.png")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate coupled main/auxiliary ball-drive dynamics in Isaac Sim."
    )
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--profile-out", type=Path)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--main-accel-rad-s2",
        type=float,
        default=DEFAULT_MAIN_ACCELERATION_RAD_S2,
    )
    parser.add_argument(
        "--aux-rate-deg-s",
        type=float,
        default=math.degrees(DEFAULT_AUXILIARY_RATE_RAD_S),
    )
    parser.add_argument("--aux-kp", type=float, default=DEFAULT_AUXILIARY_KP)
    parser.add_argument("--aux-kd", type=float, default=DEFAULT_AUXILIARY_KD)
    parser.add_argument("--render-report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.render_report is None and args.out is None:
        parser.error("--out is required")
    if args.render_report is not None and (
        args.out is not None or args.profile_out is not None or args.gui
    ):
        parser.error(
            "--render-report cannot be combined with --out, --profile-out, or --gui"
        )
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.render_report is not None:
        _render_report_file(args.render_report)
        return 0
    report = run_calibration(
        args.contract,
        args.out,
        gui=args.gui,
        main_acceleration_rad_s2=args.main_accel_rad_s2,
        auxiliary_kp=args.aux_kp,
        auxiliary_kd=args.aux_kd,
        auxiliary_rate_rad_s=math.radians(args.aux_rate_deg_s),
        profile_output_path=args.profile_out,
    )
    print(
        f"Calibration complete: {report['case_count']} cases -> "
        f"{args.out / 'drive_dynamics_calibration.json'}",
        flush=True,
    )
    print(args.out / "drive_dynamics_calibration.png", flush=True)
    if args.profile_out is not None:
        print(args.profile_out, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
