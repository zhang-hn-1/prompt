"""Coupled main/auxiliary drive-dynamics calibration for the ball robot.

Isaac Sim is imported only by the runtime entry point so that experiment
construction and trace analysis remain available to CPU-only tests and tools.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import traceback
from typing import Sequence

import numpy as np

from action_smoothing import (
    ActionSmoother,
    DEFAULT_AUXILIARY_KD,
    DEFAULT_AUXILIARY_KP,
    DEFAULT_AUXILIARY_RATE_RAD_S,
    DEFAULT_MAIN_ACCELERATION_RAD_S2,
    DEFAULT_MAIN_DERIVATIVE_GAIN,
    slew_limited_target,
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
    dataset_role: str = "legacy"
    repeat_of: str | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class TrialStage:
    phase: str
    duration_s: float
    main_target_rad_s: float
    auxiliary_target_rad: float


ROUND4_SCHEMA = "round4_drive_calibration_v1"
ROUND4_DATASET_ROLES = frozenset(
    ("fit_core", "repeat_validation", "heldout_validation")
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_round4_manifest(
    path: str | Path, contract: object
) -> tuple[dict[str, object], tuple[CalibrationCase, ...]]:
    """Load the frozen Round 4 matrix without importing or starting Isaac."""

    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"could not load Round 4 manifest: {source}") from error
    if not isinstance(payload, dict) or payload.get("schema_version") != ROUND4_SCHEMA:
        raise ValueError(f"Round 4 manifest must use schema {ROUND4_SCHEMA}")
    if payload.get("action_protocol_id") != EXPECTED_PROTOCOL_ID:
        raise ValueError(f"Round 4 manifest must use {EXPECTED_PROTOCOL_ID}")
    if getattr(contract, "action_protocol_id", None) != EXPECTED_PROTOCOL_ID:
        raise ValueError(f"calibration requires {EXPECTED_PROTOCOL_ID}")
    if getattr(contract, "action_names", None) != EXPECTED_ACTION_NAMES:
        raise ValueError("calibration requires the two-field ball action protocol")
    minimum = np.asarray(getattr(contract, "action_minimum", ()), dtype=np.float64)
    maximum = np.asarray(getattr(contract, "action_maximum", ()), dtype=np.float64)
    if minimum.shape != (2,) or maximum.shape != (2,) or not np.isfinite(
        [minimum, maximum]
    ).all():
        raise ValueError("action bounds must be two finite vectors")

    controller = payload.get("controller")
    if not isinstance(controller, Mapping):
        raise ValueError("Round 4 controller metadata is required")
    expected_controller = {
        "main_acceleration_limit_rad_s2": DEFAULT_MAIN_ACCELERATION_RAD_S2,
        "main_derivative_gain": DEFAULT_MAIN_DERIVATIVE_GAIN,
        "auxiliary_kp": DEFAULT_AUXILIARY_KP,
        "auxiliary_kd": DEFAULT_AUXILIARY_KD,
        "auxiliary_target_rate_limit_rad_s": DEFAULT_AUXILIARY_RATE_RAD_S,
        "auxiliary_velocity_limit_rad_s": 0.30,
    }
    for name, expected in expected_controller.items():
        actual = _finite_float(controller.get(name), name=f"controller.{name}")
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"Round 4 controller.{name} does not match frozen value")

    raw_cases = payload.get("cases")
    if not isinstance(raw_cases, list) or not raw_cases:
        raise ValueError("Round 4 manifest cases must be a non-empty list")
    cases: list[CalibrationCase] = []
    seen_ids: set[str] = set()
    role_counts = {role: 0 for role in ROUND4_DATASET_ROLES}
    for row in raw_cases:
        if not isinstance(row, Mapping):
            raise ValueError("each Round 4 case must be an object")
        case_id = row.get("case_id")
        role = row.get("dataset_role")
        mode = row.get("mode")
        if not isinstance(case_id, str) or not case_id or case_id in seen_ids:
            raise ValueError("Round 4 case_id values must be unique non-empty strings")
        if role not in ROUND4_DATASET_ROLES:
            raise ValueError("Round 4 dataset_role is invalid")
        if mode not in {"straight", "steering_step"}:
            raise ValueError("Round 4 mode must be straight or steering_step")
        main = _finite_float(row.get("main_command_rad_s"), name="main_command_rad_s")
        auxiliary = _finite_float(
            row.get("auxiliary_command_rad"), name="auxiliary_command_rad"
        )
        if main <= 0.0 or main < minimum[0] or main > maximum[0]:
            raise ValueError("Round 4 main command lies outside the action protocol")
        if auxiliary < minimum[1] - 1e-9 or auxiliary > maximum[1] + 1e-9:
            raise ValueError("Round 4 auxiliary command lies outside the action protocol")
        if (mode == "straight") != math.isclose(auxiliary, 0.0, abs_tol=1e-12):
            raise ValueError("Round 4 straight cases require zero auxiliary command")
        repeat_of = row.get("repeat_of")
        if repeat_of is not None and not isinstance(repeat_of, str):
            raise ValueError("repeat_of must be a string when present")
        seen_ids.add(case_id)
        role_counts[str(role)] += 1
        cases.append(
            CalibrationCase(
                case_id=case_id,
                mode=str(mode),
                main_command_rad_s=main,
                auxiliary_command_rad=auxiliary,
                dataset_role=str(role),
                repeat_of=repeat_of,
            )
        )
    if role_counts["fit_core"] > 20:
        raise ValueError("Round 4 permits at most 20 core cases")
    if role_counts["repeat_validation"] + role_counts["heldout_validation"] > 8:
        raise ValueError("Round 4 permits at most 8 validation cases")
    fit_ids = {case.case_id for case in cases if case.dataset_role == "fit_core"}
    for case in cases:
        if case.dataset_role == "repeat_validation" and case.repeat_of not in fit_ids:
            raise ValueError("repeat_validation cases must name a fit_core repeat_of")
        if case.dataset_role != "repeat_validation" and case.repeat_of is not None:
            raise ValueError("repeat_of is only valid for repeat_validation cases")
    return payload, tuple(cases)


def estimate_geometric_curvature(
    position_xy_m: Sequence[Sequence[float]],
    *,
    minimum_path_length_m: float,
) -> dict[str, object]:
    """Estimate signed curvature from XY motion tangents versus travelled arc length."""

    positions = _finite_vector(position_xy_m, name="position_xy_m", columns=2)
    minimum_length = _finite_float(
        minimum_path_length_m, name="minimum_path_length_m"
    )
    if minimum_length <= 0.0:
        raise ValueError("minimum_path_length_m must be positive")
    deltas = np.diff(positions, axis=0)
    lengths = np.linalg.norm(deltas, axis=1)
    moving = lengths > 1e-6
    path_length = float(lengths.sum())
    result: dict[str, object] = {
        "source": "xy_motion_tangent_vs_arc_length",
        "path_length_m": path_length,
        "curvature_1_m": None,
        "fit_residual_rad_rms": None,
        "valid": False,
        "invalid_reason": None,
    }
    if path_length < minimum_length or np.count_nonzero(moving) < 3:
        result["invalid_reason"] = "insufficient_path_length"
        return result
    segment_lengths = lengths[moving]
    segment_deltas = deltas[moving]
    heading = np.unwrap(np.arctan2(segment_deltas[:, 1], segment_deltas[:, 0]))
    arc_end = np.cumsum(segment_lengths)
    arc_mid = arc_end - 0.5 * segment_lengths
    design = np.column_stack((arc_mid, np.ones_like(arc_mid)))
    coefficients, *_ = np.linalg.lstsq(design, heading, rcond=None)
    fitted = design @ coefficients
    curvature = float(coefficients[0])
    residual = float(np.sqrt(np.mean(np.square(heading - fitted))))
    if not np.isfinite([curvature, residual]).all():
        result["invalid_reason"] = "nonfinite_geometry_fit"
        return result
    result.update(
        {
            "curvature_1_m": curvature,
            "fit_residual_rad_rms": residual,
            "valid": True,
            "invalid_reason": None,
        }
    )
    return result


def analyze_round4_response(
    *,
    case: CalibrationCase,
    traces: Mapping[str, Sequence[object]],
    stability: Mapping[str, object],
    steady_window_s: float,
) -> dict[str, object]:
    """Apply the predeclared Round 4 steady-state and validity rules."""

    required = (
        "time_s",
        "phase",
        "requested_main_rad_s",
        "final_main_rad_s",
        "main_joint_velocity_rad_s",
        "requested_auxiliary_rad",
        "final_auxiliary_rad",
        "auxiliary_joint_rad",
        "auxiliary_joint_velocity_rad_s",
        "position_xy_m",
        "linear_velocity_world_xy_m_s",
        "body_yaw_rate_rad_s",
    )
    missing = [name for name in required if name not in traces]
    if missing:
        raise ValueError(f"Round 4 trace is missing: {', '.join(missing)}")
    count = _validate_equal_lengths(**{name: traces[name] for name in required})
    time = _finite_vector(traces["time_s"], name="time_s")
    if np.any(np.diff(time) <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    phases = np.asarray(traces["phase"], dtype=str)
    target_phase = "steer" if case.mode == "steering_step" else "drive"
    phase_indices = np.flatnonzero(phases == target_phase)
    if not phase_indices.size:
        raise ValueError(f"Round 4 trace has no {target_phase} phase")
    phase_end_time = float(time[int(phase_indices[-1])])
    window_start = phase_end_time - _finite_float(
        steady_window_s, name="steady_window_s"
    )
    indices = phase_indices[time[phase_indices] >= window_start - 1e-12]
    if indices.size < 2:
        raise ValueError("steady window contains fewer than two samples")

    auxiliary_actual = _finite_vector(
        traces["auxiliary_joint_rad"], name="auxiliary_joint_rad"
    )
    auxiliary_velocity = _finite_vector(
        traces["auxiliary_joint_velocity_rad_s"],
        name="auxiliary_joint_velocity_rad_s",
    )
    main_actual = _finite_vector(
        traces["main_joint_velocity_rad_s"], name="main_joint_velocity_rad_s"
    )
    velocity = _finite_vector(
        traces["linear_velocity_world_xy_m_s"],
        name="linear_velocity_world_xy_m_s",
        columns=2,
    )
    speed = np.linalg.norm(velocity, axis=1)
    yaw_rate = _finite_vector(
        traces["body_yaw_rate_rad_s"], name="body_yaw_rate_rad_s"
    )
    position = _finite_vector(
        traces["position_xy_m"], name="position_xy_m", columns=2
    )
    target_error = np.abs(auxiliary_actual[indices] - case.auxiliary_command_rad)
    median_aux_error = float(np.median(target_error))
    p95_aux_velocity = float(np.percentile(np.abs(auxiliary_velocity[indices]), 95.0))
    median_speed = float(np.median(speed[indices]))
    speed_cv = float(np.std(speed[indices]) / max(median_speed, 1e-12))
    relative_time = time[indices] - time[indices[0]]
    yaw_trend = float(
        np.polyfit(relative_time, yaw_rate[indices], 1)[0]
        if indices.size >= 3
        else 0.0
    )
    geometry = estimate_geometric_curvature(
        position[indices],
        minimum_path_length_m=_finite_float(
            stability.get("minimum_geometric_path_length_m"),
            name="minimum_geometric_path_length_m",
        ),
    )
    reasons: list[str] = []
    if median_aux_error > _finite_float(
        stability.get("maximum_auxiliary_median_error_rad"),
        name="maximum_auxiliary_median_error_rad",
    ):
        reasons.append("auxiliary_target_not_reached")
    if p95_aux_velocity > _finite_float(
        stability.get("maximum_auxiliary_p95_abs_velocity_rad_s"),
        name="maximum_auxiliary_p95_abs_velocity_rad_s",
    ):
        reasons.append("auxiliary_still_moving")
    if median_speed < _finite_float(
        stability.get("minimum_planar_speed_m_s"), name="minimum_planar_speed_m_s"
    ):
        reasons.append("low_planar_speed")
    if speed_cv > _finite_float(
        stability.get("maximum_speed_coefficient_of_variation"),
        name="maximum_speed_coefficient_of_variation",
    ):
        reasons.append("speed_not_steady")
    if abs(yaw_trend) > _finite_float(
        stability.get("maximum_abs_yaw_rate_trend_rad_s2"),
        name="maximum_abs_yaw_rate_trend_rad_s2",
    ):
        reasons.append("yaw_rate_not_steady")
    if not geometry["valid"]:
        reasons.append(str(geometry["invalid_reason"]))
    valid = not reasons
    curvature = geometry["curvature_1_m"] if valid else None
    return {
        "case_id": case.case_id,
        "dataset_role": case.dataset_role,
        "steady_window": {
            "phase": target_phase,
            "start_time_s": float(time[int(indices[0])]),
            "end_time_s": float(time[int(indices[-1])]),
            "sample_count": int(indices.size),
        },
        "steady_state_valid": valid,
        "invalid_reasons": reasons,
        "steady_main_joint_velocity_rad_s": float(np.median(main_actual[indices])),
        "steady_auxiliary_joint_rad": float(np.median(auxiliary_actual[indices])),
        "steady_auxiliary_error_rad": median_aux_error,
        "steady_auxiliary_p95_abs_velocity_rad_s": p95_aux_velocity,
        "steady_speed_m_s": median_speed,
        "steady_speed_coefficient_of_variation": speed_cv,
        "steady_body_yaw_rate_rad_s": float(np.median(yaw_rate[indices])),
        "steady_body_yaw_rate_trend_rad_s2": yaw_trend,
        "steady_curvature_1_m": curvature,
        "geometric_curvature": geometry,
        "trace_sample_count": int(count),
    }


def evaluate_candidate_on_heldout(
    rows: Sequence[Mapping[str, object]], gates: Mapping[str, object]
) -> dict[str, object]:
    """Evaluate a mapping candidate only on valid predeclared held-out rows."""

    heldout = [
        row
        for row in rows
        if row.get("dataset_role") == "heldout_validation"
        and row.get("steady_state_valid") is True
    ]
    metrics: dict[str, float] = {}
    for prefix in ("v1", "candidate"):
        for output in ("speed", "curvature"):
            values = [
                abs(_finite_float(row.get(f"{prefix}_{output}_error_m_s" if output == "speed" else f"{prefix}_{output}_error_1_m"), name=f"{prefix}_{output}_error"))
                for row in heldout
            ]
            metrics[f"{prefix}_{output}_mae"] = (
                float(np.mean(values)) if values else math.inf
            )
    curvature_improvement = (
        (metrics["v1_curvature_mae"] - metrics["candidate_curvature_mae"])
        / metrics["v1_curvature_mae"]
        if math.isfinite(metrics["v1_curvature_mae"])
        and metrics["v1_curvature_mae"] > 0.0
        else -math.inf
    )
    speed_regression = (
        (metrics["candidate_speed_mae"] - metrics["v1_speed_mae"])
        / metrics["v1_speed_mae"]
        if math.isfinite(metrics["v1_speed_mae"])
        and metrics["v1_speed_mae"] > 0.0
        else math.inf
    )
    minimum_count = int(gates.get("minimum_valid_heldout_cases", 1))
    passed = (
        len(heldout) >= minimum_count
        and curvature_improvement
        >= _finite_float(
            gates.get("minimum_curvature_mae_improvement_fraction"),
            name="minimum_curvature_mae_improvement_fraction",
        )
        and speed_regression
        <= _finite_float(
            gates.get("maximum_speed_mae_regression_fraction"),
            name="maximum_speed_mae_regression_fraction",
        )
    )
    return {
        "heldout_case_count": len(heldout),
        **metrics,
        "curvature_mae_improvement_fraction": curvature_improvement,
        "speed_mae_regression_fraction": speed_regression,
        "passed": bool(passed),
    }


def _case_number(value: float) -> str:
    sign = "neg" if value < 0.0 else "pos"
    magnitude = f"{abs(value):.1f}".replace(".", "p")
    return f"{sign}{magnitude}"


def _parse_positive_optional_velocity_limit(value: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise argparse.ArgumentTypeError(
            "auxiliary velocity limit must be finite and positive"
        ) from error
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError(
            "auxiliary velocity limit must be finite and positive"
        )
    return result


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
    auxiliary_velocity_limit_rad_s: float | None = None,
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
    if auxiliary_velocity_limit_rad_s is not None and (
        not math.isfinite(float(auxiliary_velocity_limit_rad_s))
        or float(auxiliary_velocity_limit_rad_s) <= 0.0
    ):
        raise ValueError(
            "auxiliary_velocity_limit_rad_s must be positive and finite or None"
        )
    main_ramp_s = abs(case.main_command_rad_s) / main_acceleration
    auxiliary_ramp_s = abs(case.auxiliary_command_rad) / auxiliary_rate
    auxiliary_settle_s = (
        0.0
        if auxiliary_velocity_limit_rad_s is None
        else abs(case.auxiliary_command_rad) / float(auxiliary_velocity_limit_rad_s)
    )
    limited_settle_margin_s = 2.0 if auxiliary_velocity_limit_rad_s is not None else 0.0
    if case.mode in {"straight", "steady_turn"}:
        return (
            TrialStage(
                phase="drive",
                duration_s=(
                    max(main_ramp_s, auxiliary_ramp_s, auxiliary_settle_s)
                    + 2.0
                    + limited_settle_margin_s
                ),
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
                duration_s=max(auxiliary_ramp_s, auxiliary_settle_s) + 1.5 + limited_settle_margin_s,
                main_target_rad_s=case.main_command_rad_s,
                auxiliary_target_rad=case.auxiliary_command_rad,
            ),
            TrialStage(
                phase="release",
                duration_s=max(auxiliary_ramp_s, auxiliary_settle_s) + 1.0 + limited_settle_margin_s,
                main_target_rad_s=case.main_command_rad_s,
                auxiliary_target_rad=0.0,
            ),
        )
    raise ValueError(f"unsupported calibration mode: {case.mode!r}")


def round4_trial_stages(
    case: CalibrationCase,
    *,
    main_acceleration_rad_s2: float,
    steady_window_s: float,
    maximum_steer_hold_s: float,
    maximum_release_s: float,
) -> tuple[TrialStage, ...]:
    """Build the predeclared baseline/hold/release sequence for Round 4."""

    main_ramp_s = abs(case.main_command_rad_s) / float(main_acceleration_rad_s2)
    if case.mode == "straight":
        return (
            TrialStage(
                "drive",
                main_ramp_s + max(2.0, float(steady_window_s)),
                case.main_command_rad_s,
                0.0,
            ),
        )
    if case.mode != "steering_step":
        raise ValueError("Round 4 supports only straight and steering_step cases")
    return (
        TrialStage("baseline", main_ramp_s + 2.0, case.main_command_rad_s, 0.0),
        TrialStage(
            "steer",
            float(maximum_steer_hold_s),
            case.main_command_rad_s,
            case.auxiliary_command_rad,
        ),
        TrialStage(
            "release",
            float(maximum_release_s),
            case.main_command_rad_s,
            0.0,
        ),
    )


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


def _strictly_decreasing(values: Sequence[float]) -> np.ndarray:
    """Project measured curvature samples onto a decreasing response curve."""

    samples = np.asarray(values, dtype=np.float64)
    if samples.ndim != 1 or samples.size == 0 or not np.isfinite(samples).all():
        raise ValueError("curvature samples must be a finite non-empty vector")
    blocks: list[list[float]] = []
    for sample in samples:
        blocks.append([float(sample), 1.0])
        while len(blocks) >= 2 and blocks[-2][0] < blocks[-1][0]:
            total = blocks[-2][0] * blocks[-2][1] + blocks[-1][0] * blocks[-1][1]
            count = blocks[-2][1] + blocks[-1][1]
            blocks[-2:] = [[total / count, count]]
    projected = np.concatenate(
        [np.full(int(count), mean, dtype=np.float64) for mean, count in blocks]
    )
    for index in range(1, projected.size):
        if projected[index] >= projected[index - 1]:
            projected[index] = projected[index - 1] - max(
                1e-9, abs(projected[index - 1]) * 1e-9
            )
    return projected


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
    auxiliary_velocity_limit = controller.get("auxiliary_velocity_limit_rad_s")
    if auxiliary_velocity_limit is not None:
        auxiliary_velocity_limit = _finite_float(
            auxiliary_velocity_limit,
            name="controller.auxiliary_velocity_limit_rad_s",
        )
        if auxiliary_velocity_limit <= 0.0:
            raise ValueError(
                "controller.auxiliary_velocity_limit_rad_s must be positive"
            )
        normalized_controller["auxiliary_velocity_limit_rad_s"] = (
            auxiliary_velocity_limit
        )

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
    for main in expected_main:
        ordered_keys = sorted(
            key for key in steering_by_key if key[0] == main
        )
        projected = _strictly_decreasing(
            [float(steering_by_key[key]["curvature_1_m"]) for key in ordered_keys]
        )
        for key, curvature in zip(ordered_keys, projected):
            steering_by_key[key]["curvature_1_m"] = float(curvature)
    if set(braking_by_main) != expected_main:
        raise ValueError("report must contain the complete signed braking matrix")

    return {
        "schema_version": (
            "drive_dynamics_v2"
            if auxiliary_velocity_limit is not None
            else "drive_dynamics_v1"
        ),
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
    auxiliary_velocity_limit_rad_s: float | None = None,
    profile_output_path: Path | None = None,
    manifest_path: Path | None = None,
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
    if auxiliary_velocity_limit_rad_s is not None and (
        not math.isfinite(float(auxiliary_velocity_limit_rad_s))
        or float(auxiliary_velocity_limit_rad_s) <= 0.0
    ):
        raise ValueError(
            "auxiliary_velocity_limit_rad_s must be positive and finite or None"
        )

    from collection_schema import load_contract

    contract = load_contract(contract_path)
    round4_manifest: dict[str, object] | None = None
    if manifest_path is None:
        cases = calibration_cases_from_contract(contract)
    else:
        round4_manifest, cases = load_round4_manifest(manifest_path, contract)
        frozen_controller = round4_manifest["controller"]
        assert isinstance(frozen_controller, Mapping)
        actual_controller = {
            "main_acceleration_limit_rad_s2": main_acceleration_rad_s2,
            "main_derivative_gain": DEFAULT_MAIN_DERIVATIVE_GAIN,
            "auxiliary_kp": auxiliary_kp,
            "auxiliary_kd": auxiliary_kd,
            "auxiliary_target_rate_limit_rad_s": auxiliary_rate_rad_s,
            "auxiliary_velocity_limit_rad_s": auxiliary_velocity_limit_rad_s,
        }
        for name, actual in actual_controller.items():
            expected = _finite_float(
                frozen_controller.get(name), name=f"controller.{name}"
            )
            if actual is None or not math.isclose(
                float(actual), expected, rel_tol=0.0, abs_tol=1e-9
            ):
                raise ValueError(
                    f"effective {name} does not match the frozen Round 4 manifest"
                )
        profile_metadata = round4_manifest.get("drive_profile")
        if not isinstance(profile_metadata, Mapping):
            raise ValueError("Round 4 drive_profile metadata is required")
        profile_path = Path(str(profile_metadata.get("path"))).resolve()
        expected_profile_hash = str(profile_metadata.get("sha256"))
        if not profile_path.is_file():
            raise FileNotFoundError(f"frozen drive profile does not exist: {profile_path}")
        actual_profile_hash = _sha256(profile_path)
        if actual_profile_hash != expected_profile_hash:
            raise RuntimeError(
                "frozen drive profile SHA-256 mismatch: "
                f"expected {expected_profile_hash}, got {actual_profile_hash}"
            )
        output_dir.mkdir(parents=True, exist_ok=True)
        resolved_manifest = dict(round4_manifest)
        resolved_manifest["resolved_identity"] = {
            "manifest_path": str(Path(manifest_path).resolve()),
            "manifest_sha256": _sha256(Path(manifest_path)),
            "contract_path": str(Path(contract_path).resolve()),
            "contract_sha256": _sha256(Path(contract_path)),
            "drive_profile_path": str(profile_path),
            "drive_profile_sha256": actual_profile_hash,
            "robot_usd_path": str(ROBOT_USD.resolve()),
            "robot_usd_sha256": _sha256(ROBOT_USD),
            "ground": "omni.isaac.core.World.scene.add_default_ground_plane",
        }
        _atomic_json(output_dir / "resolved_calibration_manifest.json", resolved_manifest)

    # Keep Isaac imports below all frozen-config validation and manifest writes.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {"headless": not gui, "width": 1280, "height": 720}
    )
    try:
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction

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
                max_auxiliary_velocity_rad_s=auxiliary_velocity_limit_rad_s,
            )
            smoother.reset()
            traces: dict[str, list[object]] = {
                "time_s": [],
                "phase": [],
                "requested_main_rad_s": [],
                "smoothed_main_rad_s": [],
                "final_main_rad_s": [],
                "main_target_rad_s": [],
                "requested_auxiliary_rad": [],
                "smoothed_auxiliary_rad": [],
                "final_auxiliary_rad": [],
                "auxiliary_target_rad": [],
                "main_joint_velocity_rad_s": [],
                "auxiliary_joint_rad": [],
                "auxiliary_joint_velocity_rad_s": [],
                "auxiliary_velocity_supervisor_active": [],
                "auxiliary_command_at_angle_limit": [],
                "auxiliary_tracking_error_rad": [],
                "auxiliary_tracking_deficient": [],
                "yaw_rad": [],
                "navigation_heading_rad": [],
                "root_quaternion_world_wxyz": [],
                "position_xy_m": [],
                "linear_velocity_world_xy_m_s": [],
                "linear_velocity_world_m_s": [],
                "angular_velocity_world_rad_s": [],
                "body_yaw_rate_rad_s": [],
                "physical_contact": [],
                "formal_collision": [],
                "reset_flag": [],
                "end_flag": [],
            }
            elapsed_steps = 0
            if round4_manifest is None:
                stages = trial_stages(
                    case,
                    main_acceleration_rad_s2=main_acceleration_rad_s2,
                    auxiliary_rate_rad_s=auxiliary_rate_rad_s,
                    auxiliary_velocity_limit_rad_s=auxiliary_velocity_limit_rad_s,
                )
                procedure: Mapping[str, object] = {}
                stability: Mapping[str, object] = {}
            else:
                procedure_value = round4_manifest.get("procedure")
                stability_value = round4_manifest.get("stability")
                if not isinstance(procedure_value, Mapping) or not isinstance(
                    stability_value, Mapping
                ):
                    raise ValueError("Round 4 procedure and stability metadata are required")
                procedure = procedure_value
                stability = stability_value
                stages = round4_trial_stages(
                    case,
                    main_acceleration_rad_s2=main_acceleration_rad_s2,
                    steady_window_s=_finite_float(
                        procedure.get("steady_window_s"), name="steady_window_s"
                    ),
                    maximum_steer_hold_s=_finite_float(
                        procedure.get("maximum_steer_hold_s"),
                        name="maximum_steer_hold_s",
                    ),
                    maximum_release_s=_finite_float(
                        procedure.get("maximum_release_s"), name="maximum_release_s"
                    ),
                )
            end_reason = "completed_fixed_sequence"
            stop_case = False
            for stage in stages:
                stage_steps = max(1, int(round(stage.duration_s * physics_hz)))
                for _ in range(stage_steps):
                    if gui and not simulation_app.is_running():
                        raise RuntimeError(
                            "Isaac GUI was closed before drive calibration completed"
                        )
                    measured_positions = np.asarray(
                        robot.get_joint_positions(), dtype=np.float64
                    )
                    measured_velocities = np.asarray(
                        robot.get_joint_velocities(), dtype=np.float64
                    )
                    if (
                        measured_positions.shape[0] < 2
                        or measured_velocities.shape[0] < 2
                    ):
                        raise RuntimeError(
                            "robot returned incomplete joint state during calibration"
                        )
                    requested = np.asarray(
                        [stage.main_target_rad_s, stage.auxiliary_target_rad],
                        dtype=np.float64,
                    )
                    clipped = np.clip(requested, action_minimum, action_maximum)
                    current = smoother.current_action
                    smoothed = np.asarray(
                        [
                            slew_limited_target(
                                current[0],
                                clipped[0],
                                main_acceleration_rad_s2,
                                physics_dt,
                            ),
                            slew_limited_target(
                                current[1],
                                clipped[1],
                                auxiliary_rate_rad_s,
                                physics_dt,
                            ),
                        ],
                        dtype=np.float64,
                    )
                    supervisor_active = bool(
                        auxiliary_velocity_limit_rad_s is not None
                        and abs(float(measured_velocities[1]))
                        > float(auxiliary_velocity_limit_rad_s)
                    )
                    if auxiliary_velocity_limit_rad_s is None:
                        applied = smoother.step(
                            requested,
                            action_minimum,
                            action_maximum,
                        )
                    else:
                        applied = smoother.step(
                            requested,
                            action_minimum,
                            action_maximum,
                            auxiliary_position_rad=float(measured_positions[1]),
                            auxiliary_velocity_rad_s=float(measured_velocities[1]),
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
                    angular_velocity = np.asarray(
                        robot.get_angular_velocity(), dtype=np.float64
                    )
                    if (
                        joint_positions.shape[0] < 2
                        or joint_velocities.shape[0] < 2
                        or linear_velocity.shape != (3,)
                        or angular_velocity.shape != (3,)
                    ):
                        raise RuntimeError("robot returned an incomplete state vector")
                    state_values = np.concatenate(
                        (
                            np.asarray(position, dtype=np.float64),
                            np.asarray(quaternion, dtype=np.float64),
                            joint_positions[:2],
                            joint_velocities[:2],
                            linear_velocity,
                            angular_velocity,
                        )
                    )
                    if not np.isfinite(state_values).all():
                        end_reason = "nonfinite_state"
                        stop_case = True
                        break
                    navigation_heading = _yaw_from_wxyz(quaternion)
                    tracking_error = float(applied[1] - joint_positions[1])
                    tracking_threshold = (
                        _finite_float(
                            stability.get("maximum_auxiliary_median_error_rad"),
                            name="maximum_auxiliary_median_error_rad",
                        )
                        if round4_manifest is not None
                        else 0.035
                    )
                    traces["time_s"].append(elapsed_steps * physics_dt)
                    traces["phase"].append(stage.phase)
                    traces["requested_main_rad_s"].append(float(requested[0]))
                    traces["smoothed_main_rad_s"].append(float(smoothed[0]))
                    traces["final_main_rad_s"].append(float(applied[0]))
                    traces["main_target_rad_s"].append(float(applied[0]))
                    traces["requested_auxiliary_rad"].append(float(requested[1]))
                    traces["smoothed_auxiliary_rad"].append(float(smoothed[1]))
                    traces["final_auxiliary_rad"].append(float(applied[1]))
                    traces["auxiliary_target_rad"].append(float(applied[1]))
                    traces["main_joint_velocity_rad_s"].append(
                        float(joint_velocities[0])
                    )
                    traces["auxiliary_joint_rad"].append(float(joint_positions[1]))
                    traces["auxiliary_joint_velocity_rad_s"].append(
                        float(joint_velocities[1])
                    )
                    traces["auxiliary_velocity_supervisor_active"].append(
                        supervisor_active
                    )
                    traces["auxiliary_command_at_angle_limit"].append(
                        bool(
                            math.isclose(
                                abs(float(requested[1])),
                                max(abs(action_minimum[1]), abs(action_maximum[1])),
                                rel_tol=0.0,
                                abs_tol=1e-7,
                            )
                        )
                    )
                    traces["auxiliary_tracking_error_rad"].append(tracking_error)
                    traces["auxiliary_tracking_deficient"].append(
                        abs(tracking_error) > tracking_threshold
                    )
                    traces["yaw_rad"].append(navigation_heading)
                    traces["navigation_heading_rad"].append(navigation_heading)
                    traces["root_quaternion_world_wxyz"].append(
                        np.asarray(quaternion, dtype=np.float64).tolist()
                    )
                    traces["position_xy_m"].append(
                        [float(position[0]), float(position[1])]
                    )
                    traces["linear_velocity_world_xy_m_s"].append(
                        [float(linear_velocity[0]), float(linear_velocity[1])]
                    )
                    traces["linear_velocity_world_m_s"].append(
                        linear_velocity.astype(float).tolist()
                    )
                    traces["angular_velocity_world_rad_s"].append(
                        angular_velocity.astype(float).tolist()
                    )
                    traces["body_yaw_rate_rad_s"].append(float(angular_velocity[2]))
                    traces["physical_contact"].append(None)
                    traces["formal_collision"].append(None)
                    traces["reset_flag"].append(elapsed_steps == 1)
                    traces["end_flag"].append(False)
                    if round4_manifest is not None:
                        diagnostic_radius = _finite_float(
                            procedure.get("diagnostic_radius_m"),
                            name="diagnostic_radius_m",
                        )
                        if float(
                            np.linalg.norm(
                                np.asarray(position[:2], dtype=np.float64)
                                - ROBOT_START[:2]
                            )
                        ) > diagnostic_radius:
                            end_reason = "diagnostic_radius_exit"
                            stop_case = True
                            break
                if stop_case:
                    break

            if traces["end_flag"]:
                traces["end_flag"][-1] = True

            summary = summarize_motion_trial(
                case_id=case.case_id,
                mode=case.mode,
                time_s=traces["time_s"],
                phase=traces["phase"],
                main_target_rad_s=traces["main_target_rad_s"],
                auxiliary_target_rad=traces["auxiliary_target_rad"],
                main_joint_velocity_rad_s=traces[
                    "main_joint_velocity_rad_s"
                ],
                auxiliary_joint_rad=traces["auxiliary_joint_rad"],
                yaw_rad=traces["yaw_rad"],
                position_xy_m=traces["position_xy_m"],
                linear_velocity_world_xy_m_s=traces[
                    "linear_velocity_world_xy_m_s"
                ],
            )
            # Braking and steering-step trials end at a zero target, so retain
            # the experimental condition explicitly at the top level.
            summary["main_command_rad_s"] = case.main_command_rad_s
            summary["auxiliary_command_rad"] = case.auxiliary_command_rad
            summary["dataset_role"] = case.dataset_role
            summary["repeat_of"] = case.repeat_of
            summary["end_reason"] = end_reason
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
            if round4_manifest is not None:
                analysis = analyze_round4_response(
                    case=case,
                    traces=traces,
                    stability=stability,
                    steady_window_s=_finite_float(
                        procedure.get("steady_window_s"), name="steady_window_s"
                    ),
                )
                summary["round4_analysis"] = analysis
                summary["steady_main_joint_velocity_rad_s"] = analysis[
                    "steady_main_joint_velocity_rad_s"
                ]
                summary["steady_auxiliary_joint_rad"] = analysis[
                    "steady_auxiliary_joint_rad"
                ]
                summary["steady_forward_speed_m_s"] = analysis["steady_speed_m_s"]
                summary["steady_yaw_rate_rad_s"] = analysis[
                    "steady_body_yaw_rate_rad_s"
                ]
                summary["steady_curvature_1_m"] = analysis[
                    "steady_curvature_1_m"
                ]
                curvature_value = analysis["steady_curvature_1_m"]
                summary["signed_turning_radius_m"] = (
                    None
                    if curvature_value is None or abs(float(curvature_value)) <= 1e-6
                    else 1.0 / float(curvature_value)
                )
                window = analysis["steady_window"]
                assert isinstance(window, Mapping)
                for sample_index, sample_time in enumerate(traces["time_s"]):
                    traces.setdefault("steady_window_flag", []).append(
                        window["start_time_s"] <= sample_time <= window["end_time_s"]
                    )
                raw_payload = {
                    "schema_version": "round4_drive_trace_v1",
                    "case": case.to_dict(),
                    "end_reason": end_reason,
                    "contact_semantics": {
                        "physical_contact": "unsupported_in_default_ground_probe",
                        "formal_collision": "unsupported_without_diagnostic_occupancy",
                    },
                    "heading_semantics": "yaw_of_articulation_root_World_BallRobot_from_world_wxyz",
                    "samples": [
                        {name: values[index] for name, values in traces.items()}
                        for index in range(len(traces["time_s"]))
                    ],
                }
                trace_path = output_dir / "traces" / f"{case.case_id}.json"
                _atomic_json(trace_path, raw_payload)
                summary["raw_trace_path"] = str(trace_path.resolve())
                summary.pop("samples", None)
            case_summaries.append(summary)

        controller_report: dict[str, float] = {
            "main_acceleration_limit_rad_s2": main_acceleration_rad_s2,
            "main_derivative_gain": DEFAULT_MAIN_DERIVATIVE_GAIN,
            "auxiliary_kp": auxiliary_kp,
            "auxiliary_kd": auxiliary_kd,
            "auxiliary_target_rate_limit_rad_s": auxiliary_rate_rad_s,
        }
        if auxiliary_velocity_limit_rad_s is not None:
            controller_report["auxiliary_velocity_limit_rad_s"] = float(
                auxiliary_velocity_limit_rad_s
            )
        report: dict[str, object] = {
            "schema_version": "1.0",
            "probe": "coupled_drive_dynamics",
            "measurement_timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "action_protocol_id": contract.action_protocol_id,
            "physics_hz": physics_hz,
            "settle_steps": SETTLE_STEPS,
            "case_count": len(case_summaries),
            "controller": controller_report,
            "cases": case_summaries,
        }
        if manifest_path is not None:
            report["round4_manifest_path"] = str(Path(manifest_path).resolve())
            report["round4_manifest_sha256"] = _sha256(Path(manifest_path))
            report["raw_trace_directory"] = str((output_dir / "traces").resolve())
        output_dir.mkdir(parents=True, exist_ok=True)
        report_path = output_dir / "drive_dynamics_calibration.json"
        _atomic_json(report_path, report)
        _render_report_in_clean_process(report_path)
        if profile_output_path is not None:
            write_compact_drive_profile(profile_output_path, report)
        return report
    except BaseException as error:
        output_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "schema_version": "drive_calibration_failure_v1",
            "failed_utc": datetime.now(timezone.utc).isoformat(),
            "error_type": type(error).__name__,
            "error": str(error),
            "traceback": traceback.format_exc(),
        }
        _atomic_json(output_dir / "failure.json", failure)
        print(failure["traceback"], file=sys.stderr, flush=True)
        raise
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
    parser.add_argument(
        "--manifest",
        type=Path,
        help="optional predeclared Round 4 case/acceptance manifest",
    )
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
    parser.add_argument(
        "--auxiliary-velocity-limit-rad-s",
        type=_parse_positive_optional_velocity_limit,
        default=None,
        help="optional measured auxiliary joint velocity envelope for v2 calibration",
    )
    parser.add_argument("--aux-kp", type=float, default=DEFAULT_AUXILIARY_KP)
    parser.add_argument("--aux-kd", type=float, default=DEFAULT_AUXILIARY_KD)
    parser.add_argument("--render-report", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.render_report is None and args.out is None:
        parser.error("--out is required")
    if args.render_report is not None and (
        args.out is not None
        or args.profile_out is not None
        or args.manifest is not None
        or args.gui
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
        auxiliary_velocity_limit_rad_s=args.auxiliary_velocity_limit_rad_s,
        profile_output_path=args.profile_out,
        manifest_path=args.manifest,
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
