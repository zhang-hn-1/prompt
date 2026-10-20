#!/usr/bin/env python3
"""Run one fixed-command or path-feedback PID isolation diagnostic case.

The module keeps its schedule and statistics CPU-only.  Isaac is imported only
through the existing lazy benchmark runtime when a case is actually executed.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
from typing import Any, Mapping, Sequence

import numpy as np


PHYSICS_HZ = 50
CONTROL_HZ = 10
PHYSICS_STEPS_PER_BLOCK = PHYSICS_HZ // CONTROL_HZ
FIXED_CASE_IDS = (
    "hold_zero",
    "forward_zero",
    "forward_pos10_return",
    "forward_neg10_return",
    "stationary_pos10_return",
)
PATH_CASE_IDS = ("closed_loop_lookahead05", "closed_loop_lookahead10")
CASE_IDS = FIXED_CASE_IDS + PATH_CASE_IDS
DEFAULT_AUXILIARY_KP = 300.0
DEFAULT_AUXILIARY_KD = 60.0
DEFAULT_MAIN_DERIVATIVE_GAIN = 80.0
DEFAULT_AUXILIARY_RATE_RAD_S = math.radians(30.0)
SETTLING_BAND_RAD = math.radians(0.5)
SETTLING_REQUIRED_S = 1.0
CLOCK_TOLERANCE_S = 5e-5


class DiagnosticValidationError(ValueError):
    """Raised when a diagnostic trace or artifact cannot be trusted."""


@dataclass(frozen=True)
class _Phase:
    name: str
    start_block: int
    end_block: int
    main_command_rad_s: float
    auxiliary_angle_rad: float


@dataclass(frozen=True)
class _Case:
    case_id: str
    duration_s: float
    phases: tuple[_Phase, ...] = ()
    lookahead_m: float | None = None

    @property
    def block_count(self) -> int:
        return int(round(self.duration_s * CONTROL_HZ))


def _phase(
    name: str,
    start_s: float,
    end_s: float,
    main_command_rad_s: float,
    auxiliary_angle_rad: float,
) -> _Phase:
    start_block = int(round(start_s * CONTROL_HZ))
    end_block = int(round(end_s * CONTROL_HZ))
    if end_block <= start_block:
        raise ValueError("diagnostic phase must have positive duration")
    return _Phase(
        name=name,
        start_block=start_block,
        end_block=end_block,
        main_command_rad_s=float(main_command_rad_s),
        auxiliary_angle_rad=float(auxiliary_angle_rad),
    )


_ZERO = 0.0
_TEN_DEG_RAD = math.pi / 18.0
_FIXED_CASES = {
    "hold_zero": _Case(
        case_id="hold_zero",
        duration_s=20.0,
        phases=(_phase("hold", 0.0, 20.0, 0.0, 0.0),),
    ),
    "forward_zero": _Case(
        case_id="forward_zero",
        duration_s=30.0,
        phases=(
            _phase("pre_hold", 0.0, 2.0, 0.0, 0.0),
            _phase("forward", 2.0, 22.0, 0.5, 0.0),
            _phase("post_hold", 22.0, 30.0, 0.0, 0.0),
        ),
    ),
    "forward_pos10_return": _Case(
        case_id="forward_pos10_return",
        duration_s=30.0,
        phases=(
            _phase("pre_hold", 0.0, 2.0, 0.0, 0.0),
            _phase("forward_zero", 2.0, 8.0, 0.5, 0.0),
            _phase("forward_pos10", 8.0, 16.0, 0.5, _TEN_DEG_RAD),
            _phase("forward_return", 16.0, 24.0, 0.5, 0.0),
            _phase("post_hold", 24.0, 30.0, 0.0, 0.0),
        ),
    ),
    "forward_neg10_return": _Case(
        case_id="forward_neg10_return",
        duration_s=30.0,
        phases=(
            _phase("pre_hold", 0.0, 2.0, 0.0, 0.0),
            _phase("forward_zero", 2.0, 8.0, 0.5, 0.0),
            _phase("forward_neg10", 8.0, 16.0, 0.5, -_TEN_DEG_RAD),
            _phase("forward_return", 16.0, 24.0, 0.5, 0.0),
            _phase("post_hold", 24.0, 30.0, 0.0, 0.0),
        ),
    ),
    "stationary_pos10_return": _Case(
        case_id="stationary_pos10_return",
        duration_s=30.0,
        phases=(
            _phase("pre_hold", 0.0, 2.0, 0.0, 0.0),
            _phase("stationary_zero", 2.0, 8.0, 0.0, 0.0),
            _phase("stationary_pos10", 8.0, 16.0, 0.0, _TEN_DEG_RAD),
            _phase("stationary_return", 16.0, 24.0, 0.0, 0.0),
            _phase("post_hold", 24.0, 30.0, 0.0, 0.0),
        ),
    ),
}
_CASE_DEFINITIONS = {
    **_FIXED_CASES,
    "closed_loop_lookahead05": _Case(
        case_id="closed_loop_lookahead05", duration_s=20.0, lookahead_m=0.5
    ),
    "closed_loop_lookahead10": _Case(
        case_id="closed_loop_lookahead10", duration_s=20.0, lookahead_m=1.0
    ),
}


def _case(case_id: str) -> _Case:
    if not isinstance(case_id, str) or case_id not in _CASE_DEFINITIONS:
        raise ValueError(f"unknown PID isolation case: {case_id!r}")
    return _CASE_DEFINITIONS[case_id]


def case_duration_s(case_id: str) -> float:
    """Return the fixed diagnostic budget for one of the seven cases."""

    return _case(case_id).duration_s


def command_for_block(case_id: str, block_index: int) -> tuple[float, float]:
    """Return a fixed case's requested ``(main, auxiliary)`` block target.

    Closed-loop cases intentionally have no schedule: their action is produced
    by ``OraclePathPolicy`` from the current observation.  Keeping that mode
    explicit prevents a future observation from silently influencing a fixed
    command case.
    """

    case = _case(case_id)
    if case.lookahead_m is not None:
        raise ValueError(f"case {case_id!r} uses closed-loop policy, not a fixed schedule")
    if isinstance(block_index, bool) or not isinstance(block_index, int):
        raise ValueError("block_index must be a non-negative integer")
    if block_index < 0 or block_index >= case.block_count:
        raise ValueError(
            f"block_index {block_index} is outside case {case_id!r} budget "
            f"[0, {case.block_count})"
        )
    for phase in case.phases:
        if phase.start_block <= block_index < phase.end_block:
            return (phase.main_command_rad_s, phase.auxiliary_angle_rad)
    raise RuntimeError(f"case {case_id!r} has no phase for block {block_index}")


def phase_for_block(case_id: str, block_index: int) -> str:
    """Return the named fixed schedule phase for artifact labeling."""

    case = _case(case_id)
    if case.lookahead_m is not None:
        raise ValueError(f"case {case_id!r} uses closed-loop policy, not a fixed schedule")
    if isinstance(block_index, bool) or not isinstance(block_index, int):
        raise ValueError("block_index must be a non-negative integer")
    if block_index < 0 or block_index >= case.block_count:
        raise ValueError(f"block_index {block_index} is outside case budget")
    for phase in case.phases:
        if phase.start_block <= block_index < phase.end_block:
            return phase.name
    raise RuntimeError(f"case {case_id!r} has no phase for block {block_index}")


def _trace_array(
    diagnostics: Mapping[str, object],
    names: Sequence[str],
    *,
    ndim: int,
    name: str,
) -> np.ndarray:
    value: object | None = None
    selected_name = name
    for candidate in names:
        if candidate in diagnostics:
            value = diagnostics[candidate]
            selected_name = candidate
            break
    if value is None:
        raise DiagnosticValidationError(f"diagnostic trace is missing {name}")
    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as error:
        raise DiagnosticValidationError(f"diagnostic trace {selected_name} is malformed") from error
    if array.ndim != ndim or array.size == 0:
        raise DiagnosticValidationError(
            f"diagnostic trace {selected_name} must have ndim={ndim}"
        )
    try:
        finite = np.all(np.isfinite(array))
    except TypeError as error:
        raise DiagnosticValidationError(
            f"diagnostic trace {selected_name} must be numeric"
        ) from error
    if not finite:
        raise DiagnosticValidationError(f"diagnostic trace {selected_name} must be finite")
    return array


def validate_physics_timing(
    diagnostics: Mapping[str, object],
    *,
    physics_hz: int = PHYSICS_HZ,
    tolerance_s: float = CLOCK_TOLERANCE_S,
) -> None:
    """Fail closed unless measured raw/relative clocks advance every physics tick."""

    if isinstance(physics_hz, bool) or not isinstance(physics_hz, int) or physics_hz <= 0:
        raise ValueError("physics_hz must be a positive integer")
    tolerance = float(tolerance_s)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("tolerance_s must be finite and non-negative")
    relative_time = _trace_array(
        diagnostics, ("simulation_time_s",), ndim=1, name="simulation_time_s"
    ).astype(np.float64)
    relative_step = _trace_array(
        diagnostics, ("physics_step_index",), ndim=1, name="physics_step_index"
    ).astype(np.float64)
    raw_time = _trace_array(
        diagnostics, ("raw_simulation_time_s",), ndim=1, name="raw_simulation_time_s"
    ).astype(np.float64)
    raw_step = _trace_array(
        diagnostics, ("raw_physics_step_index",), ndim=1, name="raw_physics_step_index"
    ).astype(np.float64)
    arrays = (relative_step, raw_step, relative_time, raw_time)
    if any(array.shape != relative_time.shape for array in arrays[1:]):
        raise DiagnosticValidationError("physics clock traces are not aligned")
    if not np.all(np.equal(relative_step, np.rint(relative_step))) or not np.all(
        np.equal(raw_step, np.rint(raw_step))
    ):
        raise DiagnosticValidationError("physics clock step traces must be integer-valued")
    if relative_step[0] < 0.0 or raw_step[0] < 0.0:
        raise DiagnosticValidationError("physics clock step traces must be non-negative")
    expected_dt = 1.0 / float(physics_hz)
    relative_step_delta = np.diff(relative_step)
    raw_step_delta = np.diff(raw_step)
    if not np.all(relative_step_delta == 1.0) or not np.all(raw_step_delta == 1.0):
        raise DiagnosticValidationError("physics clock step index skipped a world step")
    if relative_time.shape[0] > 1:
        if not np.all(np.abs(np.diff(relative_time) - expected_dt) <= tolerance):
            raise DiagnosticValidationError("relative physics clock time skipped a world step")
        if not np.all(np.abs(np.diff(raw_time) - expected_dt) <= tolerance):
            raise DiagnosticValidationError("raw physics clock time skipped a world step")
    if not np.allclose(
        relative_time,
        raw_time - raw_time[0] + relative_time[0],
        rtol=0.0,
        atol=tolerance,
    ):
        raise DiagnosticValidationError("relative physics clock is not raw-clock aligned")
    if not np.array_equal(
        relative_step, raw_step - raw_step[0] + relative_step[0]
    ):
        raise DiagnosticValidationError("relative physics steps are not raw-clock aligned")


def _summary(values: np.ndarray) -> dict[str, object]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "rms": float(np.sqrt(np.mean(np.square(values)))),
        "max_abs": float(np.max(np.abs(values))),
        "values": values.tolist(),
    }


def _tail_summary(values: np.ndarray, *, physics_hz: int) -> dict[str, object]:
    count = max(1, min(values.shape[0], int(round(2.0 * physics_hz))))
    tail = np.asarray(values[-count:], dtype=np.float64)
    return {
        "window_s": float(tail.shape[0] / physics_hz),
        "full_window_used": bool(tail.shape[0] == int(round(2.0 * physics_hz))),
        "peak_to_peak": float(np.max(tail) - np.min(tail)),
        "std": float(np.std(tail)),
        "values": tail.tolist(),
    }


def _first_within(values: np.ndarray, target: float, tolerance: float) -> int | None:
    within = np.abs(np.asarray(values, dtype=np.float64) - float(target)) <= tolerance
    required = max(1, int(round(SETTLING_REQUIRED_S * PHYSICS_HZ)))
    if within.shape[0] < required:
        return None
    for start in range(within.shape[0] - required + 1):
        if bool(np.all(within[start:])):
            return start
    return None


def _phase_statistics(
    phase: _Phase,
    sample_indices: np.ndarray,
    *,
    requested: np.ndarray,
    applied: np.ndarray,
    joint_positions: np.ndarray,
    joint_velocities: np.ndarray,
    yaw_rate: np.ndarray,
    roll: np.ndarray,
    physics_hz: int,
) -> dict[str, object]:
    if sample_indices.size == 0:
        raise DiagnosticValidationError(f"diagnostic phase {phase.name!r} has no samples")
    aux_target = float(phase.auxiliary_angle_rad)
    aux_actual = joint_positions[sample_indices, 1]
    aux_applied = applied[sample_indices, 1]
    main_actual = joint_velocities[sample_indices, 0]
    main_applied = applied[sample_indices, 0]
    aux_error = aux_actual - aux_applied
    main_error = main_actual - main_applied
    target_to_applied_index = None
    applied_to_actual_index = None
    applied_aux_within = np.abs(aux_applied - aux_target) <= SETTLING_BAND_RAD
    if np.any(applied_aux_within):
        target_to_applied_index = int(np.flatnonzero(applied_aux_within)[0])
    if abs(aux_target) > np.finfo(np.float64).eps:
        direction = math.copysign(1.0, aux_target)
        response_threshold = 0.9 * abs(aux_target)
        reached = direction * aux_actual >= response_threshold
        if np.any(reached):
            applied_to_actual_index = int(np.flatnonzero(reached)[0])
        peak_along_target = float(np.max(direction * aux_actual))
        overshoot_rad = max(0.0, peak_along_target - abs(aux_target))
        overshoot_percent: float | None = 100.0 * overshoot_rad / abs(aux_target)
    else:
        overshoot_rad = None
        overshoot_percent = None
        reached = np.ones(aux_actual.shape, dtype=bool)
    settling_index = _first_within(aux_actual, aux_target, SETTLING_BAND_RAD)
    tail_count = max(1, min(sample_indices.size, int(round(2.0 * physics_hz))))
    aux_tail = aux_actual[-tail_count:]
    roll_tail = roll[sample_indices][-tail_count:]
    yaw_rate_tail = yaw_rate[sample_indices][-tail_count:]
    return {
        "name": phase.name,
        "start_block": phase.start_block,
        "end_block": phase.end_block,
        "sample_count": int(sample_indices.size),
        "duration_s": float(sample_indices.size / physics_hz),
        "main_command_rad_s": phase.main_command_rad_s,
        "auxiliary_target_rad": aux_target,
        "main_actual_speed_rad_s": _summary(main_actual),
        "main_actual_applied_error": _summary(main_error),
        "aux_actual_applied_error": _summary(aux_error),
        "main_actual_applied_rms_rad_s": float(np.sqrt(np.mean(np.square(main_error)))),
        "main_actual_applied_max_abs_rad_s": float(np.max(np.abs(main_error))),
        "aux_actual_applied_rms_rad": float(np.sqrt(np.mean(np.square(aux_error)))),
        "aux_actual_applied_max_abs_rad": float(np.max(np.abs(aux_error))),
        "aux_tail": _tail_summary(aux_tail, physics_hz=physics_hz),
        "roll_tail": _tail_summary(roll_tail, physics_hz=physics_hz),
        "yaw_rate_tail": _tail_summary(yaw_rate_tail, physics_hz=physics_hz),
        "aux_tail_peak_to_peak_rad": float(np.max(aux_tail) - np.min(aux_tail)),
        "aux_tail_std_rad": float(np.std(aux_tail)),
        "roll_tail_peak_to_peak_rad": float(np.max(roll_tail) - np.min(roll_tail)),
        "roll_tail_std_rad": float(np.std(roll_tail)),
        "yaw_rate_tail_peak_to_peak_rad_s": float(np.max(yaw_rate_tail) - np.min(yaw_rate_tail)),
        "yaw_rate_tail_std_rad_s": float(np.std(yaw_rate_tail)),
        "overshoot_rad": overshoot_rad,
        "overshoot_percent": overshoot_percent,
        "settling_time_s": (
            None if settling_index is None else float(settling_index / physics_hz)
        ),
        "settling_band_rad": SETTLING_BAND_RAD,
        "settling_required_s": SETTLING_REQUIRED_S,
        "target_to_applied_delay_s": (
            None
            if target_to_applied_index is None
            else float(target_to_applied_index / physics_hz)
        ),
        "applied_to_actual_response_delay_s": (
            None
            if applied_to_actual_index is None
            else float(applied_to_actual_index / physics_hz)
        ),
        "target_to_applied_reached": target_to_applied_index is not None,
        "applied_to_actual_reached": applied_to_actual_index is not None,
        "requested_auxiliary_rad": requested[sample_indices, 1].tolist(),
        "applied_auxiliary_rad": aux_applied.tolist(),
        "actual_auxiliary_rad": aux_actual.tolist(),
    }


def compute_isolation_metrics(
    case_id: str,
    diagnostics: Mapping[str, object],
    *,
    physics_hz: int = PHYSICS_HZ,
) -> dict[str, object]:
    """Compute descriptive per-phase diagnostics from a 50 Hz post-step trace."""

    case = _case(case_id)
    if not isinstance(diagnostics, Mapping):
        raise TypeError("diagnostics must be a mapping")
    validate_physics_timing(diagnostics, physics_hz=physics_hz)
    requested = _trace_array(
        diagnostics,
        ("raw_command_target", "requested_target", "requested_action"),
        ndim=2,
        name="raw_command_target",
    ).astype(np.float64)
    clipped = _trace_array(
        diagnostics, ("clipped_target",), ndim=2, name="clipped_target"
    ).astype(np.float64)
    applied = _trace_array(
        diagnostics, ("applied_action", "applied_command"), ndim=2, name="applied_action"
    ).astype(np.float64)
    joint_positions = _trace_array(
        diagnostics,
        ("joint_position_rad", "joint_positions"),
        ndim=2,
        name="joint_position_rad",
    ).astype(np.float64)
    joint_velocities = _trace_array(
        diagnostics,
        ("joint_velocity_rad_s", "joint_velocities"),
        ndim=2,
        name="joint_velocity_rad_s",
    ).astype(np.float64)
    roll = _trace_array(diagnostics, ("roll_world_rad",), ndim=1, name="roll_world_rad").astype(np.float64)
    angular_velocity = _trace_array(
        diagnostics,
        ("angular_velocity_world_rad_s",),
        ndim=2,
        name="angular_velocity_world_rad_s",
    ).astype(np.float64)
    arrays = (requested, clipped, applied, joint_positions, joint_velocities, roll, angular_velocity)
    sample_count = requested.shape[0]
    if requested.shape[1] != 2 or clipped.shape != requested.shape or applied.shape != requested.shape:
        raise DiagnosticValidationError("command traces must have shape (N, 2)")
    if joint_positions.shape[0] != sample_count or joint_positions.shape[1] < 2:
        raise DiagnosticValidationError("joint_position_rad must have shape (N, >=2)")
    if joint_velocities.shape[0] != sample_count or joint_velocities.shape[1] < 1:
        raise DiagnosticValidationError("joint_velocity_rad_s must have shape (N, >=1)")
    if roll.shape != (sample_count,) or angular_velocity.shape[0] != sample_count or angular_velocity.shape[1] < 3:
        raise DiagnosticValidationError("root state traces are not aligned")
    block_values = diagnostics.get("action_block_index")
    if block_values is None:
        block_index = np.arange(sample_count, dtype=np.int64) // PHYSICS_STEPS_PER_BLOCK
    else:
        block_index = _trace_array(
            diagnostics, ("action_block_index",), ndim=1, name="action_block_index"
        ).astype(np.float64)
        if not np.all(np.equal(block_index, np.rint(block_index))) or np.any(block_index < 0):
            raise DiagnosticValidationError("action_block_index must be non-negative integers")
        block_index = block_index.astype(np.int64)
    if block_index.shape != (sample_count,):
        raise DiagnosticValidationError("action_block_index is not aligned")
    yaw_rate = angular_velocity[:, 2]
    if case.phases:
        phase_items = []
        for phase in case.phases:
            indices = np.flatnonzero(
                (block_index >= phase.start_block) & (block_index < phase.end_block)
            )
            phase_items.append(
                _phase_statistics(
                    phase,
                    indices,
                    requested=requested,
                    applied=applied,
                    joint_positions=joint_positions,
                    joint_velocities=joint_velocities,
                    yaw_rate=yaw_rate,
                    roll=roll,
                    physics_hz=physics_hz,
                )
            )
    else:
        phase_items = []
    aux_error = joint_positions[:, 1] - applied[:, 1]
    main_error = joint_velocities[:, 0] - applied[:, 0]
    aux_tail = _tail_summary(joint_positions[:, 1], physics_hz=physics_hz)
    roll_tail = _tail_summary(roll, physics_hz=physics_hz)
    yaw_rate_tail = _tail_summary(yaw_rate, physics_hz=physics_hz)
    summary = {
        "sample_count": int(sample_count),
        "duration_s": float(sample_count / physics_hz),
        "main_actual_applied_rms_rad_s": float(np.sqrt(np.mean(np.square(main_error)))),
        "main_actual_applied_max_abs_rad_s": float(np.max(np.abs(main_error))),
        "aux_actual_applied_rms_rad": float(np.sqrt(np.mean(np.square(aux_error)))),
        "aux_actual_applied_max_abs_rad": float(np.max(np.abs(aux_error))),
        "main_actual_applied_error": _summary(main_error),
        "aux_actual_applied_error": _summary(aux_error),
        "aux_tail": aux_tail,
        "roll_tail": roll_tail,
        "yaw_rate_tail": yaw_rate_tail,
        "aux_tail_peak_to_peak_rad": float(aux_tail["peak_to_peak"]),
        "aux_tail_std_rad": float(aux_tail["std"]),
        "roll_tail_peak_to_peak_rad": float(roll_tail["peak_to_peak"]),
        "roll_tail_std_rad": float(roll_tail["std"]),
        "yaw_rate_tail_peak_to_peak_rad_s": float(yaw_rate_tail["peak_to_peak"]),
        "yaw_rate_tail_std_rad_s": float(yaw_rate_tail["std"]),
    }
    return {
        "schema_version": "pid_isolation_metrics_v1",
        "case_id": case_id,
        "case_duration_s": case.duration_s,
        "phases": phase_items,
        "summary": summary,
        "clock": {
            "physics_hz": physics_hz,
            "post_step": True,
            "validated": True,
        },
        "trace_fields": list(diagnostics.keys()),
        "clipped_target": clipped.tolist(),
    }


def _jsonable(value: object) -> object:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _require_json_finite(value: object, field: str = "JSON") -> None:
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        raise ValueError(f"{field} must contain only finite numeric values")
    if isinstance(value, Mapping):
        for key, item in value.items():
            _require_json_finite(item, f"{field}.{key}")
    elif isinstance(value, (tuple, list)):
        for index, item in enumerate(value):
            _require_json_finite(item, f"{field}[{index}]")


def _canonical_json_bytes(payload: Mapping[str, object]) -> bytes:
    payload = _jsonable(payload)  # type: ignore[assignment]
    _require_json_finite(payload)
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_case_directory(output_root: str | Path, case_id: str) -> tuple[Path, Path]:
    """Create a unique partial directory and its eventual destination path."""

    if not isinstance(case_id, str) or not case_id or Path(case_id).name != case_id:
        raise ValueError("case_id must be one portable filename")
    _case(case_id)
    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    destination = root / case_id
    partial = root / f".{case_id}.partial"
    if destination.exists() or partial.exists():
        raise FileExistsError(f"refusing to overwrite existing PID isolation case: {destination}")
    partial.mkdir()
    return partial, destination


def _write_failure(partial: Path, error: BaseException) -> None:
    """Persist a failure before the caller closes its simulation runtime."""

    payload = {
        "schema_version": "pid_isolation_failure_v1",
        "error_type": type(error).__name__,
        "error": str(error),
    }
    try:
        (partial / "failure.json").write_bytes(_canonical_json_bytes(payload))
        (partial / "exception.txt").write_text(
            f"{type(error).__name__}: {error}\n", encoding="utf-8"
        )
    except OSError:
        # Preserve the original failure; the partial directory itself remains
        # available even if a filesystem error prevents the detail files.
        pass


def _write_trajectory_png(
    destination: Path,
    actual_xy: np.ndarray,
    reference_xy: np.ndarray | None = None,
) -> None:
    """Write a small dependency-light top-down diagnostic trajectory plot."""

    from PIL import Image, ImageDraw

    actual = np.asarray(actual_xy, dtype=np.float64)
    if actual.ndim != 2 or actual.shape[1] != 2 or actual.shape[0] == 0:
        raise DiagnosticValidationError("actual trajectory must have shape (N, 2)")
    points = [actual]
    if reference_xy is not None:
        reference = np.asarray(reference_xy, dtype=np.float64)
        if reference.ndim != 2 or reference.shape[1] != 2 or reference.shape[0] == 0:
            raise DiagnosticValidationError("reference trajectory must have shape (N, 2)")
        points.append(reference)
    else:
        reference = None
    all_points = np.concatenate(points, axis=0)
    if not np.all(np.isfinite(all_points)):
        raise DiagnosticValidationError("trajectory points must be finite")
    minimum = np.min(all_points, axis=0)
    maximum = np.max(all_points, axis=0)
    span = np.maximum(maximum - minimum, 1e-6)
    padding = np.maximum(span * 0.05, 0.1)
    minimum -= padding
    maximum += padding
    width, height = 800, 600
    image = Image.new("RGB", (width, height), (248, 249, 250))
    draw = ImageDraw.Draw(image)

    def pixel(point: np.ndarray) -> tuple[int, int]:
        x = 30.0 + (float(point[0]) - minimum[0]) / (maximum[0] - minimum[0]) * (width - 60.0)
        y = height - 30.0 - (float(point[1]) - minimum[1]) / (maximum[1] - minimum[1]) * (height - 60.0)
        return int(round(x)), int(round(y))

    if reference is not None and reference.shape[0] > 1:
        draw.line([pixel(point) for point in reference], fill=(35, 105, 210), width=3)
    if actual.shape[0] > 1:
        draw.line([pixel(point) for point in actual], fill=(230, 126, 34), width=3)
    start = pixel(actual[0])
    end = pixel(actual[-1])
    draw.ellipse((start[0] - 5, start[1] - 5, start[0] + 5, start[1] + 5), fill=(35, 150, 75))
    draw.ellipse((end[0] - 5, end[1] - 5, end[0] + 5, end[1] + 5), fill=(170, 45, 120))
    image.save(destination, format="PNG")


def write_pid_artifacts(
    output_root: str | Path,
    case_id: str,
    diagnostics: Mapping[str, object],
    *,
    metrics: Mapping[str, object] | None = None,
    block_states: Mapping[str, object] | None = None,
    actions: object | None = None,
    frames: Sequence[np.ndarray] | None = None,
    config: Mapping[str, object] | None = None,
    reference_xy: object | None = None,
    actual_xy: object | None = None,
    _partial_dir: Path | None = None,
    _destination: Path | None = None,
) -> Path:
    """Persist one complete PID case atomically, retaining failures as partials."""

    if _partial_dir is None:
        partial, destination = _atomic_case_directory(output_root, case_id)
    else:
        partial = Path(_partial_dir)
        destination = (
            Path(_destination)
            if _destination is not None
            else partial.parent / case_id
        )
        if not partial.is_dir():
            raise FileNotFoundError(f"partial PID isolation directory does not exist: {partial}")
    try:
        diagnostic_arrays = {
            name: np.asarray(value)
            for name, value in diagnostics.items()
        }
        if not diagnostic_arrays:
            raise DiagnosticValidationError("physics diagnostics are empty")
        np.savez(partial / "physics.npz", **diagnostic_arrays)
        if actions is not None:
            action_array = np.asarray(actions, dtype=np.float32)
            if action_array.ndim == 2 and action_array.shape[1] == 10:
                controls = action_array.reshape((-1, PHYSICS_STEPS_PER_BLOCK, 2))
            else:
                controls = action_array
            np.savez(partial / "commands.npz", action_blocks=action_array, controls=controls)
        if block_states is not None:
            np.savez(
                partial / "states.npz",
                **{name: np.asarray(value) for name, value in block_states.items()},
            )
        actual_array = (
            np.asarray(actual_xy, dtype=np.float64)
            if actual_xy is not None
            else np.asarray(diagnostic_arrays.get("position_world_xy"), dtype=np.float64)
        )
        if actual_array.ndim != 2 or actual_array.shape[1] != 2 or actual_array.shape[0] == 0:
            raise DiagnosticValidationError("actual trajectory is unavailable")
        _write_trajectory_png(
            partial / "trajectory.png",
            actual_array,
            None if reference_xy is None else np.asarray(reference_xy, dtype=np.float64),
        )
        if frames is None:
            raise DiagnosticValidationError("third-person video frames are unavailable")
        frame_list = tuple(np.asarray(frame) for frame in frames)
        if not frame_list:
            raise DiagnosticValidationError("third-person video frames are unavailable")
        from benchmark.artifacts import write_rgb_video

        write_rgb_video(partial / "third_person.mp4", frame_list, fps=float(CONTROL_HZ))
        metrics_payload = (
            dict(metrics)
            if metrics is not None
            else compute_isolation_metrics(case_id, diagnostics)
        )
        (partial / "metrics.json").write_bytes(_canonical_json_bytes(metrics_payload))
        config_payload = {
            "schema_version": "pid_isolation_case_v1",
            "case_id": case_id,
            "case_duration_s": case_duration_s(case_id),
            "physics_hz": PHYSICS_HZ,
            "control_hz": CONTROL_HZ,
            "trace_post_step": True,
            "trace_fields": list(diagnostics.keys()),
            "configuration": {} if config is None else _jsonable(config),
        }
        config_bytes = _canonical_json_bytes(config_payload)
        (partial / "config.json").write_bytes(config_bytes)
        (partial / "config.sha256").write_text(
            hashlib.sha256(config_bytes).hexdigest() + "\n", encoding="utf-8"
        )
        partial.replace(destination)
    except BaseException as error:
        _write_failure(partial, error)
        raise
    return destination


def _parse_positive_float(value: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError("value must be finite and positive") from error
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return result


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-id", required=True, choices=CASE_IDS)
    parser.add_argument("--run-root", required=True, type=Path)
    parser.add_argument("--scene-root", type=Path, default=None)
    parser.add_argument("--fixture", type=Path, default=None)
    parser.add_argument("--occupancy", type=Path, default=None)
    parser.add_argument("--contract", type=Path, default=None)
    parser.add_argument("--drive-profile", type=Path, default=None)
    parser.add_argument("--aux-kp", type=_parse_positive_float, default=DEFAULT_AUXILIARY_KP)
    parser.add_argument("--aux-kd", type=_parse_positive_float, default=DEFAULT_AUXILIARY_KD)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--gui", action="store_true", help="show Isaac Sim GUI")
    mode.add_argument("--headless", action="store_true", help="run without a GUI (default)")
    parser.add_argument("--no-video", action="store_true", help="disable video capture")
    parser.add_argument("--seed", type=int, default=3072)
    return parser


class _FixedCommandPolicy:
    """Small non-privileged policy whose action comes only from a fixed schedule."""

    policy_name = "pid_isolation_fixed_schedule"
    privileged = False

    def __init__(self, case_id: str, action_protocol: Any) -> None:
        self._case_id = case_id
        self._action_protocol = action_protocol
        self._block_index = 0

    def reset(self, _context: object) -> None:
        self._block_index = 0

    def act(self, _observation: object, _pointgoal_base_xy: np.ndarray) -> np.ndarray:
        command = command_for_block(self._case_id, self._block_index)
        self._block_index += 1
        return np.tile(
            np.asarray(command, dtype=np.float32),
            int(self._action_protocol.controls_per_block),
        )


def _relative_scene_uri(scene_root: Path, fixture: Path) -> str:
    try:
        relative = fixture.resolve().relative_to(scene_root.resolve())
    except ValueError as error:
        raise ValueError("fixture must reside below scene_root") from error
    if not relative.parts:
        raise ValueError("fixture must name a file")
    return "${SCENE_ROOT}/" + relative.as_posix()


def _build_episode(
    case_id: str,
    *,
    scene_root: Path,
    fixture: Path,
    occupancy: Path,
    seed: int,
) -> Any:
    from benchmark.manifest import BenchmarkEpisode, HumanReview

    case = _case(case_id)
    if case.lookahead_m is None:
        goal_xy = (12.0, 0.0)
        path_length = 12.0
    else:
        goal_xy = (3.0, 0.0)
        path_length = 3.0
    return BenchmarkEpisode(
        episode_id=f"pid_isolation_{case_id}",
        split="dev",
        difficulty="E0",
        scene_uri=_relative_scene_uri(scene_root, fixture),
        scene_sha256=_sha256(fixture),
        occupancy_sha256=_sha256(occupancy),
        start_position_world_m=(0.0, 0.0, 0.45),
        start_quaternion_world_wxyz=(1.0, 0.0, 0.0, 0.0),
        goal_position_world_m=(goal_xy[0], goal_xy[1], 0.45),
        euclidean_distance_m=path_length,
        oracle_path_length_m=path_length,
        seed=int(seed),
        human_review=HumanReview("approved", "pid_isolation", "diagnostic fixture"),
        classification_evidence={
            "pid_isolation": True,
            "case_id": case_id,
            "fixed_command_case": case.lookahead_m is None,
            "target_avoids_early_goal": case.lookahead_m is None,
        },
    )


def _reference_for_case(case_id: str) -> np.ndarray:
    case = _case(case_id)
    end = 12.0 if case.lookahead_m is None else 3.0
    return np.asarray(((0.0, 0.0), (end, 0.0)), dtype=np.float64)


def _case_config(
    case_id: str,
    *,
    scene_root: Path,
    fixture: Path,
    occupancy: Path,
    contract: Path,
    drive_profile: Path,
    aux_kp: float,
    aux_kd: float,
    gui: bool,
    record_video: bool,
    seed: int,
    runtime_configuration: Mapping[str, object] | None = None,
) -> dict[str, object]:
    case = _case(case_id)
    return {
        "schema_version": "pid_isolation_case_v1",
        "case": {
            "case_id": case_id,
            "duration_s": case.duration_s,
            "fixed_schedule": case.lookahead_m is None,
            "lookahead_m": case.lookahead_m,
            "phases": [
                {
                    "name": phase.name,
                    "start_block": phase.start_block,
                    "end_block": phase.end_block,
                    "main_command_rad_s": phase.main_command_rad_s,
                    "auxiliary_angle_rad": phase.auxiliary_angle_rad,
                }
                for phase in case.phases
            ],
        },
        "controller": {
            "action_protocol_id": "ball_action_v2_aux30",
            "main_acceleration_limit_rad_s2": 0.5,
            "main_derivative_gain": DEFAULT_MAIN_DERIVATIVE_GAIN,
            "auxiliary_kp_baseline": DEFAULT_AUXILIARY_KP,
            "auxiliary_kd_baseline": DEFAULT_AUXILIARY_KD,
            "auxiliary_kp_requested": aux_kp,
            "auxiliary_kd_requested": aux_kd,
            "auxiliary_target_rate_limit_rad_s": DEFAULT_AUXILIARY_RATE_RAD_S,
            "auxiliary_bounds_rad": [-math.radians(30.0), math.radians(30.0)],
            "drive_model_revalidated": False,
        },
        "clock": {
            "physics_hz": PHYSICS_HZ,
            "control_hz": CONTROL_HZ,
            "physics_steps_per_block": PHYSICS_STEPS_PER_BLOCK,
            "trace_post_step": True,
        },
        "runtime": {
            "diagnostic_open_plane": True,
            "diagnostic_physics_trace": True,
            "gui": bool(gui),
            "record_video": bool(record_video),
            "seed": int(seed),
            "configuration": {} if runtime_configuration is None else _jsonable(runtime_configuration),
        },
        "assets": {
            "scene_root": str(scene_root.resolve()),
            "fixture_path": str(fixture.resolve()),
            "fixture_sha256": _sha256(fixture),
            "occupancy_path": str(occupancy.resolve()),
            "occupancy_sha256": _sha256(occupancy),
            "contract_path": str(contract.resolve()),
            "contract_sha256": _sha256(contract),
            "drive_profile_path": str(drive_profile.resolve()),
            "drive_profile_sha256": _sha256(drive_profile),
        },
        "source": {
            "script_path": str(Path(__file__).resolve()),
            "script_sha256": _sha256(Path(__file__).resolve()),
            "git_revision": None,
            "python_version": platform.python_version(),
            "numpy_version": np.__version__,
        },
    }


def _result_states(result: object) -> dict[str, np.ndarray]:
    fields = (
        "positions_world_xy",
        "yaw_world_rad",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "applied_action",
        "clearances_m",
        "physical_contacts",
        "collisions",
        "simulation_time_s",
        "physics_step_index",
        "raw_simulation_time_s",
        "raw_physics_step_index",
        "roll_world_rad",
    )
    result_states: dict[str, np.ndarray] = {}
    for name in fields:
        if hasattr(result, name):
            value = getattr(result, name)
            if value is not None:
                result_states[name] = np.asarray(value)
    return result_states


def run_pid_case(
    case_id: str,
    *,
    run_root: str | Path,
    scene_root: str | Path | None = None,
    fixture: str | Path | None = None,
    occupancy: str | Path | None = None,
    contract: str | Path | None = None,
    drive_profile: str | Path | None = None,
    aux_kp: float = DEFAULT_AUXILIARY_KP,
    aux_kd: float = DEFAULT_AUXILIARY_KD,
    gui: bool = False,
    record_video: bool = True,
    seed: int = 3072,
    runtime_factory: Any | None = None,
) -> dict[str, object]:
    """Run exactly one case and publish a complete diagnostic bundle."""

    case = _case(case_id)
    project_root = Path(__file__).resolve().parents[1]
    resolved_scene_root = (
        Path(scene_root).expanduser().resolve()
        if scene_root is not None
        else (project_root / "data" / "benchmark_v1" / "scenes").resolve()
    )
    resolved_fixture = (
        Path(fixture).expanduser().resolve()
        if fixture is not None
        else (resolved_scene_root / "tracking_open_plane.usda").resolve()
    )
    resolved_occupancy = (
        Path(occupancy).expanduser().resolve()
        if occupancy is not None
        else (
            project_root
            / "data"
            / "benchmark_v1"
            / "occupancy"
            / "tracking_open_plane_free.npz"
        ).resolve()
    )
    resolved_contract = (
        Path(contract).expanduser().resolve()
        if contract is not None
        else (project_root / "configs" / "data_contract.yaml").resolve()
    )
    resolved_drive_profile = (
        Path(drive_profile).expanduser().resolve()
        if drive_profile is not None
        else (project_root / "configs" / "benchmark_v1" / "drive_dynamics_v1.json").resolve()
    )
    for path, label in (
        (resolved_fixture, "fixture"),
        (resolved_occupancy, "occupancy"),
        (resolved_contract, "contract"),
        (resolved_drive_profile, "drive profile"),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"{label} does not exist: {path}")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a non-negative integer")
    aux_kp = _parse_positive_float(str(aux_kp))
    aux_kd = _parse_positive_float(str(aux_kd))
    if not isinstance(gui, bool) or not isinstance(record_video, bool):
        raise ValueError("gui and record_video must be booleans")

    partial, destination = _atomic_case_directory(run_root, case_id)
    runtime = None
    return_code = 1
    try:
        initial_config = _case_config(
            case_id,
            scene_root=resolved_scene_root,
            fixture=resolved_fixture,
            occupancy=resolved_occupancy,
            contract=resolved_contract,
            drive_profile=resolved_drive_profile,
            aux_kp=aux_kp,
            aux_kd=aux_kd,
            gui=gui,
            record_video=record_video,
            seed=seed,
        )
        config_bytes = _canonical_json_bytes(initial_config)
        (partial / "config.json").write_bytes(config_bytes)
        (partial / "config.sha256").write_text(
            hashlib.sha256(config_bytes).hexdigest() + "\n", encoding="utf-8"
        )

        if runtime_factory is None:
            from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig

            runtime_factory = IsaacBenchmarkRuntime
            runtime_config_factory = IsaacRuntimeConfig
        else:
            runtime_config_factory = None
        if runtime_config_factory is None:
            # Test doubles can accept the same configuration object without importing
            # Isaac; keeping this import local preserves the CPU-only entry point.
            from benchmark.isaac_runtime import IsaacRuntimeConfig

            runtime_config_factory = IsaacRuntimeConfig
        runtime_config = runtime_config_factory(
            scene_root=resolved_scene_root,
            contract_path=resolved_contract,
            gui=gui,
            record_video=record_video,
            occupancy_path=resolved_occupancy,
            drive_profile_path=resolved_drive_profile,
            diagnostic_open_plane=True,
            diagnostic_physics_trace=True,
            diagnostic_auxiliary_kp=aux_kp,
            diagnostic_auxiliary_kd=aux_kd,
        )
        runtime = runtime_factory(runtime_config)
        reference_xy = _reference_for_case(case_id)
        setter = getattr(runtime, "set_reference_visualization_path", None)
        if callable(setter):
            setter(reference_xy)

        from benchmark.manifest import BenchmarkEpisode
        from benchmark.runner import RunnerConfig, run_episode
        from training.action_protocol import load_action_protocol

        episode = _build_episode(
            case_id,
            scene_root=resolved_scene_root,
            fixture=resolved_fixture,
            occupancy=resolved_occupancy,
            seed=seed,
        )
        del BenchmarkEpisode  # keep static type checkers from treating the import as a runtime dependency
        action_protocol = load_action_protocol(resolved_contract)
        if action_protocol.protocol_id != "ball_action_v2_aux30":
            raise ValueError("PID isolation requires ball_action_v2_aux30")
        if case.lookahead_m is None:
            policy: Any = _FixedCommandPolicy(case_id, action_protocol)
            kinodynamic_plan = None
            oracle_path = None
            settling_budget_s = 0.0
        else:
            from benchmark.drive_model import DriveModel
            from benchmark.kinodynamic_planning import KinodynamicPlanResult
            from benchmark.policies import OraclePathPolicy
            from benchmark.tracking_acceptance import TrackingCase, build_tracking_reference

            tracking_case = TrackingCase(
                case_id=f"pid_{case_id}",
                kind="line",
                main_command_rad_s=0.5,
                budget_s=case.duration_s,
                parameters={
                    "start_x_m": 0.0,
                    "start_y_m": 0.0,
                    "start_yaw_rad": 0.0,
                    "length_m": 3.0,
                },
            )
            reference = build_tracking_reference(tracking_case)
            policy = OraclePathPolicy(
                action_protocol,
                lookahead_m=float(case.lookahead_m),
                drive_model=DriveModel.from_json(resolved_drive_profile),
            )
            kinodynamic_plan = KinodynamicPlanResult(
                status="success",
                waypoints=reference.waypoints,
                expanded_nodes=0,
                path_length_m=reference.path_length_m,
                direction_changes=0,
            )
            oracle_path = reference.points_xy
            settling_budget_s = min(3.0, case.duration_s)
        result = run_episode(
            runtime,
            policy,
            episode,
            action_protocol=action_protocol,
            config=RunnerConfig(
                episode_budget_s=case.duration_s,
                replan_hz=CONTROL_HZ,
                goal_tolerance_m=0.2,
                diagnostic_settling_budget_s=settling_budget_s,
            ),
            oracle_path_xy=oracle_path,
            kinodynamic_plan=kinodynamic_plan,
        )
        if getattr(result, "termination_reason", None) == "runtime_error":
            runtime_error = getattr(result, "runtime_error", None)
            if not isinstance(runtime_error, str) or not runtime_error:
                runtime_error = "run_episode returned runtime_error without a message"
            raise RuntimeError(runtime_error)
        diagnostics = runtime.physics_diagnostics
        if not diagnostics:
            raise DiagnosticValidationError("runtime did not expose physics diagnostics")
        validate_physics_timing(diagnostics)
        metrics = compute_isolation_metrics(case_id, diagnostics)
        runtime_configuration = getattr(runtime, "diagnostic_configuration", {})
        final_config = _case_config(
            case_id,
            scene_root=resolved_scene_root,
            fixture=resolved_fixture,
            occupancy=resolved_occupancy,
            contract=resolved_contract,
            drive_profile=resolved_drive_profile,
            aux_kp=aux_kp,
            aux_kd=aux_kd,
            gui=gui,
            record_video=record_video,
            seed=seed,
            runtime_configuration=runtime_configuration,
        )
        frames_payload = getattr(runtime, "captured_frames", {})
        frames = (
            frames_payload.get("third_person", ())
            if isinstance(frames_payload, Mapping)
            else ()
        )
        output = write_pid_artifacts(
            run_root,
            case_id,
            diagnostics,
            metrics=metrics,
            block_states=_result_states(result),
            actions=getattr(result, "actions", None),
            frames=frames,
            config=final_config,
            reference_xy=reference_xy,
            actual_xy=getattr(result, "positions_world_xy", None),
            _partial_dir=partial,
            _destination=destination,
        )
        return_code = 0
        metrics_payload = dict(metrics)
        metrics_payload.update(
            {
                "episode_id": episode.episode_id,
                "termination_reason": getattr(result, "termination_reason", None),
                "runtime_error": getattr(result, "runtime_error", None),
                "output_dir": str(output),
            }
        )
        return metrics_payload
    except BaseException as error:
        _write_failure(partial, error)
        raise
    finally:
        if runtime is not None:
            runtime.close(return_code=return_code)


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.seed < 0:
        print("--seed must be non-negative", file=sys.stderr)
        return 2
    try:
        metrics = run_pid_case(
            args.case_id,
            run_root=args.run_root,
            scene_root=args.scene_root,
            fixture=args.fixture,
            occupancy=args.occupancy,
            contract=args.contract,
            drive_profile=args.drive_profile,
            aux_kp=args.aux_kp,
            aux_kd=args.aux_kd,
            gui=bool(args.gui),
            record_video=not bool(args.no_video),
            seed=args.seed,
        )
        print(json.dumps(_jsonable(metrics), sort_keys=True, ensure_ascii=False))
        return 0
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        print(f"PID isolation failed: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
