#!/usr/bin/env python3
"""Run the eight-case, free-space tracking acceptance diagnostic.

This entry point intentionally stays thin: geometry is supplied by
``benchmark.tracking_acceptance`` and motion is executed by the existing Isaac
runtime, runner, and ``OraclePathPolicy``.  The coordinator launches one
fresh child process per case because an Isaac ``SimulationApp`` is not safe to
close and recreate in one process.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
from types import SimpleNamespace
from typing import Any, Mapping, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark.artifacts import artifact_sha256, write_rgb_video  # noqa: E402
from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig  # noqa: E402
from benchmark.drive_model import DriveModel  # noqa: E402
from benchmark.kinodynamic_planning import KinodynamicPlanResult  # noqa: E402
from benchmark.manifest import BenchmarkEpisode, HumanReview  # noqa: E402
from benchmark.policies import (  # noqa: E402
    DelayAwareTrajectoryOraclePolicy,
    OraclePathPolicy,
    TrajectoryOraclePolicy,
)
from benchmark.runner import RunnerConfig, run_episode  # noqa: E402
from benchmark.tracking_acceptance import (  # noqa: E402
    TrackingCase,
    TrackingReference,
    build_tracking_reference,
    heading_errors_rad,
    load_tracking_cases,
    polyline_cross_track_errors,
)
from scripts.trajectory_map import TrajectoryRecord, render_trajectory_map  # noqa: E402
from training.action_protocol import load_action_protocol  # noqa: E402


_HERE = Path(__file__).resolve()
_PROJECT_ROOT = _HERE.parents[1]
DEFAULT_CONFIG = _PROJECT_ROOT / "configs" / "benchmark_v1" / "tracking_acceptance.json"
DEFAULT_CONTRACT = _PROJECT_ROOT / "configs" / "data_contract.yaml"
DEFAULT_SCENE_ROOT = _PROJECT_ROOT / "data" / "benchmark_v1" / "scenes"
DEFAULT_FIXTURE = DEFAULT_SCENE_ROOT / "tracking_open_plane.usda"
DEFAULT_OCCUPANCY = (
    _PROJECT_ROOT / "data" / "benchmark_v1" / "occupancy" / "tracking_open_plane_free.npz"
)
DEFAULT_DRIVE_PROFILE = _PROJECT_ROOT / "configs" / "benchmark_v1" / "drive_dynamics_v1.json"
DEFAULT_LOOKAHEAD_M = 0.5
DEFAULT_YAW_RATE_FEEDBACK_GAIN_S = 0.15
DEFAULT_POLICY_VARIANT = "baseline"
TRAJECTORY_POLICY_VARIANT = "oracle_trajectory_v1"
TRAJECTORY_POLICY_V2_VARIANT = "oracle_trajectory_v2"
POLICY_VARIANTS = (
    DEFAULT_POLICY_VARIANT,
    TRAJECTORY_POLICY_VARIANT,
    TRAJECTORY_POLICY_V2_VARIANT,
)
TRAJECTORY_POLICY_DEFAULTS: dict[str, float] = {
    "lookahead_m": 1.0,
    "smoothing_window_m": 0.40,
    "curvature_preview_m": 0.80,
    "lateral_curvature_gain_1_m2": 0.20,
    "heading_curvature_gain_1_m": 0.15,
    "max_feedback_curvature_1_m": 0.20,
    "yaw_rate_feedback_gain_s": 0.15,
    "latency_preview_s": 0.0,
}
TRAJECTORY_POLICY_PARAMETER_NAMES = tuple(TRAJECTORY_POLICY_DEFAULTS)
TRAJECTORY_POLICY_V2_DEFAULTS: dict[str, float] = {
    "auxiliary_latency_s": 1.0,
    "maximum_delay_distance_m": 0.30,
    "profile_window_m": 0.30,
    "profile_blend_start_1_m": 0.05,
    "profile_blend_full_1_m": 0.20,
    "feedback_lookahead_m": 1.0,
    "maximum_feedback_curvature_1_m": 0.20,
    "supported_main_command_rad_s": 0.50,
    "episode_budget_s": 35.0,
    "terminal_brake_buffer_m": 0.10,
}
TRAJECTORY_POLICY_V2_PARAMETER_NAMES = tuple(TRAJECTORY_POLICY_V2_DEFAULTS)
TRAJECTORY_DIAGNOSTIC_NAMES = (
    "reference_curvature_1_m",
    "feedback_curvature_1_m",
    "command_curvature_1_m",
    "lateral_error_m",
    "heading_error_rad",
    "main_cap_rad_s",
    "auxiliary_feedforward_rad",
    "auxiliary_feedback_rad",
)
TRAJECTORY_POLICY_V2_DIAGNOSTIC_NAMES = (
    "projection_s_m",
    "delay_distance_m",
    "query_s_m",
    "pure_pursuit_curvature_1_m",
    "profile_curvature_1_m",
    "blend_weight",
    "budget_infeasible",
    "terminal_runway_s",
    "terminal_brake_start_s",
    "goal_entry_s",
    "stop_below_0p05_s",
    "hold_complete_s",
    "elapsed_s",
)
TRAJECTORY_POLICY_V2_NUMERIC_DIAGNOSTIC_NAMES = tuple(
    name
    for name in TRAJECTORY_POLICY_V2_DIAGNOSTIC_NAMES
    if name != "budget_infeasible"
)

CLOCK_TIME_TOLERANCE_S = 5e-5
BOUNDARY_TOLERANCE_M = 0.2


class DiagnosticValidationError(ValueError):
    """A fail-closed diagnostic evidence or artifact validation error."""


class _CaseIdAction(argparse.Action):
    """Selecting one case implicitly disables the default all-case mode."""

    def __call__(self, parser, namespace, values, option_string=None):  # type: ignore[no-untyped-def]
        setattr(namespace, self.dest, values)
        setattr(namespace, "all_cases", False)


class _MarkProvidedAction(argparse.Action):
    """Store a value and remember that the caller supplied the option."""

    def __call__(self, parser, namespace, values, option_string=None):  # type: ignore[no-untyped-def]
        setattr(namespace, self.dest, values)
        setattr(namespace, f"_{self.dest}_provided", True)


class _TrackingArgumentParser(argparse.ArgumentParser):
    """Apply variant-specific defaults after normal argparse conversion."""

    def parse_args(self, args=None, namespace=None):  # type: ignore[no-untyped-def]
        parsed = super().parse_args(args=args, namespace=namespace)
        if getattr(parsed, "policy_variant", DEFAULT_POLICY_VARIANT) == TRAJECTORY_POLICY_VARIANT:
            if not getattr(parsed, "_lookahead_m_provided", False):
                # The historical baseline parser default remains 0.5 m.  V1
                # has its own fixed 1.0 m default unless explicitly selected.
                parsed.lookahead_m = TRAJECTORY_POLICY_DEFAULTS["lookahead_m"]
        return parsed


def _validate_policy_variant(value: object) -> str:
    if not isinstance(value, str) or value not in POLICY_VARIANTS:
        raise ValueError(
            "policy_variant must be one of " + ", ".join(POLICY_VARIANTS)
        )
    return value


def _validate_v1_scalar(
    value: object,
    *,
    name: str,
    strictly_positive: bool = False,
) -> float:
    if isinstance(value, (bool, np.bool_)):
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}") from error
    valid = numeric > 0.0 if strictly_positive else numeric >= 0.0
    if not math.isfinite(numeric) or not valid:
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return numeric


def _validate_v2_scalar(
    value: object,
    *,
    name: str,
    strictly_positive: bool = False,
) -> float:
    """Validate one finite V2 constructor parameter at the CLI boundary."""

    if isinstance(value, (bool, np.bool_)):
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}") from error
    valid = numeric > 0.0 if strictly_positive else numeric >= 0.0
    if not math.isfinite(numeric) or not valid:
        qualifier = "positive" if strictly_positive else "non-negative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return numeric


def _parse_v2_scalar(
    value: str, *, name: str, strictly_positive: bool = False
) -> float:
    try:
        return _validate_v2_scalar(
            value,
            name=name,
            strictly_positive=strictly_positive,
        )
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _parse_auxiliary_latency_s(value: str) -> float:
    return _parse_v2_scalar(value, name="auxiliary_latency_s")


def _parse_auxiliary_target_rate_deg_s(value: str) -> float:
    return _parse_v2_scalar(
        value,
        name="auxiliary_target_rate_deg_s",
        strictly_positive=True,
    )


def _parse_auxiliary_velocity_limit_rad_s(value: str) -> float:
    return _parse_v2_scalar(
        value,
        name="auxiliary_velocity_limit_rad_s",
        strictly_positive=True,
    )


def _parse_maximum_delay_distance_m(value: str) -> float:
    return _parse_v2_scalar(value, name="maximum_delay_distance_m")


def _parse_profile_window_m(value: str) -> float:
    return _parse_v2_scalar(value, name="profile_window_m")


def _parse_profile_blend_start_1_m(value: str) -> float:
    return _parse_v2_scalar(value, name="profile_blend_start_1_m")


def _parse_profile_blend_full_1_m(value: str) -> float:
    return _parse_v2_scalar(value, name="profile_blend_full_1_m")


def _parse_feedback_lookahead_m(value: str) -> float:
    return _parse_v2_scalar(
        value,
        name="feedback_lookahead_m",
        strictly_positive=True,
    )


def _parse_maximum_feedback_curvature_1_m(value: str) -> float:
    return _parse_v2_scalar(value, name="maximum_feedback_curvature_1_m")


def _parse_supported_main_command_rad_s(value: str) -> float:
    return _parse_v2_scalar(value, name="supported_main_command_rad_s")


def _parse_episode_budget_s(value: str) -> float:
    return _parse_v2_scalar(
        value,
        name="episode_budget_s",
        strictly_positive=True,
    )


def _parse_terminal_brake_buffer_m(value: str) -> float:
    return _parse_v2_scalar(value, name="terminal_brake_buffer_m")


def _validate_latency_preview_s(value: object) -> float:
    latency = _validate_v1_scalar(value, name="latency_preview_s")
    if latency != 0.0:
        raise ValueError("latency_preview_s must be exactly 0.0 in V1")
    return latency


def _parse_v1_scalar(value: str, *, name: str, strictly_positive: bool = False) -> float:
    try:
        return _validate_v1_scalar(value, name=name, strictly_positive=strictly_positive)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _parse_smoothing_window_m(value: str) -> float:
    return _parse_v1_scalar(value, name="smoothing_window_m")


def _parse_curvature_preview_m(value: str) -> float:
    return _parse_v1_scalar(value, name="curvature_preview_m")


def _parse_lateral_curvature_gain_1_m2(value: str) -> float:
    return _parse_v1_scalar(value, name="lateral_curvature_gain_1_m2")


def _parse_heading_curvature_gain_1_m(value: str) -> float:
    return _parse_v1_scalar(value, name="heading_curvature_gain_1_m")


def _parse_max_feedback_curvature_1_m(value: str) -> float:
    return _parse_v1_scalar(value, name="max_feedback_curvature_1_m")


def _parse_latency_preview_s(value: str) -> float:
    try:
        return _validate_latency_preview_s(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _validate_lookahead_m(value: object) -> float:
    if isinstance(value, bool):
        raise ValueError("lookahead_m must be finite and positive")
    try:
        lookahead_m = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("lookahead_m must be finite and positive") from error
    if not math.isfinite(lookahead_m) or lookahead_m <= 0.0:
        raise ValueError("lookahead_m must be finite and positive")
    return lookahead_m


def _parse_lookahead_m(value: str) -> float:
    try:
        return _validate_lookahead_m(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _validate_yaw_rate_feedback_gain_s(value: object) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(
            "yaw_rate_feedback_gain_s must be finite and non-negative"
        )
    try:
        gain = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            "yaw_rate_feedback_gain_s must be finite and non-negative"
        ) from error
    if not math.isfinite(gain) or gain < 0.0:
        raise ValueError("yaw_rate_feedback_gain_s must be finite and non-negative")
    return gain


def _parse_yaw_rate_feedback_gain_s(value: str) -> float:
    try:
        return _validate_yaw_rate_feedback_gain_s(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def _resolve_policy_parameters(
    policy_variant: object,
    values: Mapping[str, object] | None = None,
) -> dict[str, float]:
    """Resolve and validate controller parameters for one policy variant."""

    variant = _validate_policy_variant(policy_variant)
    supplied = {} if values is None else dict(values)
    if variant == DEFAULT_POLICY_VARIANT:
        lookahead = supplied.get("lookahead_m", DEFAULT_LOOKAHEAD_M)
        yaw_rate_gain = supplied.get(
            "yaw_rate_feedback_gain_s", DEFAULT_YAW_RATE_FEEDBACK_GAIN_S
        )
        if lookahead is None:
            lookahead = DEFAULT_LOOKAHEAD_M
        if yaw_rate_gain is None:
            yaw_rate_gain = DEFAULT_YAW_RATE_FEEDBACK_GAIN_S
        return {
            "lookahead_m": _validate_lookahead_m(lookahead),
            "yaw_rate_feedback_gain_s": _validate_yaw_rate_feedback_gain_s(
                yaw_rate_gain
            ),
        }

    if variant == TRAJECTORY_POLICY_VARIANT:
        resolved_v1: dict[str, float] = {}
        for name, default in TRAJECTORY_POLICY_DEFAULTS.items():
            value = supplied.get(name, default)
            if value is None:
                value = default
            if name == "lookahead_m":
                resolved_v1[name] = _validate_v1_scalar(
                    value, name=name, strictly_positive=True
                )
            elif name == "latency_preview_s":
                resolved_v1[name] = _validate_latency_preview_s(value)
            else:
                resolved_v1[name] = _validate_v1_scalar(value, name=name)
        return resolved_v1

    resolved_v2: dict[str, float] = {}
    for name, default in TRAJECTORY_POLICY_V2_DEFAULTS.items():
        value = supplied.get(name, default)
        if value is None:
            value = default
        resolved_v2[name] = _validate_v2_scalar(
            value,
            name=name,
            strictly_positive=name in {"feedback_lookahead_m", "episode_budget_s"},
        )
    if resolved_v2["maximum_delay_distance_m"] > 0.30:
        raise ValueError("maximum_delay_distance_m must not exceed 0.30 m")
    if (
        resolved_v2["profile_blend_full_1_m"]
        <= resolved_v2["profile_blend_start_1_m"]
    ):
        raise ValueError("profile_blend_full_1_m must exceed profile_blend_start_1_m")
    if resolved_v2["supported_main_command_rad_s"] != 0.50:
        raise ValueError(
            "supported_main_command_rad_s must be the calibrated 0.50 rad/s operating point"
        )
    return resolved_v2


def _v2_parameters_from_payload(payload: Mapping[str, object]) -> dict[str, float]:
    """Read the frozen V2 parameter block from a raw or case config.

    The raw tracking config uses ``trajectory_policy_v2`` while published
    case artifacts use ``trajectory_policy``.  Both forms are accepted here,
    but the name and parameter set remain exact so a partial or cross-variant
    block cannot silently fall back to code defaults.
    """

    block = payload.get("trajectory_policy_v2")
    if block is None:
        block = payload.get("trajectory_policy")
    if not isinstance(block, Mapping):
        raise ValueError("V2 config is missing trajectory_policy_v2")
    if block.get("name") != TRAJECTORY_POLICY_V2_VARIANT:
        raise ValueError("V2 config trajectory policy name is invalid")
    parameters = block.get("parameters")
    if not isinstance(parameters, Mapping):
        raise ValueError("V2 config trajectory policy parameters are missing")
    expected_names = set(TRAJECTORY_POLICY_V2_PARAMETER_NAMES)
    actual_names = set(parameters)
    missing = sorted(expected_names - actual_names)
    extra = sorted(actual_names - expected_names)
    if missing or extra:
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if extra:
            details.append("extra=" + ",".join(extra))
        raise ValueError(
            "V2 config trajectory policy parameters must contain exactly the "
            + "frozen names ("
            + "; ".join(details)
            + ")"
        )
    return _resolve_policy_parameters(TRAJECTORY_POLICY_V2_VARIANT, parameters)


def _v2_parameters_from_config(path: str | Path) -> dict[str, float]:
    """Load the authoritative V2 parameter block from ``path``."""

    return _v2_parameters_from_payload(_read_json_object(path))


def _policy_parameters_from_namespace(args: argparse.Namespace) -> dict[str, float]:
    parameter_names = (
        TRAJECTORY_POLICY_V2_PARAMETER_NAMES
        if args.policy_variant == TRAJECTORY_POLICY_V2_VARIANT
        else TRAJECTORY_POLICY_PARAMETER_NAMES
    )
    if args.policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
        authority = _v2_parameters_from_config(args.config)
        values: dict[str, object] = {}
        for name in parameter_names:
            # The parser records explicit CLI presence independently from the
            # numeric default.  A default-valued argparse attribute is not an
            # override of the raw configuration block.
            marker = f"_{name}_provided"
            values[name] = (
                getattr(args, name)
                if bool(getattr(args, marker, False))
                else authority[name]
            )
        return _resolve_policy_parameters(args.policy_variant, values)
    values = {name: getattr(args, name, None) for name in parameter_names}
    return _resolve_policy_parameters(args.policy_variant, values)


def _v2_explicit_parameters_from_namespace(
    args: argparse.Namespace,
) -> dict[str, float | None]:
    """Return only V2 values that were present on this parser invocation."""

    return {
        name: (
            getattr(args, name)
            if bool(getattr(args, f"_{name}_provided", False))
            else None
        )
        for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES
    }


def _summary_policy_kwargs(
    policy_variant: object,
    policy_parameters: Mapping[str, float],
) -> dict[str, object]:
    """Return only parameters resolved for the selected summary variant."""

    variant = _validate_policy_variant(policy_variant)
    parameter_names = set(
        TRAJECTORY_POLICY_V2_PARAMETER_NAMES
        if variant == TRAJECTORY_POLICY_V2_VARIANT
        else TRAJECTORY_POLICY_PARAMETER_NAMES
    )
    return {
        "policy_variant": variant,
        **{
            name: value
            for name, value in policy_parameters.items()
            if name in parameter_names
        },
    }


def _reject_json_constant(token: str) -> object:
    raise ValueError(f"JSON must contain finite values, got {token}")


def _require_json_finite(value: object, field: str = "JSON") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{field} must contain only finite numeric values")
    if isinstance(value, Mapping):
        for key, item in value.items():
            _require_json_finite(item, f"{field}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _require_json_finite(item, f"{field}[{index}]")


def _canonical_json_bytes(payload: Mapping[str, object]) -> bytes:
    _require_json_finite(payload)
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


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


def _sha256(path: str | Path) -> str:
    return artifact_sha256(path)


def _read_json_object(path: str | Path) -> dict[str, object]:
    try:
        payload = json.loads(
            Path(path).read_text(encoding="utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (OSError, json.JSONDecodeError, ValueError) as error:
        raise ValueError(f"could not read finite JSON object: {path}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    _require_json_finite(payload, Path(path).name)
    return payload


def _v2_raw_policy_block_from_payload(
    payload: Mapping[str, object],
) -> dict[str, object]:
    """Return the parsed raw ``trajectory_policy_v2`` block unchanged."""

    block = payload.get("trajectory_policy_v2")
    if not isinstance(block, Mapping):
        raise ValueError("V2 raw config is missing trajectory_policy_v2")
    normalized = _jsonable(dict(block))
    if not isinstance(normalized, dict):
        raise ValueError("V2 raw trajectory_policy_v2 block is malformed")
    return normalized


def _v2_override_metadata(
    metadata: Mapping[str, object] | None,
    *,
    allow_absent: bool = True,
) -> dict[str, object]:
    """Normalize explicit V2 override markers and values.

    The marker map is deliberately separate from the value map.  A numeric
    value equal to a raw-config value is still an explicit override when its
    marker is true, while a parser default is not an override when its marker
    is false.
    """

    default_markers = {
        name: False for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES
    }
    if metadata is None:
        if not allow_absent:
            raise ValueError("V2 override metadata is required")
        return {"override_markers": default_markers, "override_values": {}}
    if not isinstance(metadata, Mapping):
        raise ValueError("V2 override metadata must be an object")
    marker_value = metadata.get("override_markers", metadata.get("markers"))
    values_value = metadata.get("override_values", metadata.get("values"))
    if not isinstance(marker_value, Mapping):
        raise ValueError("V2 override_markers must be an object")
    if not isinstance(values_value, Mapping):
        raise ValueError("V2 override_values must be an object")
    expected_names = set(TRAJECTORY_POLICY_V2_PARAMETER_NAMES)
    if set(marker_value) != expected_names:
        raise ValueError("V2 override_markers must contain exactly the frozen names")
    markers: dict[str, bool] = {}
    for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
        value = marker_value[name]
        if not isinstance(value, (bool, np.bool_)):
            raise ValueError(f"V2 override_markers.{name} must be boolean")
        markers[name] = bool(value)
    expected_override_names = {
        name for name, marked in markers.items() if marked
    }
    if set(values_value) != expected_override_names:
        raise ValueError(
            "V2 override_values must contain exactly the explicitly overridden names"
        )
    values: dict[str, float] = {}
    for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
        if not markers[name]:
            continue
        value = values_value[name]
        if isinstance(value, (bool, np.bool_)):
            raise ValueError(f"V2 override_values.{name} must be finite numeric")
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"V2 override_values.{name} must be finite numeric") from error
        if not math.isfinite(numeric):
            raise ValueError(f"V2 override_values.{name} must be finite numeric")
        values[name] = numeric
    return {"override_markers": markers, "override_values": values}


def load_harness_config(
    path: str | Path,
    *,
    scene_root: str | Path | None = None,
    contract_path: str | Path | None = None,
    occupancy_path: str | Path | None = None,
    fixture_path: str | Path | None = None,
    record_video: bool = True,
    seed: int = 3072,
    policy_variant: str = DEFAULT_POLICY_VARIANT,
    lookahead_m: float | None = None,
    smoothing_window_m: float | None = None,
    curvature_preview_m: float | None = None,
    lateral_curvature_gain_1_m2: float | None = None,
    heading_curvature_gain_1_m: float | None = None,
    max_feedback_curvature_1_m: float | None = None,
    yaw_rate_feedback_gain_s: float | None = None,
    latency_preview_s: float | None = None,
    auxiliary_latency_s: float | None = None,
    maximum_delay_distance_m: float | None = None,
    profile_window_m: float | None = None,
    profile_blend_start_1_m: float | None = None,
    profile_blend_full_1_m: float | None = None,
    feedback_lookahead_m: float | None = None,
    maximum_feedback_curvature_1_m: float | None = None,
    supported_main_command_rad_s: float | None = None,
    episode_budget_s: float | None = None,
    terminal_brake_buffer_m: float | None = None,
    auxiliary_target_rate_deg_s: float | None = None,
    auxiliary_velocity_limit_rad_s: float | None = None,
) -> dict[str, object]:
    """Load and freeze the fixed case configuration plus runtime paths."""

    config_path = Path(path).expanduser().resolve()
    payload = _read_json_object(config_path)
    # This validates the exact eight-case ordering and all fixed case fields.
    load_tracking_cases(config_path)
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError("seed must be a non-negative integer")
    variant = _validate_policy_variant(policy_variant)
    supplied_parameters: dict[str, object] = {
        "lookahead_m": lookahead_m,
        "smoothing_window_m": smoothing_window_m,
        "curvature_preview_m": curvature_preview_m,
        "lateral_curvature_gain_1_m2": lateral_curvature_gain_1_m2,
        "heading_curvature_gain_1_m": heading_curvature_gain_1_m,
        "max_feedback_curvature_1_m": max_feedback_curvature_1_m,
        "yaw_rate_feedback_gain_s": yaw_rate_feedback_gain_s,
        "latency_preview_s": latency_preview_s,
        "auxiliary_latency_s": auxiliary_latency_s,
        "maximum_delay_distance_m": maximum_delay_distance_m,
        "profile_window_m": profile_window_m,
        "profile_blend_start_1_m": profile_blend_start_1_m,
        "profile_blend_full_1_m": profile_blend_full_1_m,
        "feedback_lookahead_m": feedback_lookahead_m,
        "maximum_feedback_curvature_1_m": maximum_feedback_curvature_1_m,
        "supported_main_command_rad_s": supported_main_command_rad_s,
        "episode_budget_s": episode_budget_s,
        "terminal_brake_buffer_m": terminal_brake_buffer_m,
    }
    originally_supplied_parameters = dict(supplied_parameters)
    if variant == TRAJECTORY_POLICY_V2_VARIANT:
        # The selected raw file is authoritative by default.  A direct
        # function argument is the explicit override equivalent to a CLI
        # flag; ``None`` leaves that parameter at the frozen config value.
        authority = _v2_parameters_from_payload(payload)
        for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
            if supplied_parameters[name] is None:
                supplied_parameters[name] = authority[name]
    resolved_parameters = _resolve_policy_parameters(variant, supplied_parameters)
    resolved_scene_root = (
        Path(scene_root).expanduser().resolve()
        if scene_root is not None
        else DEFAULT_SCENE_ROOT.resolve()
    )
    resolved_contract = (
        Path(contract_path).expanduser().resolve()
        if contract_path is not None
        else DEFAULT_CONTRACT.resolve()
    )
    resolved_fixture = (
        Path(fixture_path).expanduser().resolve()
        if fixture_path is not None
        else (resolved_scene_root / DEFAULT_FIXTURE.name).resolve()
    )
    resolved_occupancy = (
        Path(occupancy_path).expanduser().resolve()
        if occupancy_path is not None
        else DEFAULT_OCCUPANCY.resolve()
    )
    result = dict(payload)
    result["policy_variant"] = variant
    result.update(resolved_parameters)
    if any(
        value is not None
        for value in (
            auxiliary_target_rate_deg_s,
            auxiliary_velocity_limit_rad_s,
        )
    ):
        raw_controller = result.get("controller", {})
        if not isinstance(raw_controller, Mapping):
            raise ValueError("controller configuration must be an object")
        controller = dict(raw_controller)
        if auxiliary_target_rate_deg_s is not None:
            controller["auxiliary_target_rate_limit_rad_s"] = math.radians(
                _validate_v2_scalar(
                    auxiliary_target_rate_deg_s,
                    name="auxiliary_target_rate_deg_s",
                    strictly_positive=True,
                )
            )
        if auxiliary_velocity_limit_rad_s is not None:
            controller["auxiliary_velocity_limit_rad_s"] = _validate_v2_scalar(
                auxiliary_velocity_limit_rad_s,
                name="auxiliary_velocity_limit_rad_s",
                strictly_positive=True,
            )
        result["controller"] = controller
    if variant in (TRAJECTORY_POLICY_VARIANT, TRAJECTORY_POLICY_V2_VARIANT):
        result["trajectory_policy"] = {
            "name": variant,
            "parameters": dict(resolved_parameters),
        }
    result["harness"] = {
        "config_path": str(config_path),
        "scene_root": str(resolved_scene_root),
        "fixture_path": str(resolved_fixture),
        "occupancy_path": str(resolved_occupancy),
        "contract_path": str(resolved_contract),
        "drive_profile_path": str(DEFAULT_DRIVE_PROFILE.resolve()),
        "record_video": bool(record_video),
        "seed": int(seed),
    }
    if variant == TRAJECTORY_POLICY_V2_VARIANT:
        markers = {
            name: originally_supplied_parameters.get(name) is not None
            for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES
        }
        metadata = _v2_override_metadata(
            {
                "override_markers": markers,
                "override_values": {
                    name: resolved_parameters[name]
                    for name, marked in markers.items()
                    if bool(marked)
                },
            },
            allow_absent=False,
        )
        result["trajectory_policy_v2_provenance"] = {
            "config_path": str(config_path),
            "raw_policy": _v2_raw_policy_block_from_payload(payload),
            **metadata,
        }
    return result


def validate_diagnostic_fixture(
    fixture_path: str | Path,
    occupancy_path: str | Path,
) -> dict[str, object]:
    """Validate the dedicated open-plane scene and all-free occupancy map."""

    fixture = Path(fixture_path).expanduser().resolve()
    occupancy = Path(occupancy_path).expanduser().resolve()
    if not fixture.is_file():
        raise FileNotFoundError(f"diagnostic fixture does not exist: {fixture}")
    if not occupancy.is_file():
        raise FileNotFoundError(f"diagnostic occupancy does not exist: {occupancy}")
    try:
        scene_text = fixture.read_text(encoding="utf-8")
    except OSError as error:
        raise DiagnosticValidationError("could not read diagnostic fixture") from error
    forbidden_tokens = ("CollisionAPI", "UsdPhysics", "defaultGroundPlane")
    if any(token in scene_text for token in forbidden_tokens):
        raise DiagnosticValidationError(
            "diagnostic open-plane fixture must not define collision geometry or ground"
        )
    try:
        with np.load(occupancy, allow_pickle=False) as archive:
            required = {"grid", "global_origin_xy", "resolution_m"}
            missing = required.difference(archive.files)
            if missing:
                raise DiagnosticValidationError(
                    "diagnostic occupancy is missing " + ", ".join(sorted(missing))
                )
            grid = np.asarray(archive["grid"])
            origin = np.asarray(archive["global_origin_xy"], dtype=np.float64)
            resolution = np.asarray(archive["resolution_m"], dtype=np.float64)
    except DiagnosticValidationError:
        raise
    except (OSError, ValueError) as error:
        raise DiagnosticValidationError("could not read diagnostic occupancy") from error
    if grid.ndim != 2 or grid.size == 0 or grid.dtype != np.uint8:
        raise DiagnosticValidationError("diagnostic occupancy grid must be non-empty uint8")
    if not np.all(grid == 0):
        raise DiagnosticValidationError("diagnostic occupancy must be all-free")
    if origin.shape != (2,) or not np.all(np.isfinite(origin)):
        raise DiagnosticValidationError("diagnostic occupancy origin must be finite XY")
    if resolution.size != 1 or not np.isfinite(resolution.reshape(-1)[0]) or resolution.reshape(-1)[0] <= 0.0:
        raise DiagnosticValidationError("diagnostic occupancy resolution must be positive")
    resolution_value = float(resolution.reshape(-1)[0])
    return {
        "scene_path": str(fixture),
        "scene_sha256": _sha256(fixture),
        "occupancy_path": str(occupancy),
        "occupancy_sha256": _sha256(occupancy),
        "grid_shape": [int(item) for item in grid.shape],
        "global_origin_xy": origin.tolist(),
        "resolution_m": resolution_value,
    }


def _harness_paths(harness_config: Mapping[str, object]) -> dict[str, Path | int | bool]:
    nested = harness_config.get("harness", {})
    if not isinstance(nested, Mapping):
        nested = {}
    scene_root = Path(nested.get("scene_root", DEFAULT_SCENE_ROOT)).expanduser().resolve()
    fixture = Path(nested.get("fixture_path", scene_root / DEFAULT_FIXTURE.name)).expanduser().resolve()
    occupancy = Path(nested.get("occupancy_path", DEFAULT_OCCUPANCY)).expanduser().resolve()
    contract = Path(nested.get("contract_path", DEFAULT_CONTRACT)).expanduser().resolve()
    drive_profile = Path(nested.get("drive_profile_path", DEFAULT_DRIVE_PROFILE)).expanduser().resolve()
    return {
        "scene_root": scene_root,
        "fixture_path": fixture,
        "occupancy_path": occupancy,
        "contract_path": contract,
        "drive_profile_path": drive_profile,
        "record_video": bool(nested.get("record_video", True)),
        "seed": int(nested.get("seed", 3072)),
    }


def _case_by_id(config_path: str | Path, case_id: str) -> TrackingCase:
    cases = load_tracking_cases(config_path)
    matches = [case for case in cases if case.case_id == case_id]
    if len(matches) != 1:
        raise ValueError(f"unknown tracking case: {case_id!r}")
    return matches[0]


def _relative_scene_uri(scene_root: Path, fixture_path: Path) -> str:
    try:
        relative = fixture_path.resolve().relative_to(scene_root.resolve())
    except ValueError as error:
        raise ValueError("tracking fixture must reside below scene_root") from error
    if not relative.parts:
        raise ValueError("tracking fixture path must name a file")
    return "${SCENE_ROOT}/" + relative.as_posix()


def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half = float(yaw_rad) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def build_tracking_episode(
    case: TrackingCase,
    reference: TrackingReference,
    *,
    scene_root: str | Path,
    fixture_path: str | Path,
    occupancy_path: str | Path,
    seed: int = 3072,
) -> BenchmarkEpisode:
    """Construct one manifest-compatible episode from geometry and fixture hashes."""

    scene_root_path = Path(scene_root).expanduser().resolve()
    fixture = Path(fixture_path).expanduser().resolve()
    occupancy = Path(occupancy_path).expanduser().resolve()
    if not fixture.is_file() or not occupancy.is_file():
        raise FileNotFoundError("tracking fixture and occupancy files are required")
    start_xy = reference.points_xy[0]
    goal_xy = reference.points_xy[-1]
    start_yaw = float(reference.yaw_rad[0])
    euclidean = float(np.linalg.norm(goal_xy - start_xy))
    return BenchmarkEpisode(
        episode_id=f"tracking_acceptance_{case.case_id}",
        split="dev",
        difficulty="E0",
        scene_uri=_relative_scene_uri(scene_root_path, fixture),
        scene_sha256=_sha256(fixture),
        occupancy_sha256=_sha256(occupancy),
        start_position_world_m=(float(start_xy[0]), float(start_xy[1]), 0.45),
        start_quaternion_world_wxyz=_yaw_quaternion(start_yaw),
        goal_position_world_m=(float(goal_xy[0]), float(goal_xy[1]), 0.45),
        euclidean_distance_m=max(euclidean, np.finfo(np.float64).eps),
        oracle_path_length_m=max(reference.path_length_m, euclidean),
        seed=int(seed),
        human_review=HumanReview("approved", "tracking_acceptance", "diagnostic fixture"),
        classification_evidence={
            "tracking_acceptance": True,
            "case_id": case.case_id,
            "fixture": "all_free_open_plane",
        },
    )


def _trace(result: object, name: str, *, ndim: int | None = None) -> np.ndarray:
    if not hasattr(result, name):
        raise DiagnosticValidationError(f"diagnostic result is missing {name}")
    value = getattr(result, name)
    if value is None:
        raise DiagnosticValidationError(f"diagnostic result {name} is unavailable")
    try:
        array = np.asarray(value)
    except (TypeError, ValueError) as error:
        raise DiagnosticValidationError(f"diagnostic result {name} is malformed") from error
    if array.size == 0 or (ndim is not None and array.ndim != ndim):
        raise DiagnosticValidationError(f"diagnostic result {name} is malformed")
    if not np.all(np.isfinite(array)):
        raise DiagnosticValidationError(f"diagnostic result {name} must be finite")
    return array


_PHYSICS_DIAGNOSTIC_SHAPES: dict[str, tuple[int, ...]] = {
    "raw_command_target": (2,),
    "clipped_target": (2,),
    "smoothed_target": (2,),
    "applied_action": (2,),
    "joint_position_rad": (3,),
    "joint_velocity_rad_s": (3,),
    "position_world_xy": (2,),
    "root_position_world_m": (3,),
    "root_quaternion_world_wxyz": (4,),
    "linear_velocity_world_m_s": (3,),
    "angular_velocity_world_rad_s": (3,),
    "yaw_world_rad": (),
    "roll_world_rad": (),
    "simulation_time_s": (),
    "physics_step_index": (),
    "raw_simulation_time_s": (),
    "raw_physics_step_index": (),
    "action_block_index": (),
    "physics_step_in_block": (),
}
_PHYSICS_DIAGNOSTIC_INTEGER_FIELDS = frozenset(
    {"physics_step_index", "raw_physics_step_index", "action_block_index", "physics_step_in_block"}
)


def _validated_physics_diagnostics(
    runtime: object, result: object | None = None
) -> dict[str, np.ndarray]:
    """Return a finite, result-aligned copy of the runtime physics trace."""

    try:
        diagnostics = getattr(runtime, "physics_diagnostics")
    except Exception as error:
        raise DiagnosticValidationError(
            "runtime physics_diagnostics are unavailable"
        ) from error
    if not isinstance(diagnostics, Mapping) or not diagnostics:
        raise DiagnosticValidationError(
            "runtime physics_diagnostics must be a non-empty mapping"
        )
    names = tuple(diagnostics)
    if any(not isinstance(name, str) or not name for name in names):
        raise DiagnosticValidationError(
            "runtime physics_diagnostics field names must be non-empty strings"
        )
    expected_names = set(_PHYSICS_DIAGNOSTIC_SHAPES)
    legacy_names = expected_names - {"smoothed_target"}
    actual_names = set(names)
    if actual_names == legacy_names:
        diagnostics = dict(diagnostics)
        diagnostics["smoothed_target"] = np.asarray(
            diagnostics["applied_action"]
        ).copy()
        names = tuple(diagnostics)
        actual_names = set(names)
    elif actual_names != expected_names:
        missing = sorted(expected_names - actual_names)
        extra = sorted(actual_names - expected_names)
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if extra:
            details.append("extra=" + ",".join(extra))
        raise DiagnosticValidationError(
                "runtime physics_diagnostics fields must match the exact 18-field legacy "
                "or 19-field Round 6 "
            "contract (" + "; ".join(details) + ")"
        )

    validated: dict[str, np.ndarray] = {}
    sample_count: int | None = None
    for name, value in diagnostics.items():
        if not isinstance(name, str) or not name:
            raise DiagnosticValidationError(
                "runtime physics_diagnostics field names must be non-empty strings"
            )
        try:
            array = np.asarray(value)
        except (TypeError, ValueError) as error:
            raise DiagnosticValidationError(
                f"runtime physics_diagnostics field {name} is malformed"
            ) from error
        if array.ndim < 1 or array.size == 0:
            raise DiagnosticValidationError(
                f"runtime physics_diagnostics field {name} must be a non-empty array"
            )
        if array.dtype.kind not in "biuf":
            raise DiagnosticValidationError(
                f"runtime physics_diagnostics field {name} must be real numeric"
            )
        if not np.all(np.isfinite(array)):
            raise DiagnosticValidationError(
                f"runtime physics_diagnostics field {name} must be finite"
            )
        if sample_count is None:
            sample_count = int(array.shape[0])
        elif array.shape[0] != sample_count:
            raise DiagnosticValidationError(
                "runtime physics_diagnostics fields must share the same sample count"
            )
        expected_tail = _PHYSICS_DIAGNOSTIC_SHAPES.get(name)
        if expected_tail is not None and array.shape[1:] != expected_tail:
            raise DiagnosticValidationError(
                f"runtime physics_diagnostics field {name} must have shape "
                f"(N, {', '.join(str(item) for item in expected_tail)})"
            )
        if name in _PHYSICS_DIAGNOSTIC_INTEGER_FIELDS:
            if not np.all(np.equal(array, np.rint(array))):
                raise DiagnosticValidationError(
                    f"runtime physics_diagnostics field {name} must be integer-valued"
                )
        validated[name] = array.copy()

    if result is not None:
        actions = _trace(result, "actions", ndim=2)
        positions = _trace(result, "positions_world_xy", ndim=2)
        action_count = int(actions.shape[0])
        if positions.shape[1] != 2 or positions.shape[0] != action_count + 1:
            raise DiagnosticValidationError(
                "state/action traces are not state-aligned for physics diagnostics"
            )
        expected_count = 1 + 5 * action_count
        if sample_count != expected_count:
            raise DiagnosticValidationError(
                "physics diagnostic sample count must equal 1 + 5 * action blocks"
            )
        expected_steps = np.arange(expected_count, dtype=np.int64)
        if not np.array_equal(validated["physics_step_index"], expected_steps):
            raise DiagnosticValidationError(
                "physics_step_index must be relative [0, 1, ..., 5N]"
            )
        expected_blocks = np.concatenate(
            (np.asarray((0,), dtype=np.int64), np.repeat(np.arange(action_count), 5))
        )
        if not np.array_equal(validated["action_block_index"], expected_blocks):
            raise DiagnosticValidationError(
                "action_block_index must match reset row plus five physics rows per block"
            )
        expected_steps_in_block = np.concatenate(
            (np.asarray((0,), dtype=np.int64), np.tile(np.arange(1, 6), action_count))
        )
        if not np.array_equal(
            validated["physics_step_in_block"], expected_steps_in_block
        ):
            raise DiagnosticValidationError(
                "physics_step_in_block must be 0 then 1..5 for each block"
            )
        expected_dt = 1.0 / 50.0
        relative_time = validated["simulation_time_s"].astype(np.float64)
        raw_time = validated["raw_simulation_time_s"].astype(np.float64)
        raw_steps = validated["raw_physics_step_index"]
        if not np.allclose(
            relative_time, np.arange(expected_count, dtype=np.float64) * expected_dt,
            rtol=0.0,
            atol=CLOCK_TIME_TOLERANCE_S,
        ):
            raise DiagnosticValidationError(
                "simulation_time_s must start at zero and increment at 50 Hz"
            )
        if not np.all(np.diff(raw_steps) == 1):
            raise DiagnosticValidationError(
                "raw_physics_step_index must increment by one per physics step"
            )
        if not np.all(np.abs(np.diff(raw_time) - expected_dt) <= CLOCK_TIME_TOLERANCE_S):
            raise DiagnosticValidationError(
                "raw_simulation_time_s must increment at 50 Hz"
            )
        if not np.allclose(
            validated["position_world_xy"][0], positions[0], rtol=0.0, atol=1e-5
        ):
            raise DiagnosticValidationError(
                "physics diagnostic state-0 position does not match result state-0"
            )
        for name in ("raw_command_target", "clipped_target", "applied_action"):
            if not np.all(validated[name][0] == 0.0):
                raise DiagnosticValidationError(
                    f"physics diagnostic state-0 {name} must be zero"
                )
    return validated


def validate_diagnostic_timing(
    result: object,
    *,
    replan_hz: int = 10,
    expected_physics_steps_per_block: int = 5,
    time_tolerance_s: float = CLOCK_TIME_TOLERANCE_S,
) -> None:
    """Fail closed unless measured raw/relative clocks align state-to-action."""

    if isinstance(replan_hz, bool) or not isinstance(replan_hz, int) or replan_hz <= 0:
        raise ValueError("replan_hz must be a positive integer")
    expected_dt = 1.0 / float(replan_hz)
    relative_time = _trace(result, "simulation_time_s", ndim=1).astype(np.float64)
    relative_step = _trace(result, "physics_step_index", ndim=1).astype(np.float64)
    raw_time = _trace(result, "raw_simulation_time_s", ndim=1).astype(np.float64)
    raw_step = _trace(result, "raw_physics_step_index", ndim=1).astype(np.float64)
    positions = _trace(result, "positions_world_xy", ndim=2)
    actions = _trace(result, "actions", ndim=2)
    if positions.shape[1] != 2:
        raise DiagnosticValidationError("positions_world_xy must have shape (N, 2)")
    if positions.shape[0] != actions.shape[0] + 1:
        raise DiagnosticValidationError("state/action traces are not state-aligned")
    if not (relative_time.shape == relative_step.shape == raw_time.shape == raw_step.shape == positions.shape[:1]):
        raise DiagnosticValidationError("clock traces are not state-aligned")
    if not np.all(np.equal(relative_step, np.rint(relative_step))) or not np.all(
        np.equal(raw_step, np.rint(raw_step))
    ):
        raise DiagnosticValidationError("physics step traces must be integer-valued")
    if relative_time[0] != 0.0 or relative_step[0] != 0.0:
        raise DiagnosticValidationError("relative clock must start at (0.0, 0)")
    relative_step_delta = np.diff(relative_step)
    raw_step_delta = np.diff(raw_step)
    if not np.all(relative_step_delta == expected_physics_steps_per_block) or not np.all(
        raw_step_delta == expected_physics_steps_per_block
    ):
        raise DiagnosticValidationError("physics_step_index must increment by five per action block")
    relative_time_delta = np.diff(relative_time)
    raw_time_delta = np.diff(raw_time)
    if not np.all(np.abs(relative_time_delta - expected_dt) <= time_tolerance_s):
        raise DiagnosticValidationError("simulation_time_s increments are not 10 Hz")
    if not np.all(np.abs(raw_time_delta - expected_dt) <= time_tolerance_s):
        raise DiagnosticValidationError("raw_simulation_time_s increments are not 10 Hz")
    if not np.allclose(relative_time, raw_time - raw_time[0], rtol=0.0, atol=time_tolerance_s):
        raise DiagnosticValidationError("relative simulation_time_s is not raw-clock aligned")
    if not np.array_equal(relative_step, raw_step - raw_step[0]):
        raise DiagnosticValidationError("relative physics_step_index is not raw-clock aligned")


def _direction_code(direction: str) -> int:
    return 1 if direction == "forward" else -1


def _main_commands(actions: object) -> np.ndarray:
    array = _action_array(actions)
    if array.ndim == 2 and array.shape[1] == 2:
        return array[:, 0]
    if array.ndim == 3 and array.shape[1:] == (5, 2):
        return array[:, 0, 0]
    if array.ndim == 2 and array.shape[1] == 10:
        return array[:, 0]
    raise DiagnosticValidationError("action commands must have shape (B, 2), (B, 5, 2), or (B, 10)")


def _action_array(actions: object) -> np.ndarray:
    try:
        array = np.asarray(actions, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise DiagnosticValidationError("action commands are malformed") from error
    valid_shape = (
        (array.ndim == 2 and array.shape[1] in (2, 10))
        or (array.ndim == 3 and array.shape[1:] == (5, 2))
    )
    if not valid_shape or array.shape[0] == 0 or not np.all(np.isfinite(array)):
        raise DiagnosticValidationError("action commands must be finite and state-aligned")
    return array


def _zero_action_blocks(actions: object) -> np.ndarray:
    array = _action_array(actions)
    if array.ndim == 3:
        return np.all(np.abs(array) <= 1e-8, axis=(1, 2))
    return np.all(np.abs(array) <= 1e-8, axis=1)


def validate_direction_boundaries(
    reference: TrackingReference,
    *,
    positions_world_xy: object,
    speed_m_s: object,
    active_direction_codes: object,
    actions: object | None = None,
    desired_direction_codes: object | None = None,
    policy_reference_index: object | None = None,
    boundary_tolerance_m: float = BOUNDARY_TOLERANCE_M,
    stop_speed_m_s: float = 0.05,
    required_stop_samples: int = 5,
) -> dict[str, object]:
    """Require physical cusp proximity, a measured stop, and real direction change.

    Policy reference indices are retained only as descriptive evidence.  They
    never satisfy boundary progression by themselves.
    """

    if not isinstance(reference, TrackingReference):
        raise TypeError("reference must be a TrackingReference")
    if not reference.stop_markers:
        return {"required": False, "all_passed": True, "boundaries": []}
    positions = _finite_trace_array(positions_world_xy, "positions_world_xy", ndim=2)
    if positions.shape[1] != 2:
        raise DiagnosticValidationError("positions_world_xy must have shape (N, 2)")
    speeds = _finite_trace_array(speed_m_s, "speed_m_s", ndim=1)
    if speeds.shape != (positions.shape[0],):
        raise DiagnosticValidationError("speed_m_s must align with positions_world_xy")
    active = _finite_trace_array(active_direction_codes, "active_direction_codes", ndim=1)
    if active.size not in (positions.shape[0], positions.shape[0] - 1):
        raise DiagnosticValidationError("active_direction_codes must align with states or actions")
    if not np.all(np.isin(active, (-1, 0, 1))):
        raise DiagnosticValidationError("active_direction_codes must use -1, 0, or 1")
    if actions is None:
        raise DiagnosticValidationError("direction evidence requires actual signed commands")
    commands = _main_commands(actions)
    zero_blocks = _zero_action_blocks(actions)
    if commands.shape[0] != positions.shape[0] - 1:
        raise DiagnosticValidationError("signed commands must align with action blocks")
    if zero_blocks.shape != commands.shape:
        raise DiagnosticValidationError("zero-command blocks must align with actions")
    desired = None
    if desired_direction_codes is None:
        raise DiagnosticValidationError(
            "direction evidence requires active and desired direction traces"
        )
    desired = _finite_trace_array(
        desired_direction_codes, "desired_direction_codes", ndim=1
    )
    if desired.size not in (positions.shape[0], commands.shape[0]) or not np.all(
        np.isin(desired, (-1, 0, 1))
    ):
        raise DiagnosticValidationError(
            "desired_direction_codes must align with states or actions"
        )
    if not math.isfinite(float(boundary_tolerance_m)) or boundary_tolerance_m <= 0:
        raise ValueError("boundary_tolerance_m must be finite and positive")
    if not math.isfinite(float(stop_speed_m_s)) or stop_speed_m_s <= 0:
        raise ValueError("stop_speed_m_s must be finite and positive")
    if required_stop_samples < 1:
        raise ValueError("required_stop_samples must be positive")

    active_is_state_aligned = active.size == positions.shape[0]
    desired_is_state_aligned = desired.size == positions.shape[0]

    def local_transition(
        codes: np.ndarray,
        state_aligned: bool,
        command_index: int,
        previous: int,
        next_code: int,
    ) -> bool:
        if state_aligned:
            return bool(
                command_index + 1 < codes.size
                and codes[command_index] == previous
                and codes[command_index + 1] == next_code
            )
        return bool(
            command_index >= 1
            and codes[command_index - 1] == previous
            and codes[command_index] == next_code
        )

    def ordered_leading_transition(
        codes: np.ndarray,
        state_aligned: bool,
        command_index: int,
        zero_start: int,
        previous: int,
        next_code: int,
    ) -> bool:
        """Accept a desired-direction lead only inside this brake episode."""

        if state_aligned:
            previous_index = zero_start
            window_start = zero_start + 1
            window_end = command_index + 2
        else:
            previous_index = zero_start - 1
            window_start = zero_start
            window_end = command_index + 1
        if previous_index < 0 or window_end > codes.size:
            return False
        window = codes[window_start:window_end]
        if window.size == 0 or codes[previous_index] != previous:
            return False
        next_indices = np.flatnonzero(window == next_code)
        if next_indices.size == 0:
            return False
        first_next = int(next_indices[0])
        return bool(
            np.all(window[:first_next] == previous)
            and np.all(window[first_next:] == next_code)
        )

    evidence: list[dict[str, object]] = []
    previous_boundary_index = -1
    previous_transition_command_index = -1
    for marker in reference.stop_markers:
        previous_direction = _direction_code(marker.drive_direction)
        next_direction = _direction_code(reference.segment_directions[marker.segment_index + 1])
        boundary = np.asarray(marker.xy, dtype=np.float64)
        distances = np.linalg.norm(positions - boundary[None, :], axis=1)
        candidate_indices = np.flatnonzero(
            (distances <= float(boundary_tolerance_m) + 1e-12)
            & (np.arange(positions.shape[0]) > previous_boundary_index)
        )
        reached = candidate_indices.size > 0
        boundary_index = int(candidate_indices[0]) if reached else -1
        boundary_distance = float(np.min(distances))

        command_lower_bound = max(
            boundary_index,
            previous_transition_command_index + 1,
            0,
        )
        command_candidates = np.flatnonzero(
            (np.arange(commands.shape[0]) >= command_lower_bound)
            & (np.sign(commands) == next_direction)
            & (np.abs(commands) > 1e-8)
        )
        command_index = int(command_candidates[0]) if command_candidates.size else -1
        stop_start = command_index - int(required_stop_samples)
        stop_command_indices = (
            np.arange(stop_start, command_index, dtype=np.int64)
            if command_index >= required_stop_samples
            else np.empty(0, dtype=np.int64)
        )
        stop_state_indices = stop_command_indices + 1
        stop_window = (
            speeds[stop_state_indices]
            if stop_state_indices.size == required_stop_samples
            else np.empty(0, dtype=np.float64)
        )
        stop_distances = (
            distances[stop_state_indices]
            if stop_state_indices.size == required_stop_samples
            else np.empty(0, dtype=np.float64)
        )
        zero_window = (
            zero_blocks[stop_command_indices]
            if stop_command_indices.size == required_stop_samples
            else np.empty(0, dtype=bool)
        )
        braking_start = command_index
        while (
            braking_start > previous_transition_command_index + 1
            and braking_start > 0
            and zero_blocks[braking_start - 1]
        ):
            braking_start -= 1
        previous_command_candidates = np.flatnonzero(
            (np.arange(commands.shape[0]) < max(stop_start, 0))
            & (np.arange(commands.shape[0]) > previous_transition_command_index)
            & (np.sign(commands) == previous_direction)
            & (np.abs(commands) > 1e-8)
        )
        previous_command_index = (
            int(previous_command_candidates[-1])
            if previous_command_candidates.size
            else -1
        )
        stop_passed = bool(
            command_index >= required_stop_samples
            and stop_window.shape[0] == required_stop_samples
            and zero_window.shape[0] == required_stop_samples
            and np.all(zero_window)
            and np.all(stop_window < float(stop_speed_m_s))
            and np.all(stop_distances <= float(boundary_tolerance_m) + 1e-12)
        )
        active_changed = bool(
            command_index >= 0
            and local_transition(
                active,
                active_is_state_aligned,
                command_index,
                previous_direction,
                next_direction,
            )
        )
        desired_changed = bool(
            command_index >= 0
            and ordered_leading_transition(
                desired,
                desired_is_state_aligned,
                command_index,
                braking_start,
                previous_direction,
                next_direction,
            )
        )
        command_sign_ok = bool(
            command_index >= 0 and previous_command_index >= 0
        )
        item = {
            "segment_index": int(marker.segment_index),
            "point_index": int(marker.point_index),
            "boundary_distance_m": boundary_distance,
            "boundary_state_index": boundary_index,
            "boundary_reached": reached,
            "opposite_command_index": command_index,
            "opposite_command_seen": command_sign_ok,
            "previous_command_index": previous_command_index,
            "braking_start_command_index": braking_start,
            "zero_command_indices": stop_command_indices.tolist(),
            "stop_state_indices": stop_state_indices.tolist(),
            "stop_sample_count": int(stop_window.shape[0]),
            "stop_hold_passed": stop_passed,
            "active_direction_changed": active_changed,
            "desired_direction_changed": desired_changed,
            "passed": bool(reached and command_sign_ok and stop_passed and active_changed and desired_changed),
        }
        evidence.append(item)
        if reached:
            previous_boundary_index = boundary_index
        if command_index >= 0:
            previous_transition_command_index = command_index
    return {
        "required": True,
        "all_passed": bool(evidence and all(bool(item["passed"]) for item in evidence)),
        "boundaries": evidence,
        "reference_indices_descriptive": (
            [] if policy_reference_index is None else np.asarray(policy_reference_index).astype(int).tolist()
        ),
    }


def _finite_trace_array(value: object, name: str, *, ndim: int) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise DiagnosticValidationError(f"{name} is malformed") from error
    if array.ndim != ndim or array.size == 0 or not np.all(np.isfinite(array)):
        raise DiagnosticValidationError(f"{name} must be finite with ndim={ndim}")
    return array


def _reference_yaw_for_states(positions: np.ndarray, reference: TrackingReference) -> np.ndarray:
    ref = np.asarray(reference.points_xy, dtype=np.float64)
    yaw = np.asarray(reference.yaw_rad, dtype=np.float64)
    starts = ref[:-1]
    vectors = ref[1:] - starts
    lengths_sq = np.einsum("ij,ij->i", vectors, vectors)
    delta = positions[:, None, :] - starts[None, :, :]
    fraction = np.divide(
        np.einsum("nsi,si->ns", delta, vectors),
        lengths_sq[None, :],
        out=np.zeros((positions.shape[0], starts.shape[0]), dtype=np.float64),
        where=lengths_sq[None, :] > 1e-24,
    )
    fraction = np.clip(fraction, 0.0, 1.0)
    projection = starts[None, :, :] + fraction[:, :, None] * vectors[None, :, :]
    segment = np.argmin(np.linalg.norm(positions[:, None, :] - projection, axis=2), axis=1)
    next_indices = np.minimum(segment + 1, yaw.shape[0] - 1)
    delta_yaw = (yaw[next_indices] - yaw[segment] + math.pi) % (2 * math.pi) - math.pi
    return (yaw[segment] + fraction[np.arange(positions.shape[0]), segment] * delta_yaw + math.pi) % (2 * math.pi) - math.pi


def _speed_trace(result: object) -> np.ndarray:
    velocity = _trace(result, "linear_velocity_world_m_s", ndim=2).astype(np.float64)
    if velocity.shape[1] < 2:
        raise DiagnosticValidationError("linear_velocity_world_m_s must have at least XY components")
    return np.linalg.norm(velocity[:, :2], axis=1)


def _independent_stop_hold(
    result: object,
    reference: TrackingReference,
    *,
    goal_tolerance_m: float,
    stop_speed_m_s: float,
    stop_hold_s: float,
    settling_budget_s: float,
    replan_hz: int,
) -> dict[str, object]:
    positions = _trace(result, "positions_world_xy", ndim=2).astype(np.float64)
    speeds = _speed_trace(result)
    if speeds.shape != (positions.shape[0],):
        raise DiagnosticValidationError("speed trace is not state-aligned")
    goal = np.asarray(reference.goal_xy, dtype=np.float64)
    goal_errors = np.linalg.norm(positions - goal[None, :], axis=1)
    entries = np.flatnonzero(goal_errors <= float(goal_tolerance_m) + 1e-12)
    required = max(1, int(math.ceil(float(stop_hold_s) * float(replan_hz) - 1e-12)))
    budget_blocks = int(round(float(settling_budget_s) * float(replan_hz)))
    if entries.size == 0:
        return {
            "goal_entry_state_index": None,
            "required_samples": required,
            "max_speed_m_s": None,
            "final_speed_m_s": None,
            "all_below_threshold": False,
            "stop_hold_passed": False,
            "speed_m_s": speeds.tolist(),
        }
    entry = int(entries[0])
    upper = min(positions.shape[0], entry + budget_blocks + 1)
    actions = _trace(result, "actions", ndim=2).astype(np.float64)
    if actions.shape[0] != positions.shape[0] - 1:
        raise DiagnosticValidationError("actions are not state-aligned for stop hold")
    qualifying = (goal_errors < float(goal_tolerance_m) + 1e-12) & (speeds < float(stop_speed_m_s))
    run = 0
    hold_indices: list[int] = []
    # The first state inside goal is the nominal event and may still be
    # moving.  A diagnostic hold consists only of subsequent state endpoints;
    # each endpoint must be produced by a genuinely zero action block.
    for index in range(entry + 1, upper):
        zero_action = bool(np.all(np.abs(actions[index - 1]) <= 1e-8))
        if qualifying[index] and zero_action:
            run += 1
            hold_indices.append(index)
            if run >= required:
                break
        else:
            run = 0
            hold_indices = []
    passed = len(hold_indices) >= required
    hold_speeds = speeds[hold_indices] if passed else speeds[entry:upper]
    return {
        "goal_entry_state_index": entry,
        "required_samples": required,
        "hold_state_indices": hold_indices,
        "hold_action_zero": bool(passed),
        "max_speed_m_s": float(np.max(hold_speeds)) if hold_speeds.size else None,
        "final_speed_m_s": float(hold_speeds[-1]) if hold_speeds.size else None,
        "all_below_threshold": bool(passed and np.all(hold_speeds < float(stop_speed_m_s))),
        "stop_hold_passed": passed,
        "speed_m_s": speeds.tolist(),
    }


def compute_tracking_metrics(
    case: TrackingCase,
    reference: TrackingReference,
    result: object,
    *,
    goal_tolerance_m: float = 0.2,
    stop_speed_m_s: float = 0.05,
    stop_hold_s: float = 0.5,
    settling_budget_s: float = 3.0,
    replan_hz: int = 10,
) -> dict[str, object]:
    """Compute descriptive traces and independent diagnostic acceptance."""

    timing_error: str | None = None
    try:
        validate_diagnostic_timing(result, replan_hz=replan_hz)
    except DiagnosticValidationError as error:
        timing_error = str(error)

    positions = _trace(result, "positions_world_xy", ndim=2).astype(np.float64)
    actual_yaw = _trace(result, "yaw_world_rad", ndim=1).astype(np.float64)
    if actual_yaw.shape != (positions.shape[0],):
        raise DiagnosticValidationError("yaw_world_rad is not state-aligned")
    cross_track = polyline_cross_track_errors(positions, reference.points_xy)
    reference_yaw = _reference_yaw_for_states(positions, reference)
    heading = heading_errors_rad(actual_yaw, reference_yaw)
    speed = _speed_trace(result)
    hold = _independent_stop_hold(
        result,
        reference,
        goal_tolerance_m=goal_tolerance_m,
        stop_speed_m_s=stop_speed_m_s,
        stop_hold_s=stop_hold_s,
        settling_budget_s=settling_budget_s,
        replan_hz=replan_hz,
    )
    boundary: dict[str, object]
    if case.kind in ("mixed", "forward_stop_reverse_stop_forward"):
        boundary = validate_direction_boundaries(
            reference,
            positions_world_xy=positions,
            speed_m_s=speed,
            active_direction_codes=_trace(result, "policy_active_drive_direction", ndim=1),
            actions=_trace(result, "actions", ndim=2),
            desired_direction_codes=_trace(result, "policy_desired_drive_direction", ndim=1),
            policy_reference_index=_trace(result, "policy_reference_index", ndim=1),
            boundary_tolerance_m=goal_tolerance_m,
            stop_speed_m_s=stop_speed_m_s,
            required_stop_samples=max(1, int(math.ceil(stop_hold_s * replan_hz - 1e-12))),
        )
    else:
        boundary = {"required": False, "all_passed": True, "boundaries": []}
    collisions = _trace(result, "collisions", ndim=1).astype(bool)
    contacts = _trace(result, "physical_contacts", ndim=1).astype(bool)
    if collisions.shape != positions.shape[:1] or contacts.shape != positions.shape[:1]:
        raise DiagnosticValidationError("collision/contact traces are not state-aligned")
    final_goal_error = float(np.linalg.norm(positions[-1] - np.asarray(reference.goal_xy)))
    nominal_success = str(getattr(result, "termination_reason", "")) == "success"
    runtime_error = bool(
        timing_error
        or getattr(result, "runtime_error", None)
        or getattr(result, "diagnostic_settling_runtime_error", None)
        or str(getattr(result, "termination_reason", "")) == "runtime_error"
    )
    collision_failure = bool(np.any(collisions) or np.any(contacts))
    acceptance = {
        "nominal_success": nominal_success,
        "goal_reached": bool(final_goal_error <= float(goal_tolerance_m) + 1e-12),
        "stop_hold": bool(hold["stop_hold_passed"]),
        "segment_boundaries": bool(boundary["all_passed"]),
        "clock": timing_error is None,
        "no_collision_or_filtered_contact": not collision_failure,
        "no_runtime_error": not runtime_error,
    }
    acceptance["complete"] = bool(all(bool(value) for value in acceptance.values()))
    if not acceptance["clock"] or not acceptance["no_runtime_error"] or not acceptance[
        "no_collision_or_filtered_contact"
    ]:
        failure_stage: str | None = "runtime"
    elif not acceptance["nominal_success"] or not acceptance["goal_reached"]:
        failure_stage = "goal"
    elif not acceptance["stop_hold"]:
        failure_stage = "stop_hold"
    elif not acceptance["segment_boundaries"]:
        failure_stage = "metric"
    else:
        failure_stage = None

    def distribution(values: np.ndarray) -> dict[str, object]:
        return {
            "mean": float(np.mean(np.abs(values))),
            "p95": float(np.percentile(np.abs(values), 95.0)),
            "max": float(np.max(np.abs(values))),
            "values": values.tolist(),
        }

    def trace_summary(value: object, component: int | None = None) -> dict[str, object]:
        array = _finite_trace_array(value, "trace", ndim=1 if component is None else 2)
        if component is not None:
            array = array[:, component]
        return {
            "mean": float(np.mean(np.abs(array))),
            "p95": float(np.percentile(np.abs(array), 95.0)),
            "max": float(np.max(np.abs(array))),
            "final": float(array[-1]),
            "values": array.tolist(),
        }

    return {
        "case_id": case.case_id,
        "probe": bool(case.probe),
        "cross_track_m": distribution(cross_track),
        "heading_error_rad": distribution(heading),
        "goal_error_m": np.linalg.norm(positions - np.asarray(reference.goal_xy), axis=1).tolist(),
        "final_goal_error_m": final_goal_error,
        "stop_hold": hold,
        "direction_boundaries": boundary,
        "actual_main_velocity_rad_s": trace_summary(_trace(result, "joint_velocity_rad_s", ndim=2), 0),
        "actual_auxiliary_angle_rad": trace_summary(_trace(result, "joint_position_rad", ndim=2), 1),
        "roll_world_rad": trace_summary(_trace(result, "roll_world_rad", ndim=1)),
        "termination_reason": getattr(result, "termination_reason", None),
        "runtime_error": getattr(result, "runtime_error", None),
        "timing_error": timing_error,
        "failure_stage": failure_stage,
        "filtered_physical_contact": bool(np.any(contacts)),
        "collision": bool(np.any(collisions)),
        "acceptance": acceptance,
    }


def _reference_arrays(reference: TrackingReference) -> dict[str, np.ndarray]:
    return {
        "reference_pose": np.column_stack((reference.points_xy, reference.yaw_rad)),
        "reference_command": np.asarray(
            [(item.main_command_rad_s, item.auxiliary_angle_rad) for item in reference.waypoints],
            dtype=np.float64,
        ),
        "reference_drive_direction": np.asarray(
            [_direction_code(item.drive_direction) for item in reference.waypoints],
            dtype=np.int8,
        ),
    }


def _finite_clearance(values: object, occupancy: object) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.ndim != 1 or np.any(np.isnan(array)) or np.any(np.isneginf(array)):
        raise DiagnosticValidationError("clearances must be finite or positive infinity")
    if np.any(np.isposinf(array)):
        grid = np.asarray(occupancy.grid)
        maximum = float(np.linalg.norm(np.asarray(grid.shape)) * float(occupancy.resolution_m))
        array = np.where(np.isposinf(array), maximum, array)
    return array


def _action_aligned_policy_diagnostic(
    result: object,
    name: str,
    action_count: int,
    *,
    required: bool = False,
    boolean: bool = False,
) -> np.ndarray:
    """Copy one action-aligned diagnostic with optional fail-closed presence."""

    source_name = name
    if name == "elapsed_s" and hasattr(result, "elapsed_s_action_aligned"):
        source_name = "elapsed_s_action_aligned"
    if not hasattr(result, source_name):
        if required:
            raise DiagnosticValidationError(
                f"policy diagnostic {name} is missing"
            )
        return np.zeros(action_count, dtype=bool if boolean else np.float64)
    value = getattr(result, source_name)
    if value is None:
        raise DiagnosticValidationError(
            f"policy diagnostic {name} is malformed"
        )
    try:
        raw = np.asarray(value)
    except (TypeError, ValueError) as error:
        raise DiagnosticValidationError(
            f"policy diagnostic {name} is malformed"
        ) from error
    # Validate the source dtype before conversion so complex values cannot
    # silently discard their imaginary component and strings/object arrays
    # cannot be coerced into a numeric trace.  Present empty arrays are also
    # malformed rather than an implicit legacy default.
    if raw.shape != (action_count,):
        raise DiagnosticValidationError(
            f"policy diagnostic {name} must be a finite trace with shape ({action_count},)"
        )
    if boolean:
        if raw.dtype.kind != "b":
            raise DiagnosticValidationError(
                f"policy diagnostic {name} must be a boolean trace with shape ({action_count},)"
            )
        return np.asarray(raw, dtype=bool).copy()
    if raw.dtype.kind not in "iuf":
        raise DiagnosticValidationError(
            f"policy diagnostic {name} must be a finite real trace with shape ({action_count},)"
        )
    try:
        array = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as error:
        raise DiagnosticValidationError(
            f"policy diagnostic {name} is malformed"
        ) from error
    if not np.all(np.isfinite(array)):
        raise DiagnosticValidationError(
            f"policy diagnostic {name} must be finite"
        )
    return array.copy()


_V2_RUNNER_EVENT_FIELDS = {
    "diagnostic_terminal_brake_start_time_s": "terminal_brake_start_time_s",
    "diagnostic_goal_entry_time_s": "goal_entry_time_s",
    "diagnostic_stop_below_speed_time_s": "stop_below_speed_time_s",
    "diagnostic_hold_complete_time_s": "hold_complete_time_s",
}


def _runner_terminal_events(
    result: object,
    *,
    required: bool = False,
) -> dict[str, float | None]:
    """Validate runner-owned terminal events and return metric field names."""

    events: dict[str, float | None] = {}
    for source_name, metric_name in _V2_RUNNER_EVENT_FIELDS.items():
        if not hasattr(result, source_name):
            if required:
                raise DiagnosticValidationError(
                    f"runner terminal event {metric_name} is missing"
                )
            events[metric_name] = None
            continue
        value = getattr(result, source_name)
        if value is None:
            events[metric_name] = None
            continue
        if isinstance(value, (bool, np.bool_)):
            raise DiagnosticValidationError(
                f"runner terminal event {metric_name} must be finite and non-negative"
            )
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise DiagnosticValidationError(
                f"runner terminal event {metric_name} must be finite and non-negative"
            ) from error
        if not math.isfinite(numeric) or numeric < 0.0:
            raise DiagnosticValidationError(
                f"runner terminal event {metric_name} must be finite and non-negative"
            )
        events[metric_name] = numeric

    brake = events["terminal_brake_start_time_s"]
    goal = events["goal_entry_time_s"]
    stop = events["stop_below_speed_time_s"]
    hold = events["hold_complete_time_s"]
    if brake is not None and goal is not None and brake > goal + 1e-12:
        raise DiagnosticValidationError(
            "runner terminal event ordering requires brake start <= goal entry"
        )
    if stop is not None and goal is None:
        raise DiagnosticValidationError(
            "runner terminal event ordering requires goal entry before low speed"
        )
    if stop is not None and goal is not None and stop < goal - 1e-12:
        raise DiagnosticValidationError(
            "runner terminal event ordering requires goal entry <= low speed"
        )
    if hold is not None and stop is None:
        raise DiagnosticValidationError(
            "runner terminal event ordering requires low speed before hold complete"
        )
    if hold is not None and stop is not None and hold < stop - 1e-12:
        raise DiagnosticValidationError(
            "runner terminal event ordering requires low speed <= hold complete"
        )
    return events


def _case_config_payload(
    case: TrackingCase,
    reference: TrackingReference,
    episode: BenchmarkEpisode,
    harness_config: Mapping[str, object],
    *,
    fixture_path: Path | None = None,
    occupancy_path: Path | None = None,
    contract_path: Path | None = None,
    profile_path: Path | None = None,
) -> dict[str, object]:
    paths = _harness_paths(harness_config)
    fixture = fixture_path or paths["fixture_path"]
    occupancy = occupancy_path or paths["occupancy_path"]
    contract = contract_path or paths["contract_path"]
    profile = profile_path or paths["drive_profile_path"]
    fixture = Path(fixture)
    occupancy = Path(occupancy)
    contract = Path(contract)
    profile = Path(profile)
    policy_variant = _validate_policy_variant(
        harness_config.get("policy_variant", DEFAULT_POLICY_VARIANT)
    )
    policy_values: Mapping[str, object] = harness_config
    raw_config_path: Path | None = None
    raw_policy_block: dict[str, object] | None = None
    v2_override_metadata: dict[str, object] | None = None
    if policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
        nested_harness = harness_config.get("harness", {})
        raw_config_value = (
            nested_harness.get("config_path")
            if isinstance(nested_harness, Mapping)
            else None
        )
        if not isinstance(raw_config_value, (str, Path)) or not str(raw_config_value):
            raise DiagnosticValidationError(
                "V2 diagnostic provenance config_path is required"
            )
        raw_config_path = Path(raw_config_value).expanduser().resolve()
        try:
            raw_payload = _read_json_object(raw_config_path)
            raw_policy_block = _v2_raw_policy_block_from_payload(raw_payload)
            authority = _v2_parameters_from_payload(raw_payload)
        except (OSError, UnicodeError, ValueError) as error:
            raise DiagnosticValidationError(
                "V2 diagnostic provenance raw config is invalid"
            ) from error
        values = dict(harness_config)
        for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
            if values.get(name) is None:
                values[name] = authority[name]
        policy_values = values
    policy_parameters = _resolve_policy_parameters(policy_variant, policy_values)
    payload: dict[str, object] = {
        "schema_version": "tracking_acceptance_case_v1",
        "policy_variant": policy_variant,
        "policy": {
            "name": "oracle" if policy_variant == DEFAULT_POLICY_VARIANT else policy_variant,
            "variant": policy_variant,
        },
        "case": {
            "case_id": case.case_id,
            "kind": case.kind,
            "main_command_rad_s": case.main_command_rad_s,
            "diagnostic_budget_s": case.budget_s,
            "probe": case.probe,
            "parameters": dict(case.parameters),
        },
        "reference": {
            "path_length_m": reference.path_length_m,
            "point_count": int(reference.points_xy.shape[0]),
            "segment_boundaries": list(reference.segment_boundaries),
            "segment_directions": list(reference.segment_directions),
            "stop_marker_indices": list(reference.stop_marker_indices),
        },
        "episode": episode.to_dict(),
        "fixture_sha256": _sha256(fixture),
        "occupancy_sha256": _sha256(occupancy),
        "contract_sha256": _sha256(contract),
        "drive_profile_sha256": _sha256(profile),
        "harness_sha256": _sha256(_HERE),
        "geometry_sha256": _sha256(
            Path(__file__).resolve().parents[1] / "benchmark" / "tracking_acceptance.py"
        ),
        "policy_sha256": _sha256(_PROJECT_ROOT / "benchmark" / "policies.py"),
        "runner_sha256": _sha256(_PROJECT_ROOT / "benchmark" / "runner.py"),
        "trajectory_profile_sha256": _sha256(
            _PROJECT_ROOT / "benchmark" / "trajectory_tracking.py"
        ),
        "diagnostic_budget_s": case.budget_s,
        "formal_budget_s": 35.0,
        "controller": harness_config.get("controller", {}),
        "goal_tolerance_m": harness_config.get("goal_tolerance_m", 0.2),
        "stop_speed_m_s": harness_config.get("stop_speed_m_s", 0.05),
        "stop_hold_s": harness_config.get("stop_hold_s", 0.5),
        "settling_budget_s": harness_config.get("settling_budget_s", 3.0),
        "replan_hz": harness_config.get("replan_hz", 10),
        "fixture": {
            "scene_path": str(fixture.resolve()),
            "scene_sha256": _sha256(fixture),
            "occupancy_path": str(occupancy.resolve()),
            "occupancy_sha256": _sha256(occupancy),
            "mode": "diagnostic_open_plane",
            "physical_contact_source": "filtered obstacle contact",
        },
        "contract": {"path": str(contract.resolve()), "sha256": _sha256(contract)},
        "drive_profile": {"path": str(profile.resolve()), "sha256": _sha256(profile)},
        "source": {
            "harness_path": str(_HERE),
            "harness_sha256": _sha256(_HERE),
            "geometry_path": str(Path(__file__).resolve().parents[1] / "benchmark" / "tracking_acceptance.py"),
            "geometry_sha256": _sha256(Path(__file__).resolve().parents[1] / "benchmark" / "tracking_acceptance.py"),
            "policy_path": str(_PROJECT_ROOT / "benchmark" / "policies.py"),
            "policy_sha256": _sha256(_PROJECT_ROOT / "benchmark" / "policies.py"),
            "runner_path": str(_PROJECT_ROOT / "benchmark" / "runner.py"),
            "runner_sha256": _sha256(_PROJECT_ROOT / "benchmark" / "runner.py"),
            "trajectory_profile_path": str(
                _PROJECT_ROOT / "benchmark" / "trajectory_tracking.py"
            ),
            "trajectory_profile_sha256": _sha256(
                _PROJECT_ROOT / "benchmark" / "trajectory_tracking.py"
            ),
            "git_revision": _git_revision(),
            "python_version": platform.python_version(),
            "numpy_version": np.__version__,
        },
    }
    # Keep the frozen baseline config literal-compatible: trajectory knobs are
    # present only for the selected variant.  The two existing baseline knobs
    # remain at their historical top-level locations.
    if policy_variant in (DEFAULT_POLICY_VARIANT, TRAJECTORY_POLICY_VARIANT):
        payload["lookahead_m"] = policy_parameters["lookahead_m"]
        payload["yaw_rate_feedback_gain_s"] = policy_parameters[
            "yaw_rate_feedback_gain_s"
        ]
    if policy_variant == TRAJECTORY_POLICY_VARIANT:
        for name in TRAJECTORY_POLICY_PARAMETER_NAMES:
            payload[name] = policy_parameters[name]
        payload["trajectory_policy"] = {
            "name": TRAJECTORY_POLICY_VARIANT,
            "parameters": dict(policy_parameters),
        }
    elif policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
        assert raw_config_path is not None
        assert raw_policy_block is not None
        if not raw_config_path.is_file():
            raise DiagnosticValidationError(
                f"V2 diagnostic provenance config_path is not a file: {raw_config_path}"
            )
        raw_config_sha256 = _sha256(raw_config_path)
        raw_provenance = harness_config.get("trajectory_policy_v2_provenance")
        try:
            v2_override_metadata = _v2_override_metadata(raw_provenance)
            raw_parameters = _v2_parameters_from_payload(
                _read_json_object(raw_config_path)
            )
            override_values = v2_override_metadata["override_values"]
            reconstructed_parameters = _resolve_policy_parameters(
                TRAJECTORY_POLICY_V2_VARIANT,
                {**raw_parameters, **override_values},
            )
        except (OSError, UnicodeError, ValueError, TypeError) as error:
            raise DiagnosticValidationError(
                "V2 diagnostic provenance override metadata is invalid"
            ) from error
        if any(
            not math.isclose(
                policy_parameters[name], reconstructed_parameters[name],
                rel_tol=0.0, abs_tol=1e-12,
            )
            for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES
        ):
            raise DiagnosticValidationError(
                "V2 effective policy parameters do not match raw config authority "
                "and explicit overrides"
            )
        payload["tracking_config_sha256"] = raw_config_sha256
        payload["tracking_config"] = {
            "path": str(raw_config_path),
            "sha256": raw_config_sha256,
            "raw_policy": raw_policy_block,
            "override_markers": v2_override_metadata["override_markers"],
            "override_values": v2_override_metadata["override_values"],
        }
        # Preserve the selected args.config path independently from the hash
        # reference.  Summary validation rejects path-only rewrites by checking
        # this duplicate against tracking_config.path.
        payload["harness"] = {"config_path": str(raw_config_path)}
        for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
            payload[name] = policy_parameters[name]
        payload["trajectory_policy"] = {
            "name": TRAJECTORY_POLICY_V2_VARIANT,
            "parameters": dict(policy_parameters),
        }
    return payload


def _git_revision() -> str | None:
    try:
        completed = subprocess.run(
            ("git", "-C", str(_PROJECT_ROOT.parent.parent), "rev-parse", "HEAD"),
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    value = completed.stdout.strip()
    return value if completed.returncode == 0 and value else None


def verify_config_hash(case_dir: str | Path) -> bool:
    directory = Path(case_dir)
    config = directory / "config.json"
    digest_path = directory / "config.sha256"
    if not config.is_file() or not digest_path.is_file():
        raise DiagnosticValidationError("config.json/config.sha256 are required")
    expected = digest_path.read_text(encoding="utf-8").strip()
    actual = hashlib.sha256(config.read_bytes()).hexdigest()
    if expected != actual:
        raise DiagnosticValidationError("config.sha256 does not match config.json")
    return True


def _atomic_case_directory(output_root: str | Path, case_id: str) -> tuple[Path, Path]:
    root = Path(output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    destination = root / case_id
    partial = root / f".{case_id}.partial"
    if destination.exists() or partial.exists():
        raise FileExistsError(f"refusing to overwrite existing diagnostic case: {destination}")
    partial.mkdir()
    return partial, destination


def write_tracking_artifacts(
    output_root: str | Path,
    *,
    case: TrackingCase,
    reference: TrackingReference,
    episode: BenchmarkEpisode,
    result: object,
    runtime: object,
    harness_config: Mapping[str, object],
) -> Path:
    """Write one case's diagnostic files into an atomic, non-overwriting dir."""

    policy_variant = _validate_policy_variant(
        harness_config.get("policy_variant", DEFAULT_POLICY_VARIANT)
    )
    expected_policy_name = (
        "oracle" if policy_variant == DEFAULT_POLICY_VARIANT else policy_variant
    )
    actual_policy_name = getattr(result, "policy_name", None)
    if actual_policy_name != expected_policy_name:
        raise DiagnosticValidationError(
            "policy identity does not match requested policy variant: "
            f"expected {expected_policy_name!r}, got {actual_policy_name!r}"
        )

    physics_diagnostics = _validated_physics_diagnostics(runtime, result)
    validate_diagnostic_timing(result, replan_hz=int(harness_config.get("replan_hz", 10)))
    metrics = compute_tracking_metrics(
        case,
        reference,
        result,
        goal_tolerance_m=float(harness_config.get("goal_tolerance_m", 0.2)),
        stop_speed_m_s=float(harness_config.get("stop_speed_m_s", 0.05)),
        stop_hold_s=float(harness_config.get("stop_hold_s", 0.5)),
        settling_budget_s=float(harness_config.get("settling_budget_s", 3.0)),
        replan_hz=int(harness_config.get("replan_hz", 10)),
    )
    paths = _harness_paths(harness_config)
    config_payload = _case_config_payload(
        case,
        reference,
        episode,
        harness_config,
        fixture_path=Path(paths["fixture_path"]),
        occupancy_path=Path(paths["occupancy_path"]),
        contract_path=Path(paths["contract_path"]),
        profile_path=Path(getattr(getattr(runtime, "config", None), "drive_profile_path", DEFAULT_DRIVE_PROFILE)),
    )
    partial, destination = _atomic_case_directory(output_root, case.case_id)
    try:
        config_bytes = _canonical_json_bytes(config_payload)
        (partial / "config.json").write_bytes(config_bytes)
        config_sha256 = hashlib.sha256(config_bytes).hexdigest()
        (partial / "config.sha256").write_text(config_sha256 + "\n", encoding="utf-8")

        position = _trace(result, "positions_world_xy", ndim=2).astype(np.float64)
        actions = _trace(result, "actions", ndim=2).astype(np.float32)
        action_count = int(actions.shape[0])
        if policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
            reported_action_blocks = getattr(result, "action_blocks", None)
            if (
                isinstance(reported_action_blocks, bool)
                or not isinstance(reported_action_blocks, (int, np.integer))
                or int(reported_action_blocks) != action_count
            ):
                raise DiagnosticValidationError(
                    "V2 action_blocks must exactly match the action trace length"
                )
        trajectory_diagnostics = {
            name: _action_aligned_policy_diagnostic(
                result,
                name,
                action_count,
                required=policy_variant == TRAJECTORY_POLICY_V2_VARIANT,
            )
            for name in TRAJECTORY_DIAGNOSTIC_NAMES
        }
        # ``heading_error_rad`` is already the state-aligned geometric metric
        # in states.npz.  Preserve that established field and use an explicit
        # policy prefix for the action-aligned V1 diagnostic.
        trajectory_diagnostics["heading_error_rad"] = _action_aligned_policy_diagnostic(
            result,
            "heading_error_rad",
            action_count,
            required=policy_variant == TRAJECTORY_POLICY_V2_VARIANT,
        )
        if policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
            for name in TRAJECTORY_POLICY_V2_DIAGNOSTIC_NAMES:
                trajectory_diagnostics[name] = _action_aligned_policy_diagnostic(
                    result,
                    name,
                    action_count,
                    required=True,
                    boolean=name == "budget_infeasible",
                )
        runner_terminal_events = (
            _runner_terminal_events(result, required=True)
            if policy_variant == TRAJECTORY_POLICY_V2_VARIANT
            else {}
        )
        goal_errors = np.linalg.norm(position - np.asarray(reference.goal_xy), axis=1)
        velocity = _trace(result, "linear_velocity_world_m_s", ndim=2)
        yaw_world = _trace(result, "yaw_world_rad", ndim=1)
        reference_yaw = _reference_yaw_for_states(position, reference)
        states = {
            "position_world_xy": position,
            "goal_error_m": goal_errors,
            "yaw_world_rad": yaw_world,
            "reference_yaw_rad": reference_yaw,
            "cross_track_error_m": polyline_cross_track_errors(position, reference.points_xy),
            "heading_error_rad": heading_errors_rad(yaw_world, reference_yaw),
            "linear_velocity_world_m_s": velocity,
            "speed_m_s": np.linalg.norm(velocity[:, :2], axis=1),
            "angular_velocity_world_rad_s": _trace(result, "angular_velocity_world_rad_s", ndim=2),
            "joint_position_rad": _trace(result, "joint_position_rad", ndim=2),
            "joint_velocity_rad_s": _trace(result, "joint_velocity_rad_s", ndim=2),
            "applied_action": _trace(result, "applied_action", ndim=2),
            "clearance_m": _finite_clearance(result.clearances_m, runtime.occupancy),
            "physical_contact": _trace(result, "physical_contacts", ndim=1).astype(bool),
            "collision": _trace(result, "collisions", ndim=1).astype(bool),
            "simulation_time_s": _trace(result, "simulation_time_s", ndim=1),
            "physics_step_index": _trace(result, "physics_step_index", ndim=1),
            "raw_simulation_time_s": _trace(result, "raw_simulation_time_s", ndim=1),
            "raw_physics_step_index": _trace(result, "raw_physics_step_index", ndim=1),
            "roll_world_rad": _trace(result, "roll_world_rad", ndim=1),
            "diagnostic_settling_mask": np.asarray(getattr(result, "diagnostic_settling_mask", np.zeros(result.actions.shape[0], dtype=bool)), dtype=bool),
            "kinodynamic_reference_pose": _reference_arrays(reference)["reference_pose"],
            "kinodynamic_reference_command": _reference_arrays(reference)["reference_command"],
            "kinodynamic_reference_drive_direction": _reference_arrays(reference)["reference_drive_direction"],
            "policy_reference_index": _trace(result, "policy_reference_index", ndim=1),
            "policy_cross_track_error_m": _trace(result, "policy_cross_track_error_m", ndim=1),
            "policy_active_drive_direction": _trace(result, "policy_active_drive_direction", ndim=1),
            "policy_desired_drive_direction": _trace(result, "policy_desired_drive_direction", ndim=1),
            "policy_pending_drive_direction": _trace(result, "policy_pending_drive_direction", ndim=1),
            "policy_stopped_cycles": _trace(result, "policy_stopped_cycles", ndim=1),
            "policy_braking_state_code": _trace(result, "policy_braking_state_code", ndim=1),
            "reference_curvature_1_m": trajectory_diagnostics["reference_curvature_1_m"],
            "feedback_curvature_1_m": trajectory_diagnostics["feedback_curvature_1_m"],
            "command_curvature_1_m": trajectory_diagnostics["command_curvature_1_m"],
            "lateral_error_m": trajectory_diagnostics["lateral_error_m"],
            "policy_heading_error_rad": trajectory_diagnostics["heading_error_rad"],
            "main_cap_rad_s": trajectory_diagnostics["main_cap_rad_s"],
            "auxiliary_feedforward_rad": trajectory_diagnostics["auxiliary_feedforward_rad"],
            "auxiliary_feedback_rad": trajectory_diagnostics["auxiliary_feedback_rad"],
        }
        if policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
            states.update(
                {
                    name: trajectory_diagnostics[name]
                    for name in TRAJECTORY_POLICY_V2_DIAGNOSTIC_NAMES
                }
            )
        if states["diagnostic_settling_mask"].shape != result.actions.shape[:1]:
            raise DiagnosticValidationError("diagnostic_settling_mask must align with actions")
        # Boundary flags are separate fixed-size metadata arrays and cannot be
        # inferred from a policy reference index alone.
        boundary_evidence = metrics["direction_boundaries"]
        boundary_items = boundary_evidence.get("boundaries", []) if isinstance(boundary_evidence, Mapping) else []
        states["direction_boundary_reached"] = np.asarray(
            [bool(item.get("boundary_reached", False)) for item in boundary_items], dtype=bool
        )
        states["active_direction_changed"] = np.asarray(
            [bool(item.get("active_direction_changed", False)) for item in boundary_items], dtype=bool
        )
        states["direction_boundary_stop_hold_passed"] = np.asarray(
            [bool(item.get("stop_hold_passed", False)) for item in boundary_items], dtype=bool
        )
        np.savez(partial / "states.npz", **states)
        np.savez(partial / "physics_diagnostics.npz", **physics_diagnostics)

        commands = actions.reshape((-1, 5, 2))
        refs = _reference_arrays(reference)
        np.savez(
            partial / "commands.npz",
            action_blocks=actions,
            controls=commands,
            **refs,
        )
        collision_points = position[np.asarray(result.collisions, dtype=bool)]
        render_trajectory_map(
            partial / "trajectory.png",
            runtime.occupancy,
            (
                TrajectoryRecord("reference", reference.points_xy, color_rgb=(35, 105, 210), draw_endpoint_markers=False),
                TrajectoryRecord("actual", position, collision_points_xy=collision_points, color_rgb=(230, 126, 34)),
            ),
            title=f"Tracking acceptance: {case.case_id}",
        )
        frames_payload = getattr(runtime, "captured_frames", {})
        frames = frames_payload.get("third_person", ()) if isinstance(frames_payload, Mapping) else ()
        if not frames:
            raise DiagnosticValidationError("third_person video frames are unavailable")
        write_rgb_video(partial / "third_person.mp4", frames, fps=float(harness_config.get("replan_hz", 10)))
        policy_variant = _validate_policy_variant(
            harness_config.get("policy_variant", DEFAULT_POLICY_VARIANT)
        )
        policy_parameters = _resolve_policy_parameters(policy_variant, harness_config)
        metrics_payload = dict(metrics)
        metrics_payload.update(
            {
                "episode_id": episode.episode_id,
                "action_blocks": int(result.action_blocks),
                "formal_budget_s": 35.0,
                "diagnostic_budget_s": case.budget_s,
                "diagnostic_settling_budget_s": float(harness_config.get("settling_budget_s", 3.0)),
                "config_sha256": config_sha256,
                "policy_variant": policy_variant,
                "policy": getattr(result, "policy_name", "oracle"),
            }
        )
        metrics_payload.update(runner_terminal_events)
        if policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
            metrics_payload["elapsed_s"] = float(getattr(result, "elapsed_s", 0.0))
            metrics_payload["tracking_config_sha256"] = config_payload[
                "tracking_config_sha256"
            ]
        if policy_variant in (DEFAULT_POLICY_VARIANT, TRAJECTORY_POLICY_VARIANT):
            metrics_payload.update(
                {
                    "lookahead_m": policy_parameters["lookahead_m"],
                    "yaw_rate_feedback_gain_s": policy_parameters[
                        "yaw_rate_feedback_gain_s"
                    ],
                }
            )
        if policy_variant == TRAJECTORY_POLICY_VARIANT:
            metrics_payload["trajectory_policy"] = {
                "name": TRAJECTORY_POLICY_VARIANT,
                "parameters": dict(policy_parameters),
            }
            for name in TRAJECTORY_POLICY_PARAMETER_NAMES:
                metrics_payload[name] = policy_parameters[name]
        elif policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
            metrics_payload["trajectory_policy"] = {
                "name": TRAJECTORY_POLICY_V2_VARIANT,
                "parameters": dict(policy_parameters),
            }
            for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
                metrics_payload[name] = policy_parameters[name]
        metrics_payload["trajectory_diagnostics"] = {
            name: values.tolist() for name, values in trajectory_diagnostics.items()
        }
        # Preserve the established state-aligned heading distribution at the
        # top level while exposing the action-aligned policy trace explicitly.
        metrics_payload["policy_heading_error_rad"] = trajectory_diagnostics[
            "heading_error_rad"
        ].tolist()
        for name, values in trajectory_diagnostics.items():
            if name != "heading_error_rad":
                if name == "elapsed_s":
                    # ``elapsed_s`` is already the formal scalar timing field;
                    # retain it and publish the policy-local action trace under
                    # an explicit alias as well as in the nested evidence.
                    metrics_payload["elapsed_s_action_aligned"] = values.tolist()
                    metrics_payload["policy_elapsed_s"] = values.tolist()
                else:
                    metrics_payload[name] = values.tolist()
        (partial / "metrics.json").write_bytes(_canonical_json_bytes(metrics_payload))
        verify_config_hash(partial)
        partial.replace(destination)
    except Exception:
        # Keep a partial tree for forensic recovery, but never publish a
        # directory that did not pass timing/evidence/artifact validation.
        raise
    return destination


def _build_plan(reference: TrackingReference) -> KinodynamicPlanResult:
    nonzero_changes = sum(
        left.drive_direction != right.drive_direction
        for left, right in zip(reference.waypoints, reference.waypoints[1:])
        if left.main_command_rad_s != 0.0 and right.main_command_rad_s != 0.0
    )
    return KinodynamicPlanResult(
        status="success",
        waypoints=reference.waypoints,
        expanded_nodes=0,
        path_length_m=reference.path_length_m,
        direction_changes=nonzero_changes,
    )


def _build_oracle_policy(
    protocol: object,
    profile_path: Path,
    *,
    lookahead_m: float,
    yaw_rate_feedback_gain_s: float = DEFAULT_YAW_RATE_FEEDBACK_GAIN_S,
) -> OraclePathPolicy:
    return OraclePathPolicy(
        protocol,
        lookahead_m=_validate_lookahead_m(lookahead_m),
        yaw_rate_feedback_gain_s=_validate_yaw_rate_feedback_gain_s(
            yaw_rate_feedback_gain_s
        ),
        drive_model=DriveModel.from_json(profile_path),
    )


def _build_policy(
    protocol: object,
    profile_path: Path,
    *,
    policy_variant: str = DEFAULT_POLICY_VARIANT,
    lookahead_m: float | None = None,
    smoothing_window_m: float | None = None,
    curvature_preview_m: float | None = None,
    lateral_curvature_gain_1_m2: float | None = None,
    heading_curvature_gain_1_m: float | None = None,
    max_feedback_curvature_1_m: float | None = None,
    yaw_rate_feedback_gain_s: float | None = None,
    latency_preview_s: float | None = None,
    auxiliary_latency_s: float | None = None,
    maximum_delay_distance_m: float | None = None,
    profile_window_m: float | None = None,
    profile_blend_start_1_m: float | None = None,
    profile_blend_full_1_m: float | None = None,
    feedback_lookahead_m: float | None = None,
    maximum_feedback_curvature_1_m: float | None = None,
    supported_main_command_rad_s: float | None = None,
    episode_budget_s: float | None = None,
    terminal_brake_buffer_m: float | None = None,
) -> object:
    """Build one acceptance policy while keeping baseline dispatch literal."""

    variant = _validate_policy_variant(policy_variant)
    values = {
        "lookahead_m": lookahead_m,
        "smoothing_window_m": smoothing_window_m,
        "curvature_preview_m": curvature_preview_m,
        "lateral_curvature_gain_1_m2": lateral_curvature_gain_1_m2,
        "heading_curvature_gain_1_m": heading_curvature_gain_1_m,
        "max_feedback_curvature_1_m": max_feedback_curvature_1_m,
        "yaw_rate_feedback_gain_s": yaw_rate_feedback_gain_s,
        "latency_preview_s": latency_preview_s,
        "auxiliary_latency_s": auxiliary_latency_s,
        "maximum_delay_distance_m": maximum_delay_distance_m,
        "profile_window_m": profile_window_m,
        "profile_blend_start_1_m": profile_blend_start_1_m,
        "profile_blend_full_1_m": profile_blend_full_1_m,
        "feedback_lookahead_m": feedback_lookahead_m,
        "maximum_feedback_curvature_1_m": maximum_feedback_curvature_1_m,
        "supported_main_command_rad_s": supported_main_command_rad_s,
        "episode_budget_s": episode_budget_s,
        "terminal_brake_buffer_m": terminal_brake_buffer_m,
    }
    resolved = _resolve_policy_parameters(variant, values)
    if variant == DEFAULT_POLICY_VARIANT:
        return _build_oracle_policy(
            protocol,
            profile_path,
            lookahead_m=resolved["lookahead_m"],
            yaw_rate_feedback_gain_s=resolved["yaw_rate_feedback_gain_s"],
        )

    if variant == TRAJECTORY_POLICY_VARIANT:
        # TrajectoryOraclePolicy intentionally keeps its exact constructor
        # (which has no drive_model parameter).  Bind the accepted profile at
        # the integration boundary after constructing it with all eight fixed
        # knobs.
        policy = TrajectoryOraclePolicy(protocol, **resolved)
        policy.drive_model = DriveModel.from_json(profile_path)
        return policy

    return DelayAwareTrajectoryOraclePolicy(
        protocol,
        **resolved,
        drive_model=DriveModel.from_json(profile_path),
    )


def run_tracking_case(
    case: TrackingCase,
    *,
    harness_config: Mapping[str, object],
    output_root: str | Path,
    gui: bool,
    policy_variant: str = DEFAULT_POLICY_VARIANT,
    yaw_rate_feedback_gain_s: float | None = None,
    lookahead_m: float | None = None,
    smoothing_window_m: float | None = None,
    curvature_preview_m: float | None = None,
    lateral_curvature_gain_1_m2: float | None = None,
    heading_curvature_gain_1_m: float | None = None,
    max_feedback_curvature_1_m: float | None = None,
    latency_preview_s: float | None = None,
    auxiliary_latency_s: float | None = None,
    maximum_delay_distance_m: float | None = None,
    profile_window_m: float | None = None,
    profile_blend_start_1_m: float | None = None,
    profile_blend_full_1_m: float | None = None,
    feedback_lookahead_m: float | None = None,
    maximum_feedback_curvature_1_m: float | None = None,
    supported_main_command_rad_s: float | None = None,
    episode_budget_s: float | None = None,
    terminal_brake_buffer_m: float | None = None,
) -> dict[str, object]:
    """Run one case in one runtime instance and publish its diagnostics."""

    if not isinstance(case, TrackingCase):
        raise TypeError("case must be a TrackingCase")
    variant = _validate_policy_variant(policy_variant)
    explicit_v2_overrides = {
        name: value
        for name, value in {
            "auxiliary_latency_s": auxiliary_latency_s,
            "maximum_delay_distance_m": maximum_delay_distance_m,
            "profile_window_m": profile_window_m,
            "profile_blend_start_1_m": profile_blend_start_1_m,
            "profile_blend_full_1_m": profile_blend_full_1_m,
            "feedback_lookahead_m": feedback_lookahead_m,
            "maximum_feedback_curvature_1_m": maximum_feedback_curvature_1_m,
            "supported_main_command_rad_s": supported_main_command_rad_s,
            "episode_budget_s": episode_budget_s,
            "terminal_brake_buffer_m": terminal_brake_buffer_m,
        }.items()
        if value is not None
    }
    supplied_parameters = dict(harness_config)
    if variant == TRAJECTORY_POLICY_V2_VARIANT:
        nested_harness = harness_config.get("harness", {})
        raw_config_value = (
            nested_harness.get("config_path")
            if isinstance(nested_harness, Mapping)
            else None
        )
        if isinstance(raw_config_value, (str, Path)) and str(raw_config_value):
            authority = _v2_parameters_from_config(raw_config_value)
            for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
                if supplied_parameters.get(name) is None:
                    supplied_parameters[name] = authority[name]
    supplied_parameters.update(
        {
            name: value
            for name, value in {
                "lookahead_m": lookahead_m,
                "smoothing_window_m": smoothing_window_m,
                "curvature_preview_m": curvature_preview_m,
                "lateral_curvature_gain_1_m2": lateral_curvature_gain_1_m2,
                "heading_curvature_gain_1_m": heading_curvature_gain_1_m,
                "max_feedback_curvature_1_m": max_feedback_curvature_1_m,
                "yaw_rate_feedback_gain_s": yaw_rate_feedback_gain_s,
                "latency_preview_s": latency_preview_s,
                "auxiliary_latency_s": auxiliary_latency_s,
                "maximum_delay_distance_m": maximum_delay_distance_m,
                "profile_window_m": profile_window_m,
                "profile_blend_start_1_m": profile_blend_start_1_m,
                "profile_blend_full_1_m": profile_blend_full_1_m,
                "feedback_lookahead_m": feedback_lookahead_m,
                "maximum_feedback_curvature_1_m": maximum_feedback_curvature_1_m,
                "supported_main_command_rad_s": supported_main_command_rad_s,
                "episode_budget_s": episode_budget_s,
                "terminal_brake_buffer_m": terminal_brake_buffer_m,
            }.items()
            if value is not None
        }
    )
    resolved_parameters = _resolve_policy_parameters(variant, supplied_parameters)
    effective_harness_config = dict(harness_config)
    effective_harness_config["policy_variant"] = variant
    effective_harness_config.update(resolved_parameters)
    if (
        variant == TRAJECTORY_POLICY_V2_VARIANT
        and "trajectory_policy_v2_provenance" not in effective_harness_config
    ):
        markers = {
            name: name in explicit_v2_overrides
            for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES
        }
        effective_harness_config["trajectory_policy_v2_provenance"] = {
            "override_markers": markers,
            "override_values": {
                name: resolved_parameters[name]
                for name, marked in markers.items()
                if marked
            },
        }
    if variant == TRAJECTORY_POLICY_V2_VARIANT:
        configured_settling_budget_s = float(
            effective_harness_config.get("settling_budget_s", 3.0)
        )
        effective_harness_config["settling_budget_s"] = min(
            3.0,
            resolved_parameters["episode_budget_s"],
            configured_settling_budget_s,
        )
    paths = _harness_paths(harness_config)
    reference = build_tracking_reference(case)
    validate_diagnostic_fixture(paths["fixture_path"], paths["occupancy_path"])
    episode = build_tracking_episode(
        case,
        reference,
        scene_root=paths["scene_root"],
        fixture_path=paths["fixture_path"],
        occupancy_path=paths["occupancy_path"],
        seed=int(paths["seed"]),
    )
    protocol = load_action_protocol(paths["contract_path"])
    if protocol.protocol_id != "ball_action_v2_aux30":
        raise ValueError("tracking acceptance requires ball_action_v2_aux30")
    profile_path = Path(paths["drive_profile_path"])
    runtime = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(
            scene_root=Path(paths["scene_root"]),
            contract_path=Path(paths["contract_path"]),
            gui=bool(gui),
            record_video=bool(paths["record_video"]),
            occupancy_path=Path(paths["occupancy_path"]),
            drive_profile_path=profile_path,
            diagnostic_open_plane=True,
            diagnostic_physics_trace=True,
            diagnostic_auxiliary_target_rate_rad_s=(
                float(effective_harness_config["controller"]["auxiliary_target_rate_limit_rad_s"])
                if isinstance(effective_harness_config.get("controller"), Mapping)
                and "auxiliary_target_rate_limit_rad_s"
                in effective_harness_config["controller"]
                else None
            ),
            diagnostic_auxiliary_velocity_limit_rad_s=(
                float(effective_harness_config["controller"]["auxiliary_velocity_limit_rad_s"])
                if isinstance(effective_harness_config.get("controller"), Mapping)
                and "auxiliary_velocity_limit_rad_s"
                in effective_harness_config["controller"]
                else None
            ),
        )
    )
    # Keep the third-person diagnostic overlay on the injected geometry.  The
    # runtime's internal all-free A* path is privileged bookkeeping, not the
    # acceptance reference for curved or mixed-direction cases.
    runtime.set_reference_visualization_path(reference.points_xy)
    return_code = 1
    try:
        policy = _build_policy(
            protocol,
            profile_path,
            policy_variant=variant,
            **resolved_parameters,
        )
        result = run_episode(
            runtime,
            policy,
            episode,
            action_protocol=protocol,
            config=RunnerConfig(
                # V2's preregistered budget is a policy parameter and must
                # be the same effective value used by the runner.  Baseline
                # and V1 retain the historical per-case diagnostic budget.
                episode_budget_s=(
                    resolved_parameters["episode_budget_s"]
                    if variant == TRAJECTORY_POLICY_V2_VARIANT
                    else case.budget_s
                ),
                replan_hz=int(harness_config.get("replan_hz", 10)),
                goal_tolerance_m=float(harness_config.get("goal_tolerance_m", 0.2)),
                diagnostic_settling_budget_s=float(
                    effective_harness_config.get("settling_budget_s", 3.0)
                ),
                diagnostic_stop_hold_s=float(harness_config.get("stop_hold_s", 0.5)),
                diagnostic_stop_speed_m_s=float(harness_config.get("stop_speed_m_s", 0.05)),
            ),
            oracle_path_xy=reference.points_xy,
            kinodynamic_plan=_build_plan(reference),
        )
        if result.positions_world_xy.shape[0] == 0:
            raise DiagnosticValidationError(result.runtime_error or "runtime produced no state")
        output = write_tracking_artifacts(
            output_root,
            case=case,
            reference=reference,
            episode=episode,
            result=result,
            runtime=runtime,
            harness_config=effective_harness_config,
        )
        return_code = 0
        with (output / "metrics.json").open("r", encoding="utf-8") as stream:
            return json.load(stream)
    finally:
        runtime.close(return_code=return_code)


def _build_parser() -> argparse.ArgumentParser:
    parser = _TrackingArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scene-root", type=Path, default=DEFAULT_SCENE_ROOT)
    parser.add_argument("--fixture", type=Path, default=None)
    parser.add_argument("--occupancy", type=Path, default=DEFAULT_OCCUPANCY)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--run-root", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--gui", action="store_true")
    mode.add_argument("--headless", action="store_true")
    parser.add_argument("--record-video", "--video", dest="record_video", action="store_true", default=True)
    parser.add_argument("--no-video", dest="record_video", action="store_false")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--case-id", action=_CaseIdAction)
    selection.add_argument("--all", dest="all_cases", action="store_true", default=True)
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument(
        "--lookahead-m",
        type=_parse_lookahead_m,
        action=_MarkProvidedAction,
        default=DEFAULT_LOOKAHEAD_M,
    )
    parser.set_defaults(_lookahead_m_provided=False)
    parser.add_argument(
        "--smoothing-window-m",
        type=_parse_smoothing_window_m,
        default=TRAJECTORY_POLICY_DEFAULTS["smoothing_window_m"],
    )
    parser.add_argument(
        "--curvature-preview-m",
        type=_parse_curvature_preview_m,
        default=TRAJECTORY_POLICY_DEFAULTS["curvature_preview_m"],
    )
    parser.add_argument(
        "--lateral-curvature-gain-1-m2",
        type=_parse_lateral_curvature_gain_1_m2,
        default=TRAJECTORY_POLICY_DEFAULTS["lateral_curvature_gain_1_m2"],
    )
    parser.add_argument(
        "--heading-curvature-gain-1-m",
        type=_parse_heading_curvature_gain_1_m,
        default=TRAJECTORY_POLICY_DEFAULTS["heading_curvature_gain_1_m"],
    )
    parser.add_argument(
        "--max-feedback-curvature-1-m",
        type=_parse_max_feedback_curvature_1_m,
        default=TRAJECTORY_POLICY_DEFAULTS["max_feedback_curvature_1_m"],
    )
    parser.add_argument(
        "--yaw-rate-feedback-gain-s",
        type=_parse_yaw_rate_feedback_gain_s,
        default=DEFAULT_YAW_RATE_FEEDBACK_GAIN_S,
    )
    parser.add_argument(
        "--latency-preview-s",
        type=_parse_latency_preview_s,
        default=TRAJECTORY_POLICY_DEFAULTS["latency_preview_s"],
    )
    parser.add_argument(
        "--auxiliary-latency-s",
        type=_parse_auxiliary_latency_s,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["auxiliary_latency_s"],
    )
    parser.add_argument(
        "--maximum-delay-distance-m",
        type=_parse_maximum_delay_distance_m,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["maximum_delay_distance_m"],
    )
    parser.add_argument(
        "--profile-window-m",
        type=_parse_profile_window_m,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["profile_window_m"],
    )
    parser.add_argument(
        "--profile-blend-start-1-m",
        type=_parse_profile_blend_start_1_m,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["profile_blend_start_1_m"],
    )
    parser.add_argument(
        "--profile-blend-full-1-m",
        type=_parse_profile_blend_full_1_m,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["profile_blend_full_1_m"],
    )
    parser.add_argument(
        "--feedback-lookahead-m",
        type=_parse_feedback_lookahead_m,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["feedback_lookahead_m"],
    )
    parser.add_argument(
        "--maximum-feedback-curvature-1-m",
        type=_parse_maximum_feedback_curvature_1_m,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS[
            "maximum_feedback_curvature_1_m"
        ],
    )
    parser.add_argument(
        "--supported-main-command-rad-s",
        type=_parse_supported_main_command_rad_s,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["supported_main_command_rad_s"],
    )
    parser.add_argument(
        "--episode-budget-s",
        type=_parse_episode_budget_s,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["episode_budget_s"],
    )
    parser.add_argument(
        "--terminal-brake-buffer-m",
        type=_parse_terminal_brake_buffer_m,
        action=_MarkProvidedAction,
        default=TRAJECTORY_POLICY_V2_DEFAULTS["terminal_brake_buffer_m"],
    )
    parser.add_argument(
        "--auxiliary-target-rate-deg-s",
        type=_parse_auxiliary_target_rate_deg_s,
        default=None,
        help="diagnostic auxiliary target slew rate in degrees per second",
    )
    parser.add_argument(
        "--auxiliary-velocity-limit-rad-s",
        type=_parse_auxiliary_velocity_limit_rad_s,
        default=None,
        help="diagnostic measured auxiliary joint velocity limit",
    )
    parser.set_defaults(
        _auxiliary_latency_s_provided=False,
        _maximum_delay_distance_m_provided=False,
        _profile_window_m_provided=False,
        _profile_blend_start_1_m_provided=False,
        _profile_blend_full_1_m_provided=False,
        _feedback_lookahead_m_provided=False,
        _maximum_feedback_curvature_1_m_provided=False,
        _supported_main_command_rad_s_provided=False,
        _episode_budget_s_provided=False,
        _terminal_brake_buffer_m_provided=False,
    )
    parser.add_argument(
        "--policy-variant",
        default=DEFAULT_POLICY_VARIANT,
        choices=POLICY_VARIANTS,
    )
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    return parser


def _child_command(args: argparse.Namespace, case_id: str) -> list[str]:
    parameters = _policy_parameters_from_namespace(args)
    command = [
        sys.executable,
        str(_HERE),
        "--config",
        str(args.config),
        "--scene-root",
        str(args.scene_root),
        "--occupancy",
        str(args.occupancy),
        "--contract",
        str(args.contract),
        "--run-root",
        str(args.run_root),
        "--case-id",
        case_id,
        "--policy-variant",
        args.policy_variant,
        "--seed",
        str(args.seed),
        "--record-video" if args.record_video else "--no-video",
        "--gui" if args.gui else "--headless",
        "--_worker",
    ]
    if args.policy_variant in (DEFAULT_POLICY_VARIANT, TRAJECTORY_POLICY_VARIANT):
        command.extend(
            (
                "--lookahead-m",
                str(parameters["lookahead_m"]),
                "--yaw-rate-feedback-gain-s",
                str(parameters["yaw_rate_feedback_gain_s"]),
            )
        )
    if args.policy_variant == TRAJECTORY_POLICY_VARIANT:
        for name in TRAJECTORY_POLICY_PARAMETER_NAMES:
            if name in {"lookahead_m", "yaw_rate_feedback_gain_s"}:
                continue
            flag = "--" + name.replace("_", "-")
            command.extend((flag, str(parameters[name])))
    elif args.policy_variant == TRAJECTORY_POLICY_V2_VARIANT:
        for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
            if not bool(getattr(args, f"_{name}_provided", False)):
                continue
            flag = "--" + name.replace("_", "-")
            command.extend((flag, str(parameters[name])))
    if args.fixture is not None:
        command.extend(("--fixture", str(args.fixture)))
    if args.auxiliary_target_rate_deg_s is not None:
        command.extend(
            ("--auxiliary-target-rate-deg-s", str(args.auxiliary_target_rate_deg_s))
        )
    if args.auxiliary_velocity_limit_rad_s is not None:
        command.extend(
            ("--auxiliary-velocity-limit-rad-s", str(args.auxiliary_velocity_limit_rad_s))
        )
    return command


_SUMMARY_INPUT_HASH_NAMES = (
    "fixture_sha256",
    "occupancy_sha256",
    "contract_sha256",
    "drive_profile_sha256",
)
_SUMMARY_V2_INPUT_HASH_NAMES = (
    *_SUMMARY_INPUT_HASH_NAMES,
    "tracking_config_sha256",
)
_SUMMARY_SOURCE_HASH_NAMES = (
    "harness_sha256",
    "geometry_sha256",
    "policy_sha256",
    "runner_sha256",
    "trajectory_profile_sha256",
)
_SUMMARY_ARTIFACT_HASH_NAMES = (
    *_SUMMARY_INPUT_HASH_NAMES,
    *_SUMMARY_SOURCE_HASH_NAMES,
)


def _summary_input_hash_names(policy_variant: str) -> tuple[str, ...]:
    return (
        _SUMMARY_V2_INPUT_HASH_NAMES
        if policy_variant == TRAJECTORY_POLICY_V2_VARIANT
        else _SUMMARY_INPUT_HASH_NAMES
    )


def _summary_sha256(value: object, *, label: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise DiagnosticValidationError(f"{label} must be a lowercase SHA-256 digest")
    return value


def _summary_parameters_match(
    payload: Mapping[str, object],
    expected: Mapping[str, float],
    *,
    label: str,
) -> None:
    for name, expected_value in expected.items():
        if name not in payload:
            raise DiagnosticValidationError(f"{label} is missing {name}")
        value = payload[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DiagnosticValidationError(f"{label}.{name} must be finite numeric")
        numeric = float(value)
        if not math.isfinite(numeric):
            raise DiagnosticValidationError(f"{label}.{name} must be finite numeric")
        if not math.isclose(numeric, expected_value, rel_tol=0.0, abs_tol=1e-12):
            raise DiagnosticValidationError(
                f"{label}.{name} does not match requested policy parameters"
            )


def _summary_parameter_mapping(
    payload: object,
    expected: Mapping[str, float],
    *,
    label: str,
) -> None:
    """Require one exact, finite parameter mapping for summary evidence."""

    if not isinstance(payload, Mapping):
        raise DiagnosticValidationError(f"{label} must be an object")
    expected_names = set(expected)
    actual_names = set(payload)
    missing = sorted(expected_names - actual_names)
    extra = sorted(actual_names - expected_names)
    if missing or extra:
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if extra:
            details.append("extra=" + ",".join(extra))
        raise DiagnosticValidationError(
            f"{label} must contain exactly the requested parameters ("
            + "; ".join(details)
            + ")"
        )
    _summary_parameters_match(payload, expected, label=label)


def _summary_number_match(
    actual: object,
    expected: float,
    *,
    label: str,
) -> None:
    if isinstance(actual, bool) or not isinstance(actual, (int, float)):
        raise DiagnosticValidationError(f"{label} must be finite numeric")
    numeric = float(actual)
    if not math.isfinite(numeric):
        raise DiagnosticValidationError(f"{label} must be finite numeric")
    if not math.isclose(numeric, float(expected), rel_tol=0.0, abs_tol=1e-12):
        raise DiagnosticValidationError(f"{label} does not match expected value")


def _summary_sequence_match(
    actual: object,
    expected: Sequence[object],
    *,
    label: str,
) -> None:
    if not isinstance(actual, (list, tuple)) or len(actual) != len(expected):
        raise DiagnosticValidationError(f"{label} does not match expected sequence")
    for index, (value, expected_value) in enumerate(zip(actual, expected)):
        if isinstance(expected_value, bool):
            if not isinstance(value, bool) or value != expected_value:
                raise DiagnosticValidationError(
                    f"{label}[{index}] does not match expected value"
                )
        elif isinstance(expected_value, (int, np.integer)) and not isinstance(
            expected_value, bool
        ):
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
                raise DiagnosticValidationError(
                    f"{label}[{index}] must be an integer"
                )
            if int(value) != int(expected_value):
                raise DiagnosticValidationError(
                    f"{label}[{index}] does not match expected value"
                )
        elif isinstance(expected_value, str):
            if not isinstance(value, str) or value != expected_value:
                raise DiagnosticValidationError(
                    f"{label}[{index}] does not match expected value"
                )
        else:
            _summary_number_match(value, float(expected_value), label=f"{label}[{index}]")


def _summary_validate_trajectory_diagnostics(
    payload: object,
    *,
    label: str,
    action_blocks: int | None = None,
    v2: bool = False,
) -> None:
    """Require complete finite action-aligned diagnostics for V2 evidence."""

    if not isinstance(payload, Mapping):
        raise DiagnosticValidationError(f"{label} must be an object")
    expected_names = set(TRAJECTORY_DIAGNOSTIC_NAMES)
    if v2:
        expected_names.update(TRAJECTORY_POLICY_V2_DIAGNOSTIC_NAMES)
    actual_names = set(payload)
    missing = sorted(expected_names - actual_names)
    extra = sorted(actual_names - expected_names)
    if missing or extra:
        details = []
        if missing:
            details.append("missing=" + ",".join(missing))
        if extra:
            details.append("extra=" + ",".join(extra))
        raise DiagnosticValidationError(
            f"{label} must contain exactly the V2 diagnostics ("
            + "; ".join(details)
            + ")"
        )
    for name in expected_names:
        values = payload[name]
        if not isinstance(values, (list, tuple)):
            raise DiagnosticValidationError(
                f"{label}.{name} must be a finite trace"
            )
        if action_blocks is not None and len(values) != action_blocks:
            raise DiagnosticValidationError(
                f"{label}.{name} length must match action_blocks={action_blocks}"
            )
        if action_blocks is None and not values:
            raise DiagnosticValidationError(
                f"{label}.{name} must be a non-empty finite trace"
            )
        for index, value in enumerate(values):
            if name == "budget_infeasible":
                if not isinstance(value, bool):
                    raise DiagnosticValidationError(
                        f"{label}.{name}[{index}] must be boolean"
                    )
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise DiagnosticValidationError(
                    f"{label}.{name}[{index}] must be finite numeric"
                )
            if not math.isfinite(float(value)):
                raise DiagnosticValidationError(
                    f"{label}.{name}[{index}] must be finite numeric"
                )


def _summary_validate_terminal_events(
    payload: Mapping[str, object],
    *,
    label: str,
) -> dict[str, float | None]:
    """Validate runner-owned V2 terminal event timing in summary evidence."""

    events: dict[str, float | None] = {}
    for name in _V2_RUNNER_EVENT_FIELDS.values():
        if name not in payload:
            raise DiagnosticValidationError(f"{label}.{name} is missing")
        value = payload[name]
        if value is None:
            events[name] = None
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DiagnosticValidationError(
                f"{label}.{name} must be finite and non-negative or null"
            )
        numeric = float(value)
        if not math.isfinite(numeric) or numeric < 0.0:
            raise DiagnosticValidationError(
                f"{label}.{name} must be finite and non-negative or null"
            )
        events[name] = numeric
    brake = events["terminal_brake_start_time_s"]
    goal = events["goal_entry_time_s"]
    stop = events["stop_below_speed_time_s"]
    hold = events["hold_complete_time_s"]
    if brake is not None and goal is not None and brake > goal + 1e-12:
        raise DiagnosticValidationError(
            f"{label} event ordering is invalid: brake start follows goal entry"
        )
    if stop is not None and goal is None:
        raise DiagnosticValidationError(
            f"{label} event ordering is invalid: low speed lacks goal entry"
        )
    if stop is not None and goal is not None and stop < goal - 1e-12:
        raise DiagnosticValidationError(
            f"{label} event ordering is invalid: low speed precedes goal entry"
        )
    if hold is not None and stop is None:
        raise DiagnosticValidationError(
            f"{label} event ordering is invalid: hold complete lacks low speed"
        )
    if hold is not None and stop is not None and hold < stop - 1e-12:
        raise DiagnosticValidationError(
            f"{label} event ordering is invalid: hold complete precedes low speed"
        )
    return events


def _summary_expected_source_paths() -> dict[str, Path]:
    return {
        "harness_sha256": _HERE,
        "geometry_sha256": _PROJECT_ROOT / "benchmark" / "tracking_acceptance.py",
        "policy_sha256": _PROJECT_ROOT / "benchmark" / "policies.py",
        "runner_sha256": _PROJECT_ROOT / "benchmark" / "runner.py",
        "trajectory_profile_sha256": (
            _PROJECT_ROOT / "benchmark" / "trajectory_tracking.py"
        ),
    }


def _summary_source_hashes() -> dict[str, str]:
    source_paths = {
        "policy_sha256": _PROJECT_ROOT / "benchmark" / "policies.py",
        "runner_sha256": _PROJECT_ROOT / "benchmark" / "runner.py",
        "trajectory_profile_sha256": (
            _PROJECT_ROOT / "benchmark" / "trajectory_tracking.py"
        ),
        "harness_sha256": _HERE,
        "geometry_sha256": _PROJECT_ROOT / "benchmark" / "tracking_acceptance.py",
    }
    return {name: _sha256(path) for name, path in source_paths.items()}


def _summary_verify_referenced_hashes(
    config_payload: Mapping[str, object],
    declared_hashes: Mapping[str, str],
    *,
    require_tracking_config: bool = False,
) -> dict[str, str]:
    """Require and verify every frozen input/source reference in a case."""

    def required_mapping(value: object, label: str) -> Mapping[str, object]:
        if not isinstance(value, Mapping):
            raise DiagnosticValidationError(f"{label} must be an object")
        return value

    def verify_file(
        container: Mapping[str, object],
        *,
        path_key: str,
        nested_hash_key: str,
        hash_name: str,
        label: str,
        expected_path: Path | None = None,
    ) -> tuple[str, str]:
        raw_path = container.get(path_key)
        if not isinstance(raw_path, str) or not raw_path:
            raise DiagnosticValidationError(f"{label}.{path_key} must be a file path")
        path = Path(raw_path).expanduser().resolve()
        nested_hash = _summary_sha256(
            container.get(nested_hash_key),
            label=f"{label}.{nested_hash_key}",
        )
        if nested_hash != declared_hashes[hash_name]:
            raise DiagnosticValidationError(
                f"{label}.{nested_hash_key} disagrees with config.{hash_name}"
            )
        if expected_path is not None and path != expected_path.expanduser().resolve():
            raise DiagnosticValidationError(
                f"{label}.{path_key} does not reference the frozen source"
            )
        if not path.is_file():
            raise DiagnosticValidationError(f"{label}.{path_key} is not a file")
        try:
            actual = _sha256(path)
        except (OSError, ValueError, TypeError) as error:
            raise DiagnosticValidationError(f"could not hash {label}.{path_key}") from error
        if actual != nested_hash:
            raise DiagnosticValidationError(
                f"{label}.{nested_hash_key} does not match its referenced file"
            )
        return str(path), nested_hash

    fixture_payload = required_mapping(config_payload.get("fixture"), "config.fixture")
    contract_payload = required_mapping(
        config_payload.get("contract"), "config.contract"
    )
    profile_payload = required_mapping(
        config_payload.get("drive_profile"), "config.drive_profile"
    )
    source_payload = required_mapping(config_payload.get("source"), "config.source")

    paths: dict[str, str] = {}
    paths["fixture_path"], _ = verify_file(
        fixture_payload,
        path_key="scene_path",
        nested_hash_key="scene_sha256",
        hash_name="fixture_sha256",
        label="config.fixture",
    )
    paths["occupancy_path"], _ = verify_file(
        fixture_payload,
        path_key="occupancy_path",
        nested_hash_key="occupancy_sha256",
        hash_name="occupancy_sha256",
        label="config.fixture",
    )
    paths["contract_path"], _ = verify_file(
        contract_payload,
        path_key="path",
        nested_hash_key="sha256",
        hash_name="contract_sha256",
        label="config.contract",
    )
    paths["drive_profile_path"], _ = verify_file(
        profile_payload,
        path_key="path",
        nested_hash_key="sha256",
        hash_name="drive_profile_sha256",
        label="config.drive_profile",
    )
    if require_tracking_config:
        tracking_payload = required_mapping(
            config_payload.get("tracking_config"), "config.tracking_config"
        )
        paths["tracking_config_path"], _ = verify_file(
            tracking_payload,
            path_key="path",
            nested_hash_key="sha256",
            hash_name="tracking_config_sha256",
            label="config.tracking_config",
        )
        top_level = _summary_sha256(
            config_payload.get("tracking_config_sha256"),
            label="config.tracking_config_sha256",
        )
        if top_level != declared_hashes["tracking_config_sha256"]:
            raise DiagnosticValidationError(
                "config.tracking_config_sha256 disagrees with config.tracking_config"
            )
    for name, expected_path in _summary_expected_source_paths().items():
        path_key = name.removesuffix("_sha256") + "_path"
        paths[path_key], _ = verify_file(
            source_payload,
            path_key=path_key,
            nested_hash_key=name,
            hash_name=name,
            label="config.source",
            expected_path=expected_path,
        )
    return paths


def _summary_validate_v2_config_authority(
    config_payload: Mapping[str, object],
    metrics_payload: Mapping[str, object],
    referenced_paths: Mapping[str, str],
    summary_parameters: Mapping[str, float],
) -> dict[str, object]:
    """Reconstruct V2 parameters from the referenced raw config evidence."""

    tracking_payload = config_payload.get("tracking_config")
    if not isinstance(tracking_payload, Mapping):
        raise DiagnosticValidationError("config.tracking_config must be an object")
    harness_payload = config_payload.get("harness")
    if not isinstance(harness_payload, Mapping):
        raise DiagnosticValidationError(
            "config.harness is required for V2 config authority"
        )
    selected_path = harness_payload.get("config_path")
    tracking_path = tracking_payload.get("path")
    if not isinstance(selected_path, str) or not selected_path:
        raise DiagnosticValidationError(
            "config.harness.config_path must preserve selected args.config"
        )
    if not isinstance(tracking_path, str) or not tracking_path:
        raise DiagnosticValidationError("config.tracking_config.path is required")
    # Compare the recorded path spelling before resolving symlinks.  A
    # path-only rewrite must be visible even when an alternate path aliases
    # the same bytes; the later resolved-path check still binds it to the
    # file that was actually hashed.
    selected_absolute = Path(selected_path).expanduser().absolute()
    tracking_absolute = Path(tracking_path).expanduser().absolute()
    if selected_absolute != tracking_absolute:
        raise DiagnosticValidationError(
            "config.harness.config_path disagrees with config.tracking_config.path"
        )
    selected_resolved = Path(selected_path).expanduser().resolve()
    tracking_resolved = Path(tracking_path).expanduser().resolve()
    referenced_resolved = Path(
        referenced_paths["tracking_config_path"]
    ).expanduser().resolve()
    if selected_resolved != referenced_resolved:
        raise DiagnosticValidationError(
            "config.harness.config_path disagrees with referenced raw config"
        )

    try:
        raw_payload = _read_json_object(referenced_resolved)
        raw_policy = _v2_raw_policy_block_from_payload(raw_payload)
        raw_parameters = _v2_parameters_from_payload(raw_payload)
    except (OSError, UnicodeError, ValueError) as error:
        raise DiagnosticValidationError(
            "could not parse referenced raw V2 tracking config"
        ) from error
    stored_raw_policy = tracking_payload.get("raw_policy")
    if not isinstance(stored_raw_policy, Mapping):
        raise DiagnosticValidationError(
            "config.tracking_config.raw_policy is required"
        )
    try:
        raw_matches = _canonical_json_bytes(dict(stored_raw_policy)) == _canonical_json_bytes(
            raw_policy
        )
    except (TypeError, ValueError) as error:
        raise DiagnosticValidationError(
            "config.tracking_config.raw_policy is malformed"
        ) from error
    if not raw_matches:
        raise DiagnosticValidationError(
            "config.tracking_config.raw_policy does not match raw config authority"
        )

    try:
        override_metadata = _v2_override_metadata(
            tracking_payload,
            allow_absent=False,
        )
        override_values = override_metadata["override_values"]
        reconstructed = _resolve_policy_parameters(
            TRAJECTORY_POLICY_V2_VARIANT,
            {**raw_parameters, **override_values},
        )
    except (TypeError, ValueError, KeyError) as error:
        raise DiagnosticValidationError(
            "config.tracking_config override provenance is malformed"
        ) from error

    def verify_parameters(payload: object, label: str) -> None:
        try:
            _summary_parameter_mapping(payload, reconstructed, label=label)
        except DiagnosticValidationError:
            raise

    verify_parameters(
        {name: config_payload.get(name) for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES},
        "config reconstructed V2 parameters",
    )
    config_policy = config_payload.get("trajectory_policy")
    if not isinstance(config_policy, Mapping):
        raise DiagnosticValidationError("config.trajectory_policy is required for V2")
    verify_parameters(
        config_policy.get("parameters"),
        "config.trajectory_policy reconstructed V2 parameters",
    )
    verify_parameters(
        {name: metrics_payload.get(name) for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES},
        "metrics reconstructed V2 parameters",
    )
    metrics_policy = metrics_payload.get("trajectory_policy")
    if not isinstance(metrics_policy, Mapping):
        raise DiagnosticValidationError("metrics.trajectory_policy is required for V2")
    verify_parameters(
        metrics_policy.get("parameters"),
        "metrics.trajectory_policy reconstructed V2 parameters",
    )
    verify_parameters(summary_parameters, "summary reconstructed V2 parameters")
    return {
        "path": str(tracking_resolved),
        "sha256": tracking_payload.get("sha256"),
        "raw_policy": raw_policy,
        **override_metadata,
    }


def _summary(
    run_root: Path,
    cases: Sequence[TrackingCase],
    *,
    policy_variant: str = DEFAULT_POLICY_VARIANT,
    lookahead_m: float | None = None,
    yaw_rate_feedback_gain_s: float | None = None,
    smoothing_window_m: float | None = None,
    curvature_preview_m: float | None = None,
    lateral_curvature_gain_1_m2: float | None = None,
    heading_curvature_gain_1_m: float | None = None,
    max_feedback_curvature_1_m: float | None = None,
    latency_preview_s: float | None = None,
    auxiliary_latency_s: float | None = None,
    maximum_delay_distance_m: float | None = None,
    profile_window_m: float | None = None,
    profile_blend_start_1_m: float | None = None,
    profile_blend_full_1_m: float | None = None,
    feedback_lookahead_m: float | None = None,
    maximum_feedback_curvature_1_m: float | None = None,
    supported_main_command_rad_s: float | None = None,
    episode_budget_s: float | None = None,
    terminal_brake_buffer_m: float | None = None,
) -> Path:
    variant = _validate_policy_variant(policy_variant)
    if not cases:
        raise DiagnosticValidationError("summary requires at least one selected case")
    case_ids = [case.case_id for case in cases]
    if len(set(case_ids)) != len(case_ids):
        raise DiagnosticValidationError("summary cases must have unique case ids")
    summary_values: dict[str, object] = {
        "lookahead_m": lookahead_m,
        "smoothing_window_m": smoothing_window_m,
        "curvature_preview_m": curvature_preview_m,
        "lateral_curvature_gain_1_m2": lateral_curvature_gain_1_m2,
        "heading_curvature_gain_1_m": heading_curvature_gain_1_m,
        "max_feedback_curvature_1_m": max_feedback_curvature_1_m,
        "yaw_rate_feedback_gain_s": yaw_rate_feedback_gain_s,
        "latency_preview_s": latency_preview_s,
        "auxiliary_latency_s": auxiliary_latency_s,
        "maximum_delay_distance_m": maximum_delay_distance_m,
        "profile_window_m": profile_window_m,
        "profile_blend_start_1_m": profile_blend_start_1_m,
        "profile_blend_full_1_m": profile_blend_full_1_m,
        "feedback_lookahead_m": feedback_lookahead_m,
        "maximum_feedback_curvature_1_m": maximum_feedback_curvature_1_m,
        "supported_main_command_rad_s": supported_main_command_rad_s,
        "episode_budget_s": episode_budget_s,
        "terminal_brake_buffer_m": terminal_brake_buffer_m,
    }
    if variant == TRAJECTORY_POLICY_V2_VARIANT:
        # A direct summary call with no CLI parameters still follows the
        # selected case's frozen V2 config.  The coordinator passes explicit
        # values, so those values remain the intentional override path.
        authority: dict[str, float] | None = None
        first_config_path = run_root / cases[0].case_id / "config.json"
        if first_config_path.is_file():
            try:
                authority = _v2_parameters_from_payload(
                    _read_json_object(first_config_path)
                )
            except (OSError, UnicodeError, ValueError):
                # The complete config validation below provides the precise
                # fail-closed diagnostic for malformed case evidence.
                authority = None
        if authority is not None:
            for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
                if summary_values[name] is None:
                    summary_values[name] = authority[name]
    policy_parameters = _resolve_policy_parameters(variant, summary_values)
    summary_input_hash_names = _summary_input_hash_names(variant)
    summary_artifact_hash_names = (
        *summary_input_hash_names,
        *_SUMMARY_SOURCE_HASH_NAMES,
    )
    source_hashes = _summary_source_hashes()
    items: list[dict[str, object]] = []
    artifact_hashes: dict[str, dict[str, str]] = {
        name: {} for name in summary_artifact_hash_names
    }
    artifact_hashes.update({"config_sha256": {}, "metrics_sha256": {}})
    expected_policy_name = (
        "oracle" if variant == DEFAULT_POLICY_VARIANT else variant
    )
    shared_hashes: dict[str, str] | None = None
    shared_paths: dict[str, str] | None = None
    shared_source_paths: dict[str, str] | None = None
    shared_v2_tracking_config: dict[str, object] | None = None
    required_case_complete: list[bool] = []
    for case in cases:
        case_dir = run_root / case.case_id
        metrics_path = case_dir / "metrics.json"
        if not metrics_path.is_file():
            raise DiagnosticValidationError(f"missing case metrics: {metrics_path}")
        try:
            item = _read_json_object(metrics_path)
        except (OSError, UnicodeError, ValueError) as error:
            raise DiagnosticValidationError(
                f"could not read case metrics evidence: {case.case_id}"
            ) from error

        # Summary evidence is case-bound: a metrics file from another case or
        # policy run must not be silently folded into this requested summary.
        if item.get("case_id") != case.case_id:
            raise DiagnosticValidationError(
                f"metrics case_id does not match requested case {case.case_id}"
            )
        if not isinstance(item.get("probe"), bool) or item["probe"] != case.probe:
            raise DiagnosticValidationError(
                f"metrics probe does not match requested case {case.case_id}"
            )
        if item.get("policy_variant") != variant:
            raise DiagnosticValidationError(
                f"metrics policy_variant does not match requested {variant}"
            )
        if item.get("policy") != expected_policy_name:
            raise DiagnosticValidationError("metrics policy identity does not match")
        v1_names_present = set(TRAJECTORY_POLICY_PARAMETER_NAMES).intersection(item)
        if variant == DEFAULT_POLICY_VARIANT and v1_names_present.difference(
            policy_parameters
        ):
            raise DiagnosticValidationError(
                "baseline metrics must not contain V1-only controller parameters"
            )
        metric_parameters = {
            name: item.get(name) for name in policy_parameters
        }
        _summary_parameter_mapping(
            metric_parameters,
            policy_parameters,
            label="metrics controller parameters",
        )
        episode_id = item.get("episode_id")
        if not isinstance(episode_id, str) or not episode_id:
            raise DiagnosticValidationError("metrics episode_id must be a non-empty string")
        metrics_trajectory_policy = item.get("trajectory_policy")
        if variant == TRAJECTORY_POLICY_V2_VARIANT and metrics_trajectory_policy is None:
            raise DiagnosticValidationError(
                "metrics.trajectory_policy is required for V2 evidence"
            )
        if metrics_trajectory_policy is not None:
            if not isinstance(metrics_trajectory_policy, Mapping):
                raise DiagnosticValidationError("metrics.trajectory_policy must be an object")
            if metrics_trajectory_policy.get("name") != variant:
                raise DiagnosticValidationError(
                    "metrics.trajectory_policy name does not match requested variant"
                )
            nested_metrics_parameters = metrics_trajectory_policy.get("parameters")
            _summary_parameter_mapping(
                nested_metrics_parameters,
                policy_parameters,
                label="metrics.trajectory_policy.parameters",
            )
        if variant == TRAJECTORY_POLICY_V2_VARIANT:
            raw_action_blocks = item.get("action_blocks")
            if (
                isinstance(raw_action_blocks, bool)
                or not isinstance(raw_action_blocks, int)
                or raw_action_blocks < 0
            ):
                raise DiagnosticValidationError(
                    "metrics.action_blocks must be a non-negative integer for V2"
                )
            _summary_validate_trajectory_diagnostics(
                item.get("trajectory_diagnostics"),
                label="metrics.trajectory_diagnostics",
                action_blocks=int(raw_action_blocks),
                v2=True,
            )
            _summary_validate_terminal_events(
                item,
                label="metrics.runner_terminal_events",
            )

        # Require and verify the complete case configuration before copying
        # any hashes into the summary.  This rejects missing, tampered, and
        # cross-case configuration evidence rather than treating it as
        # optional provenance.
        try:
            verify_config_hash(case_dir)
        except (OSError, UnicodeError, DiagnosticValidationError) as error:
            if isinstance(error, DiagnosticValidationError):
                raise
            raise DiagnosticValidationError(
                f"could not verify case config evidence: {case.case_id}"
            ) from error
        config_path = case_dir / "config.json"
        try:
            config_payload = _read_json_object(config_path)
        except (OSError, UnicodeError, ValueError) as error:
            raise DiagnosticValidationError(
                f"could not read case config evidence: {case.case_id}"
            ) from error
        if config_payload.get("schema_version") != "tracking_acceptance_case_v1":
            raise DiagnosticValidationError("config schema_version is not production tracking evidence")
        case_payload = config_payload.get("case")
        if not isinstance(case_payload, Mapping):
            raise DiagnosticValidationError("config.case must be an object")
        if case_payload.get("case_id") != case.case_id:
            raise DiagnosticValidationError(
                f"config case_id does not match requested case {case.case_id}"
            )
        if "case_id" in config_payload and config_payload["case_id"] != case.case_id:
            raise DiagnosticValidationError(
                f"config case_id does not match requested case {case.case_id}"
            )
        if case_payload.get("kind") != case.kind:
            raise DiagnosticValidationError("config case kind does not match requested case")
        if not isinstance(case_payload.get("probe"), bool) or case_payload["probe"] != case.probe:
            raise DiagnosticValidationError("config case probe does not match requested case")
        _summary_number_match(
            case_payload.get("main_command_rad_s"),
            case.main_command_rad_s,
            label="config.case.main_command_rad_s",
        )
        _summary_number_match(
            case_payload.get("diagnostic_budget_s"),
            case.budget_s,
            label="config.case.diagnostic_budget_s",
        )
        _summary_parameter_mapping(
            case_payload.get("parameters"),
            case.parameters,
            label="config.case.parameters",
        )
        if config_payload.get("policy_variant") != variant:
            raise DiagnosticValidationError(
                f"config policy_variant does not match requested {variant}"
            )
        if "diagnostic_budget_s" not in config_payload:
            raise DiagnosticValidationError("config.diagnostic_budget_s is required")
        _summary_number_match(
            config_payload.get("diagnostic_budget_s"),
            case.budget_s,
            label="config.diagnostic_budget_s",
        )
        _summary_number_match(
            config_payload.get("formal_budget_s"),
            35.0,
            label="config.formal_budget_s",
        )
        if variant == DEFAULT_POLICY_VARIANT and set(
            TRAJECTORY_POLICY_PARAMETER_NAMES
        ).difference(policy_parameters).intersection(config_payload):
            raise DiagnosticValidationError(
                "baseline config must not contain V1-only controller parameters"
            )
        config_parameters = {
            name: config_payload.get(name) for name in policy_parameters
        }
        _summary_parameter_mapping(
            config_parameters,
            policy_parameters,
            label="config controller parameters",
        )
        config_policy = config_payload.get("policy")
        if not isinstance(config_policy, Mapping):
            raise DiagnosticValidationError("config.policy must be an object")
        if config_policy.get("name") != expected_policy_name:
            raise DiagnosticValidationError("config policy name does not match requested")
        if config_policy.get("variant") != variant:
            raise DiagnosticValidationError("config policy variant does not match requested")
        config_trajectory_policy = config_payload.get("trajectory_policy")
        if variant == TRAJECTORY_POLICY_V2_VARIANT and config_trajectory_policy is None:
            raise DiagnosticValidationError(
                "config.trajectory_policy is required for V2 evidence"
            )
        if config_trajectory_policy is not None:
            if not isinstance(config_trajectory_policy, Mapping):
                raise DiagnosticValidationError("config.trajectory_policy must be an object")
            if config_trajectory_policy.get("name") != variant:
                raise DiagnosticValidationError(
                    "config.trajectory_policy name does not match requested variant"
                )
            nested_config_parameters = config_trajectory_policy.get("parameters")
            _summary_parameter_mapping(
                nested_config_parameters,
                policy_parameters,
                label="config.trajectory_policy.parameters",
            )

        reference = build_tracking_reference(case)
        reference_payload = config_payload.get("reference")
        if not isinstance(reference_payload, Mapping):
            raise DiagnosticValidationError("config.reference must be an object")
        _summary_number_match(
            reference_payload.get("path_length_m"),
            reference.path_length_m,
            label="config.reference.path_length_m",
        )
        point_count = reference_payload.get("point_count")
        if isinstance(point_count, bool) or not isinstance(point_count, int):
            raise DiagnosticValidationError("config.reference.point_count must be an integer")
        if point_count != int(reference.points_xy.shape[0]):
            raise DiagnosticValidationError("config.reference.point_count does not match reference")
        _summary_sequence_match(
            reference_payload.get("segment_boundaries"),
            reference.segment_boundaries,
            label="config.reference.segment_boundaries",
        )
        _summary_sequence_match(
            reference_payload.get("segment_directions"),
            reference.segment_directions,
            label="config.reference.segment_directions",
        )
        _summary_sequence_match(
            reference_payload.get("stop_marker_indices"),
            reference.stop_marker_indices,
            label="config.reference.stop_marker_indices",
        )

        declared_hashes: dict[str, str] = {}
        for name in summary_input_hash_names:
            declared_hashes[name] = _summary_sha256(
                config_payload.get(name), label=f"config.{name}"
            )
        source_payload = config_payload.get("source")
        if not isinstance(source_payload, Mapping):
            raise DiagnosticValidationError("config.source must be an object")
        for name in _SUMMARY_SOURCE_HASH_NAMES:
            declared_hashes[name] = _summary_sha256(
                source_payload.get(name), label=f"config.source.{name}"
            )
            if name in config_payload:
                top_level = _summary_sha256(
                    config_payload[name], label=f"config.{name}"
                )
                if top_level != declared_hashes[name]:
                    raise DiagnosticValidationError(
                        f"config.{name} disagrees with config.source.{name}"
                    )
        if variant == TRAJECTORY_POLICY_V2_VARIANT:
            metrics_tracking_config_hash = _summary_sha256(
                item.get("tracking_config_sha256"),
                label="metrics.tracking_config_sha256",
            )
            if metrics_tracking_config_hash != declared_hashes[
                "tracking_config_sha256"
            ]:
                raise DiagnosticValidationError(
                    "metrics.tracking_config_sha256 does not match config"
                )
        referenced_paths = _summary_verify_referenced_hashes(
            config_payload,
            declared_hashes,
            require_tracking_config=variant == TRAJECTORY_POLICY_V2_VARIANT,
        )
        current_v2_tracking_config: dict[str, object] | None = None
        if variant == TRAJECTORY_POLICY_V2_VARIANT:
            current_v2_tracking_config = _summary_validate_v2_config_authority(
                config_payload,
                item,
                referenced_paths,
                policy_parameters,
            )
        for name, expected_hash in source_hashes.items():
            if declared_hashes[name] != expected_hash:
                raise DiagnosticValidationError(
                    f"config.source.{name} does not match current source"
                )

        episode_payload = config_payload.get("episode")
        if not isinstance(episode_payload, Mapping):
            raise DiagnosticValidationError("config.episode must be an object")
        expected_episode_id = f"tracking_acceptance_{case.case_id}"
        if episode_id != expected_episode_id:
            raise DiagnosticValidationError("metrics episode_id does not match requested case")
        if episode_payload.get("schema_version") != "benchmark_episode_v1":
            raise DiagnosticValidationError("config.episode schema_version is invalid")
        if episode_payload.get("episode_id") != expected_episode_id:
            raise DiagnosticValidationError("config.episode episode_id does not match requested case")
        if episode_payload.get("split") != "dev" or episode_payload.get("difficulty") != "E0":
            raise DiagnosticValidationError("config.episode split/difficulty do not match tracking acceptance")
        scene_uri = episode_payload.get("scene_uri")
        if not isinstance(scene_uri, str) or not scene_uri.startswith("${SCENE_ROOT}/"):
            raise DiagnosticValidationError("config.episode.scene_uri must use SCENE_ROOT")
        scene_relative = scene_uri[len("${SCENE_ROOT}/") :]
        if (
            not scene_relative
            or ".." in Path(scene_relative).parts
            or "\\" in scene_relative
        ):
            raise DiagnosticValidationError("config.episode.scene_uri is unsafe")
        if Path(scene_relative).name != Path(referenced_paths["fixture_path"]).name:
            raise DiagnosticValidationError("config.episode.scene_uri does not match fixture")
        episode_scene_hash = _summary_sha256(
            episode_payload.get("scene_sha256"), label="config.episode.scene_sha256"
        )
        episode_occupancy_hash = _summary_sha256(
            episode_payload.get("occupancy_sha256"),
            label="config.episode.occupancy_sha256",
        )
        if episode_scene_hash != declared_hashes["fixture_sha256"]:
            raise DiagnosticValidationError("config.episode.scene_sha256 does not match fixture")
        if episode_occupancy_hash != declared_hashes["occupancy_sha256"]:
            raise DiagnosticValidationError("config.episode.occupancy_sha256 does not match occupancy")
        expected_start = (
            float(reference.points_xy[0, 0]),
            float(reference.points_xy[0, 1]),
            0.45,
        )
        expected_goal = (
            float(reference.points_xy[-1, 0]),
            float(reference.points_xy[-1, 1]),
            0.45,
        )
        _summary_sequence_match(
            episode_payload.get("start_position_world_m"),
            expected_start,
            label="config.episode.start_position_world_m",
        )
        _summary_sequence_match(
            episode_payload.get("goal_position_world_m"),
            expected_goal,
            label="config.episode.goal_position_world_m",
        )
        _summary_sequence_match(
            episode_payload.get("start_quaternion_world_wxyz"),
            _yaw_quaternion(float(reference.yaw_rad[0])),
            label="config.episode.start_quaternion_world_wxyz",
        )
        euclidean_distance = float(
            np.linalg.norm(reference.points_xy[-1] - reference.points_xy[0])
        )
        _summary_number_match(
            episode_payload.get("euclidean_distance_m"),
            euclidean_distance,
            label="config.episode.euclidean_distance_m",
        )
        _summary_number_match(
            episode_payload.get("oracle_path_length_m"),
            max(reference.path_length_m, euclidean_distance),
            label="config.episode.oracle_path_length_m",
        )
        episode_seed = episode_payload.get("seed")
        if isinstance(episode_seed, bool) or not isinstance(episode_seed, int) or episode_seed < 0:
            raise DiagnosticValidationError("config.episode.seed must be a non-negative integer")
        review_payload = episode_payload.get("human_review")
        if not isinstance(review_payload, Mapping):
            raise DiagnosticValidationError("config.episode.human_review must be an object")
        if review_payload.get("status") != "approved" or review_payload.get("reviewer") != "tracking_acceptance":
            raise DiagnosticValidationError("config.episode.human_review identity is invalid")
        classification = episode_payload.get("classification_evidence")
        if not isinstance(classification, Mapping):
            raise DiagnosticValidationError("config.episode.classification_evidence must be an object")
        if (
            classification.get("tracking_acceptance") is not True
            or classification.get("case_id") != case.case_id
            or classification.get("fixture") != "all_free_open_plane"
        ):
            raise DiagnosticValidationError("config.episode classification evidence does not match case")

        config_sha256 = _sha256(config_path)
        try:
            config_digest_sha256 = _summary_sha256(
                (case_dir / "config.sha256").read_text(encoding="utf-8").strip(),
                label="config.sha256",
            )
        except (OSError, UnicodeError) as error:
            raise DiagnosticValidationError("config.sha256 is unavailable") from error
        if "config_sha256" in item:
            metrics_config_sha256 = _summary_sha256(
                item["config_sha256"], label="metrics.config_sha256"
            )
            if metrics_config_sha256 != config_sha256:
                raise DiagnosticValidationError(
                    "metrics.config_sha256 does not match config.json"
                )
        elif variant in (TRAJECTORY_POLICY_VARIANT, TRAJECTORY_POLICY_V2_VARIANT):
            raise DiagnosticValidationError(
                "metrics.config_sha256 is required for trajectory evidence"
            )
        if config_digest_sha256 != config_sha256:
            raise DiagnosticValidationError("config.sha256 does not match config.json")

        case_shared_hashes = {
            name: declared_hashes[name]
            for name in (*summary_input_hash_names, *_SUMMARY_SOURCE_HASH_NAMES)
        }
        if shared_hashes is None:
            shared_hashes = case_shared_hashes
            shared_paths = dict(referenced_paths)
            shared_v2_tracking_config = current_v2_tracking_config
            shared_source_paths = {
                name: referenced_paths[name]
                for name in (
                    "harness_path",
                    "geometry_path",
                    "policy_path",
                    "runner_path",
                    "trajectory_profile_path",
                )
            }
        else:
            if case_shared_hashes != shared_hashes:
                raise DiagnosticValidationError(
                    "selected cases do not share frozen input/source hashes"
                )
            if shared_source_paths != {
                name: referenced_paths[name]
                for name in (
                    "harness_path",
                    "geometry_path",
                    "policy_path",
                    "runner_path",
                    "trajectory_profile_path",
                )
            }:
                raise DiagnosticValidationError(
                    "selected cases do not share frozen source paths"
                )
            if variant == TRAJECTORY_POLICY_V2_VARIANT:
                if shared_v2_tracking_config is None:
                    raise DiagnosticValidationError(
                        "missing shared V2 tracking config provenance"
                    )
                if current_v2_tracking_config != shared_v2_tracking_config:
                    raise DiagnosticValidationError(
                        "selected cases do not share V2 tracking config provenance"
                    )

        provenance: dict[str, object] = {
            "metrics_sha256": _sha256(metrics_path),
            "config_sha256": config_sha256,
            "config_digest_sha256": config_digest_sha256,
            "referenced_paths": referenced_paths,
            **declared_hashes,
        }
        for name, value in declared_hashes.items():
            artifact_hashes[name][case.case_id] = value
        artifact_hashes["config_sha256"][case.case_id] = config_sha256
        artifact_hashes["metrics_sha256"][case.case_id] = _sha256(metrics_path)
        item["artifact_provenance"] = provenance
        items.append(item)
        acceptance_payload = item.get("acceptance")
        if not isinstance(acceptance_payload, Mapping):
            raise DiagnosticValidationError("metrics.acceptance must be an object")
        if not isinstance(acceptance_payload.get("complete"), bool):
            raise DiagnosticValidationError("metrics.acceptance.complete must be boolean")
        required_case_complete.append(
            bool(acceptance_payload["complete"])
            if not case.probe
            else True
        )
    assert shared_hashes is not None
    assert shared_paths is not None
    assert shared_source_paths is not None
    payload = {
        "schema_version": "tracking_acceptance_summary_v1",
        "policy_variant": variant,
        "formal_budget_s": 35.0,
        "policy_sha256": source_hashes["policy_sha256"],
        "runner_sha256": source_hashes["runner_sha256"],
        "trajectory_profile_sha256": source_hashes["trajectory_profile_sha256"],
        "policy_source_sha256": source_hashes["policy_sha256"],
        "runner_source_sha256": source_hashes["runner_sha256"],
        "trajectory_profile_source_sha256": source_hashes[
            "trajectory_profile_sha256"
        ],
        "harness_sha256": source_hashes["harness_sha256"],
        "geometry_sha256": source_hashes["geometry_sha256"],
        "git_revision": _git_revision(),
        "source_hashes": source_hashes,
        "artifact_hashes": artifact_hashes,
        "fixture_sha256": artifact_hashes["fixture_sha256"],
        "occupancy_sha256": artifact_hashes["occupancy_sha256"],
        "contract_sha256": artifact_hashes["contract_sha256"],
        "drive_profile_sha256": artifact_hashes["drive_profile_sha256"],
        "config_sha256": artifact_hashes["config_sha256"],
        "metrics_sha256": artifact_hashes["metrics_sha256"],
        "case_count": len(items),
        "cases": items,
        "shared_input_hashes": {
            name: shared_hashes[name] for name in summary_input_hash_names
        },
        "shared_source_hashes": {
            name: shared_hashes[name] for name in _SUMMARY_SOURCE_HASH_NAMES
        },
        "shared_referenced_paths": shared_paths,
        "required_cases_complete": all(required_case_complete),
        "probe_cases": [item for item in items if bool(item.get("probe", False))],
    }
    if variant == TRAJECTORY_POLICY_V2_VARIANT:
        payload["tracking_config_sha256"] = artifact_hashes[
            "tracking_config_sha256"
        ]
        if shared_v2_tracking_config is None:
            raise DiagnosticValidationError(
                "missing shared V2 tracking config provenance"
            )
        payload["tracking_config"] = shared_v2_tracking_config
    if variant in (DEFAULT_POLICY_VARIANT, TRAJECTORY_POLICY_VARIANT):
        payload["lookahead_m"] = policy_parameters["lookahead_m"]
        payload["yaw_rate_feedback_gain_s"] = policy_parameters[
            "yaw_rate_feedback_gain_s"
        ]
    if variant == TRAJECTORY_POLICY_VARIANT:
        payload["trajectory_policy"] = {
            "name": TRAJECTORY_POLICY_VARIANT,
            "parameters": dict(policy_parameters),
        }
        for name in TRAJECTORY_POLICY_PARAMETER_NAMES:
            payload[name] = policy_parameters[name]
    elif variant == TRAJECTORY_POLICY_V2_VARIANT:
        payload["trajectory_policy"] = {
            "name": TRAJECTORY_POLICY_V2_VARIANT,
            "parameters": dict(policy_parameters),
        }
        for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES:
            payload[name] = policy_parameters[name]
    destination = run_root / "summary.json"
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite summary: {destination}")
    destination.write_bytes(_canonical_json_bytes(payload))
    return destination


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.seed < 0:
        print("--seed must be non-negative", file=sys.stderr)
        return 2
    if args.gui and args.headless:
        print("choose only one of --gui/--headless", file=sys.stderr)
        return 2
    try:
        policy_parameters = _policy_parameters_from_namespace(args)
        all_cases = load_tracking_cases(args.config)
        cases = (
            tuple(case for case in all_cases if case.case_id == args.case_id)
            if args.case_id is not None
            else all_cases
        )
        if not cases:
            raise ValueError(f"unknown tracking case: {args.case_id!r}")
        if not args._worker and len(cases) > 1:
            for case in cases:
                completed = subprocess.run(_child_command(args, case.case_id), check=False)
                if completed.returncode != 0:
                    raise RuntimeError(f"case worker failed for {case.case_id} with code {completed.returncode}")
            summary = _summary(
                args.run_root,
                cases,
                **_summary_policy_kwargs(args.policy_variant, policy_parameters),
            )
            print(f"Tracking acceptance summary: {summary}")
            return 0
        v2_config_parameters = (
            _v2_explicit_parameters_from_namespace(args)
            if args.policy_variant == TRAJECTORY_POLICY_V2_VARIANT
            else {
                name: policy_parameters.get(name)
                for name in TRAJECTORY_POLICY_V2_PARAMETER_NAMES
            }
        )
        harness_config = load_harness_config(
            args.config,
            scene_root=args.scene_root,
            contract_path=args.contract,
            occupancy_path=args.occupancy,
            fixture_path=args.fixture,
            record_video=args.record_video,
            seed=args.seed,
            policy_variant=args.policy_variant,
            lookahead_m=args.lookahead_m,
            smoothing_window_m=policy_parameters.get("smoothing_window_m"),
            curvature_preview_m=policy_parameters.get("curvature_preview_m"),
            lateral_curvature_gain_1_m2=policy_parameters.get(
                "lateral_curvature_gain_1_m2"
            ),
            heading_curvature_gain_1_m=policy_parameters.get(
                "heading_curvature_gain_1_m"
            ),
            max_feedback_curvature_1_m=policy_parameters.get(
                "max_feedback_curvature_1_m"
            ),
            yaw_rate_feedback_gain_s=args.yaw_rate_feedback_gain_s,
            latency_preview_s=policy_parameters.get("latency_preview_s"),
            auxiliary_latency_s=v2_config_parameters["auxiliary_latency_s"],
            maximum_delay_distance_m=v2_config_parameters[
                "maximum_delay_distance_m"
            ],
            profile_window_m=v2_config_parameters["profile_window_m"],
            profile_blend_start_1_m=v2_config_parameters[
                "profile_blend_start_1_m"
            ],
            profile_blend_full_1_m=v2_config_parameters[
                "profile_blend_full_1_m"
            ],
            feedback_lookahead_m=v2_config_parameters["feedback_lookahead_m"],
            maximum_feedback_curvature_1_m=v2_config_parameters[
                "maximum_feedback_curvature_1_m"
            ],
            supported_main_command_rad_s=v2_config_parameters[
                "supported_main_command_rad_s"
            ],
            episode_budget_s=v2_config_parameters["episode_budget_s"],
            terminal_brake_buffer_m=v2_config_parameters["terminal_brake_buffer_m"],
            auxiliary_target_rate_deg_s=args.auxiliary_target_rate_deg_s,
            auxiliary_velocity_limit_rad_s=args.auxiliary_velocity_limit_rad_s,
        )
        metrics = run_tracking_case(
            cases[0],
            harness_config=harness_config,
            output_root=args.run_root,
            gui=bool(args.gui),
            policy_variant=args.policy_variant,
            **policy_parameters,
        )
        if not args._worker:
            _summary(
                args.run_root,
                cases,
                **_summary_policy_kwargs(args.policy_variant, policy_parameters),
            )
        print(json.dumps(metrics, sort_keys=True, ensure_ascii=False))
        return 0
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        print(f"Tracking acceptance failed: {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
