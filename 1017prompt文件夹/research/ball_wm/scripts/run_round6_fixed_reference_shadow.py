#!/usr/bin/env python3
"""Run the frozen 0002/0004 references with a causal, read-only B0 shadow."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
from typing import Mapping, Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark.boundary_shadow import (  # noqa: E402
    M1TransitionPredictor,
    ReadOnlyBoundaryShadowPolicy,
    SupportEnvelope,
)
from benchmark.drive_model import DriveModel  # noqa: E402
from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig  # noqa: E402
from benchmark.policies import (  # noqa: E402
    OraclePathPolicy,
    Round8ProfileBlendOraclePolicy,
)
from benchmark.runner import RunnerConfig, run_episode  # noqa: E402
from benchmark.tracking_acceptance import polyline_cross_track_errors  # noqa: E402
from scripts.run_round4_fixed_reference import (  # noqa: E402
    EXPECTED_V1_SHA256,
    _plan,
    load_transformed_reference,
)
from scripts.run_tracking_acceptance import (  # noqa: E402
    DEFAULT_CONTRACT,
    DEFAULT_DRIVE_PROFILE,
    DEFAULT_FIXTURE,
    DEFAULT_OCCUPANCY,
    DEFAULT_SCENE_ROOT,
    build_tracking_episode,
    validate_diagnostic_fixture,
)
from training.action_protocol import load_action_protocol  # noqa: E402


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    if path.is_dir():
        for child in sorted(item for item in path.rglob("*") if item.is_file()):
            digest.update(child.relative_to(path).as_posix().encode("utf-8"))
            digest.update(b"\0")
            with child.open("rb") as handle:
                for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                    digest.update(chunk)
        return digest.hexdigest()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _jsonable(value: object) -> object:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def run_shadow(
    *,
    source_states_path: Path,
    case_id: str,
    output_dir: Path,
    model_path: Path,
    support_cloud_path: Path,
    gui: bool,
    shadow_off: bool = False,
    round8_profile_blend: bool = False,
) -> dict[str, object]:
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite Round 6 shadow run: {output_dir}")
    profile = DEFAULT_DRIVE_PROFILE.resolve()
    if _sha256(profile) != EXPECTED_V1_SHA256:
        raise RuntimeError("frozen drive_dynamics_v1 SHA-256 mismatch")
    reference, provenance = load_transformed_reference(source_states_path, case_id=case_id)
    validate_diagnostic_fixture(DEFAULT_FIXTURE.resolve(), DEFAULT_OCCUPANCY.resolve())
    episode = build_tracking_episode(
        reference.case,
        reference,
        scene_root=DEFAULT_SCENE_ROOT.resolve(),
        fixture_path=DEFAULT_FIXTURE.resolve(),
        occupancy_path=DEFAULT_OCCUPANCY.resolve(),
        seed=3072,
    )
    protocol = load_action_protocol(DEFAULT_CONTRACT)
    support = None
    predictor = None
    if not shadow_off:
        with np.load(support_cloud_path, allow_pickle=False) as archive:
            points = np.asarray(archive["state_request_points"], dtype=np.float64)
            maximum_distance = float(archive["maximum_standardized_nearest_neighbor_distance"])
        support = SupportEnvelope.from_points(points, maximum_distance=maximum_distance)
        predictor = M1TransitionPredictor.from_json(model_path)
    output_dir.mkdir(parents=True)
    implementation_paths = {
        "run_round6_fixed_reference_shadow.py": Path(__file__).resolve(),
        "boundary_shadow.py": (Path(__file__).resolve().parents[1] / "benchmark" / "boundary_shadow.py"),
        "policies.py": (Path(__file__).resolve().parents[1] / "benchmark" / "policies.py"),
        "runner.py": (Path(__file__).resolve().parents[1] / "benchmark" / "runner.py"),
        "isaac_runtime.py": (Path(__file__).resolve().parents[1] / "benchmark" / "isaac_runtime.py"),
        "action_smoothing.py": (Path(__file__).resolve().parent / "action_smoothing.py"),
    }
    _atomic_json(
        output_dir / "config.json",
        {
            "schema_version": "round6_online_shadow_run_v1",
            "case_id": case_id,
            "policy": (
                "R8_candidate/profile_blend_after_handoff"
                if round8_profile_blend
                else ("B0/oracle" if shadow_off else "B0/oracle+read_only_shadow")
            ),
            "shadow_enabled": not shadow_off,
            "model_accepted": False,
            "model_role": "diagnostic_prediction_only; advice must abstain",
            "model_path": str(model_path.resolve()),
            "model_sha256": _sha256(model_path),
            "support_cloud_path": str(support_cloud_path.resolve()),
            "support_cloud_sha256": _sha256(support_cloud_path),
            "drive_profile_sha256": _sha256(profile),
            "action_protocol_id": protocol.protocol_id,
            "episode_budget_s": 35.0,
            "replan_hz": 10,
            "physics_hz": 50,
            "shadow_horizon_s": 1.5,
            "scene_sha256": _sha256(DEFAULT_SCENE_ROOT.resolve()),
            "fixture_sha256": _sha256(DEFAULT_FIXTURE.resolve()),
            "occupancy_sha256": _sha256(DEFAULT_OCCUPANCY.resolve()),
            "contract_sha256": _sha256(DEFAULT_CONTRACT.resolve()),
            "implementation_sha256": {
                name: _sha256(path) for name, path in implementation_paths.items()
            },
            "reference": provenance,
        },
    )
    runtime = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(
            scene_root=DEFAULT_SCENE_ROOT.resolve(),
            contract_path=DEFAULT_CONTRACT.resolve(),
            gui=gui,
            record_video=False,
            occupancy_path=DEFAULT_OCCUPANCY.resolve(),
            drive_profile_path=profile,
            diagnostic_open_plane=True,
            diagnostic_physics_trace=True,
            diagnostic_auxiliary_target_rate_rad_s=math.radians(30.0),
            diagnostic_auxiliary_velocity_limit_rad_s=0.30,
        )
    )
    runtime.set_reference_visualization_path(reference.points_xy)
    return_code = 1
    try:
        policy_class = (
            Round8ProfileBlendOraclePolicy
            if round8_profile_blend
            else OraclePathPolicy
        )
        base = policy_class(
            protocol,
            lookahead_m=0.5,
            yaw_rate_feedback_gain_s=0.15,
            drive_model=DriveModel.from_json(profile),
        )
        policy = (
            base
            if shadow_off
            else ReadOnlyBoundaryShadowPolicy(
                base,
                predictor=predictor,
                support_envelope=support,
                model_accepted=False,
                horizon_s=1.5,
            )
        )
        result = run_episode(
            runtime,
            policy,
            episode,
            action_protocol=protocol,
            config=RunnerConfig(
                episode_budget_s=35.0,
                replan_hz=10,
                goal_tolerance_m=0.2,
                diagnostic_settling_budget_s=3.0,
                diagnostic_stop_hold_s=0.5,
                diagnostic_stop_speed_m_s=0.05,
            ),
            oracle_path_xy=reference.points_xy,
            kinodynamic_plan=_plan(reference),
        )
        if result.positions_world_xy.shape[0] == 0:
            raise RuntimeError(result.runtime_error or "shadow run produced no state")
        cross_track = polyline_cross_track_errors(result.positions_world_xy, reference.points_xy)
        np.savez(
            output_dir / "states.npz",
            position_world_xy=result.positions_world_xy,
            yaw_world_rad=result.yaw_world_rad,
            linear_velocity_world_m_s=result.linear_velocity_world_m_s,
            angular_velocity_world_rad_s=result.angular_velocity_world_rad_s,
            joint_position_rad=result.joint_position_rad,
            joint_velocity_rad_s=result.joint_velocity_rad_s,
            action_block=result.actions,
            applied_action=result.applied_action,
            simulation_time_s=result.simulation_time_s,
            policy_reference_index=result.policy_reference_index,
            policy_reference_progress_s_m=result.policy_reference_progress_s_m,
            policy_cross_track_error_m=result.policy_cross_track_error_m,
            policy_active_drive_direction=result.policy_active_drive_direction,
            policy_desired_drive_direction=result.policy_desired_drive_direction,
            policy_pending_drive_direction=result.policy_pending_drive_direction,
            policy_stopped_cycles=result.policy_stopped_cycles,
            policy_braking_state_code=result.policy_braking_state_code,
            cross_track_error_m=cross_track,
            kinodynamic_reference_pose=np.column_stack((reference.points_xy, reference.yaw_rad)),
            kinodynamic_reference_command=np.asarray(
                [(item.main_command_rad_s, item.auxiliary_angle_rad) for item in reference.waypoints]
            ),
            kinodynamic_reference_drive_direction=np.asarray(
                [1 if item.drive_direction == "forward" else -1 for item in reference.waypoints],
                dtype=np.int8,
            ),
        )
        np.savez(output_dir / "physics_diagnostics.npz", **runtime.physics_diagnostics)
        tracking_records = list(base.tracking_records)
        if len(tracking_records) != int(result.action_blocks):
            raise RuntimeError(
                "tracking audit must contain exactly one record per policy action block"
            )
        _atomic_json(
            output_dir / "tracking_audit.json",
            {
                "schema_version": "round8_oracle_tracking_audit_v1",
                "single_policy_call_per_record": True,
                "record_count": len(tracking_records),
                "records": _jsonable(tracking_records),
            },
        )
        contexts = [] if shadow_off else [item.to_dict() for item in policy.contexts]
        records = [] if shadow_off else list(policy.records)
        _atomic_json(
            output_dir / "online_shadow.json",
            {
                "schema_version": "round6_online_shadow_records_v1",
                "shadow_enabled": not shadow_off,
                "causal_inputs": [
                    "current_measured_state",
                    "current_and_past_policy_state",
                    "current_b0_request",
                    "frozen_reference_known_at_current_time",
                    "request_plans_constructed_at_current_time",
                ],
                "forbidden_inputs_used": [],
                "base_policy_act_calls": len(contexts),
                "context_count": len(contexts),
                "contexts": contexts,
                "records": _jsonable(records),
            },
        )
        boundary = [record for record in records if record["boundary_active"]]
        advice_counts: dict[str, int] = {}
        for record in boundary:
            advice = str(record["advice"])
            advice_counts[advice] = advice_counts.get(advice, 0) + 1
        metrics = {
            "schema_version": "round6_online_shadow_metrics_v1",
            "case_id": case_id,
            "termination_reason": result.termination_reason,
            "runtime_error": result.runtime_error,
            "action_blocks": int(result.action_blocks),
            "elapsed_s": float(result.elapsed_s),
            "final_distance_m": float(result.final_distance_m),
            "path_length_m": float(result.path_length_m),
            "reference_length_m": float(reference.path_length_m),
            "reference_progress_final_m": float(result.policy_reference_progress_s_m[-1]),
            "reference_index_final": int(result.policy_reference_index[-1]),
            "cross_track_rms_m": float(np.sqrt(np.mean(np.square(cross_track)))),
            "cross_track_peak_m": float(np.max(cross_track)),
            "actual_direction_change_completed": bool(
                np.any(np.diff(result.policy_active_drive_direction) != 0)
            ),
            "boundary_record_count": len(boundary),
            "shadow_advice_counts": advice_counts,
            "base_policy_act_call_equals_action_blocks": (
                True if shadow_off else len(contexts) == int(result.action_blocks)
            ),
            "shadow_enabled": not shadow_off,
            "reference_replanned": False,
        }
        _atomic_json(output_dir / "metrics.json", metrics)
        return_code = 0
        return metrics
    except BaseException as error:
        _atomic_json(
            output_dir / "failure.json",
            {
                "error_type": type(error).__name__,
                "error": str(error),
                "traceback": traceback.format_exc(),
            },
        )
        raise
    finally:
        runtime.close(return_code=return_code)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-states", type=Path, required=True)
    parser.add_argument("--case-id", required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--support-cloud", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--shadow-off", action="store_true")
    parser.add_argument("--round8-profile-blend", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    metrics = run_shadow(
        source_states_path=args.source_states.resolve(),
        case_id=args.case_id,
        output_dir=args.out.resolve(),
        model_path=args.model.resolve(),
        support_cloud_path=args.support_cloud.resolve(),
        gui=bool(args.gui),
        shadow_off=bool(args.shadow_off),
        round8_profile_blend=bool(args.round8_profile_blend),
    )
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
