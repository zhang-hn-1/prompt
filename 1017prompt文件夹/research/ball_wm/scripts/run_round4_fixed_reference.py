#!/usr/bin/env python3
"""Replay one frozen formal reference in the diagnostic open plane."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
from typing import Sequence

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark.drive_model import DriveModel  # noqa: E402
from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig  # noqa: E402
from benchmark.kinodynamic_planning import (  # noqa: E402
    KinodynamicPlanResult,
    KinodynamicWaypoint,
)
from benchmark.policies import OraclePathPolicy  # noqa: E402
from benchmark.runner import RunnerConfig, run_episode  # noqa: E402
from benchmark.tracking_acceptance import (  # noqa: E402
    TrackingCase,
    TrackingReference,
    polyline_cross_track_errors,
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


EXPECTED_V1_SHA256 = "2de28cdfe98ecd6d45d5158261efb436acabaf31c1df4774f75b708a9086ddfd"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _wrap_angle(value: np.ndarray | float) -> np.ndarray | float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def load_transformed_reference(
    source_states_path: str | Path, *, case_id: str
) -> tuple[TrackingReference, dict[str, object]]:
    """Load stored planner arrays and apply one rigid start-frame transform."""

    source = Path(source_states_path).expanduser().resolve()
    try:
        with np.load(source, allow_pickle=False) as archive:
            pose = np.asarray(archive["kinodynamic_reference_pose"], dtype=np.float64)
            command = np.asarray(
                archive["kinodynamic_reference_command"], dtype=np.float64
            )
            direction_code = np.asarray(
                archive["kinodynamic_reference_drive_direction"], dtype=np.int8
            )
    except (OSError, KeyError, ValueError) as error:
        raise ValueError(f"could not load frozen reference arrays: {source}") from error
    if pose.ndim != 2 or pose.shape[1] != 3 or pose.shape[0] < 2:
        raise ValueError("reference pose must have shape (N, 3)")
    if command.shape != (pose.shape[0], 2) or direction_code.shape != (
        pose.shape[0],
    ):
        raise ValueError("reference pose, command, and direction arrays must align")
    if not np.isfinite(pose).all() or not np.isfinite(command).all():
        raise ValueError("reference arrays must be finite")
    if not np.all(np.isin(direction_code, (-1, 1))):
        raise ValueError("reference direction codes must be -1 or +1")

    origin_xy = pose[0, :2].copy()
    origin_yaw = float(pose[0, 2])
    rotation_angle = -origin_yaw
    cosine = math.cos(rotation_angle)
    sine = math.sin(rotation_angle)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]], dtype=np.float64)
    transformed_xy = (pose[:, :2] - origin_xy) @ rotation.T
    transformed_yaw = np.asarray(_wrap_angle(pose[:, 2] - origin_yaw))
    directions = tuple("forward" if code > 0 else "reverse" for code in direction_code)
    waypoints = tuple(
        KinodynamicWaypoint(
            float(xy[0]),
            float(xy[1]),
            float(yaw),
            direction,
            float(action[0]),
            float(action[1]),
        )
        for xy, yaw, direction, action in zip(
            transformed_xy, transformed_yaw, directions, command
        )
    )
    boundaries = tuple(
        [
            index
            for index in range(len(directions) - 1)
            if directions[index] != directions[index + 1]
        ]
        + [len(directions) - 1]
    )
    segment_starts = (0, *(index + 1 for index in boundaries[:-1]))
    segment_directions = tuple(directions[index] for index in segment_starts)
    case = TrackingCase(
        case_id=f"round4_fixed_{case_id}",
        kind="line",
        main_command_rad_s=0.5,
        budget_s=35.0,
        parameters={
            "start_x_m": 0.0,
            "start_y_m": 0.0,
            "start_yaw_rad": 0.0,
            "length_m": float(
                np.linalg.norm(np.diff(transformed_xy, axis=0), axis=1).sum()
            ),
        },
    )
    reference = TrackingReference(
        case=case,
        points_xy=transformed_xy,
        yaw_rad=transformed_yaw,
        waypoints=waypoints,
        segment_boundaries=boundaries,
        segment_directions=segment_directions,
    )
    provenance = {
        "source_states_path": str(source),
        "source_states_sha256": _sha256(source),
        "source_reference_point_count": int(pose.shape[0]),
        "source_reference_pose_first": pose[0].tolist(),
        "source_reference_pose_last": pose[-1].tolist(),
        "transform": {
            "translation_xy_m": (-origin_xy).tolist(),
            "rotation_rad": rotation_angle,
            "definition": "p_open=R(-yaw0)*(p_source-p0); yaw_open=wrap(yaw_source-yaw0)",
        },
        "command_arrays_unchanged": True,
        "replanning": False,
    }
    return reference, provenance


def _plan(reference: TrackingReference) -> KinodynamicPlanResult:
    return KinodynamicPlanResult(
        status="success",
        waypoints=reference.waypoints,
        expanded_nodes=0,
        path_length_m=reference.path_length_m,
        direction_changes=sum(
            left != right
            for left, right in zip(
                reference.segment_directions, reference.segment_directions[1:]
            )
        ),
    )


def run_fixed_reference(
    *,
    source_states_path: Path,
    case_id: str,
    output_dir: Path,
    gui: bool,
) -> dict[str, object]:
    """Run B0/V1 against one transformed frozen reference without replanning."""

    profile = DEFAULT_DRIVE_PROFILE.resolve()
    if _sha256(profile) != EXPECTED_V1_SHA256:
        raise RuntimeError("frozen drive_dynamics_v1 SHA-256 mismatch")
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite fixed-reference run: {output_dir}")
    reference, provenance = load_transformed_reference(
        source_states_path, case_id=case_id
    )
    fixture = DEFAULT_FIXTURE.resolve()
    occupancy = DEFAULT_OCCUPANCY.resolve()
    scene_root = DEFAULT_SCENE_ROOT.resolve()
    validate_diagnostic_fixture(fixture, occupancy)
    episode = build_tracking_episode(
        reference.case,
        reference,
        scene_root=scene_root,
        fixture_path=fixture,
        occupancy_path=occupancy,
        seed=3072,
    )
    protocol = load_action_protocol(DEFAULT_CONTRACT)
    if protocol.protocol_id != "ball_action_v2_aux30":
        raise ValueError("fixed-reference replay requires ball_action_v2_aux30")
    output_dir.mkdir(parents=True)
    config = {
        "schema_version": "round4_fixed_reference_v1",
        "case_id": case_id,
        "policy": "B0/oracle",
        "drive_profile_path": str(profile),
        "drive_profile_sha256": _sha256(profile),
        "action_protocol_id": protocol.protocol_id,
        "episode_budget_s": 35.0,
        "replan_hz": 10,
        "auxiliary_target_rate_limit_rad_s": math.radians(30.0),
        "auxiliary_velocity_limit_rad_s": 0.30,
        "fixture_path": str(fixture),
        "fixture_sha256": _sha256(fixture),
        "occupancy_path": str(occupancy),
        "occupancy_sha256": _sha256(occupancy),
        "contract_path": str(DEFAULT_CONTRACT.resolve()),
        "contract_sha256": _sha256(DEFAULT_CONTRACT.resolve()),
        "reference": provenance,
    }
    _atomic_json(output_dir / "config.json", config)
    runtime = IsaacBenchmarkRuntime(
        IsaacRuntimeConfig(
            scene_root=scene_root,
            contract_path=DEFAULT_CONTRACT.resolve(),
            gui=gui,
            record_video=False,
            occupancy_path=occupancy,
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
        policy = OraclePathPolicy(
            protocol,
            lookahead_m=0.5,
            yaw_rate_feedback_gain_s=0.15,
            drive_model=DriveModel.from_json(profile),
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
            raise RuntimeError(result.runtime_error or "fixed-reference run produced no state")
        cross_track = polyline_cross_track_errors(
            result.positions_world_xy, reference.points_xy
        )
        state_payload = {
            "position_world_xy": result.positions_world_xy,
            "yaw_world_rad": result.yaw_world_rad,
            "linear_velocity_world_m_s": result.linear_velocity_world_m_s,
            "angular_velocity_world_rad_s": result.angular_velocity_world_rad_s,
            "joint_position_rad": result.joint_position_rad,
            "joint_velocity_rad_s": result.joint_velocity_rad_s,
            "applied_action": result.applied_action,
            "physical_contact": result.physical_contacts,
            "collision": result.collisions,
            "simulation_time_s": result.simulation_time_s,
            "policy_reference_index": result.policy_reference_index,
            "policy_reference_progress_s_m": result.policy_reference_progress_s_m,
            "policy_reference_heading_rad": result.policy_reference_heading_rad,
            "policy_reference_curvature_1_m": result.policy_reference_curvature_1_m,
            "policy_baseline_main_command_rad_s": result.policy_baseline_main_command_rad_s,
            "policy_auxiliary_target_angle_rad": result.policy_auxiliary_target_angle_rad,
            "policy_auxiliary_velocity_rad_s": result.policy_auxiliary_velocity_rad_s,
            "cross_track_error_m": cross_track,
            "kinodynamic_reference_pose": np.column_stack(
                (reference.points_xy, reference.yaw_rad)
            ),
            "kinodynamic_reference_command": np.asarray(
                [
                    (item.main_command_rad_s, item.auxiliary_angle_rad)
                    for item in reference.waypoints
                ]
            ),
            "kinodynamic_reference_drive_direction": np.asarray(
                [1 if item.drive_direction == "forward" else -1 for item in reference.waypoints],
                dtype=np.int8,
            ),
        }
        np.savez(output_dir / "states.npz", **state_payload)
        np.savez(output_dir / "physics_diagnostics.npz", **runtime.physics_diagnostics)
        metrics = {
            "schema_version": "round4_fixed_reference_metrics_v1",
            "case_id": case_id,
            "termination_reason": result.termination_reason,
            "runtime_error": result.runtime_error,
            "action_blocks": int(result.action_blocks),
            "elapsed_s": float(result.elapsed_s),
            "final_distance_m": float(result.final_distance_m),
            "path_length_m": float(result.path_length_m),
            "reference_length_m": float(reference.path_length_m),
            "cross_track_rms_m": float(np.sqrt(np.mean(np.square(cross_track)))),
            "cross_track_peak_m": float(np.max(cross_track)),
            "cross_track_final_m": float(cross_track[-1]),
            "physical_contact_any": bool(np.any(result.physical_contacts)),
            "formal_collision_any": bool(np.any(result.collisions)),
            "reference_replanned": False,
            "drive_profile_sha256": _sha256(profile),
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
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    metrics = run_fixed_reference(
        source_states_path=args.source_states,
        case_id=args.case_id,
        output_dir=args.out,
        gui=args.gui,
    )
    print(json.dumps(metrics, indent=2, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
