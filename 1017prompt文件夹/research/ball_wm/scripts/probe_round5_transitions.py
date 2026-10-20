"""Collect bounded continuous-state Round 5 transition sequences in Isaac Sim.

The module stays importable without Isaac. Simulator imports live only inside
``run_collection`` so manifest and schedule contracts can be tested on CPU.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Mapping, Sequence

import numpy as np

from action_smoothing import (
    ActionSmoother,
    DEFAULT_AUXILIARY_KD,
    DEFAULT_AUXILIARY_KP,
    DEFAULT_AUXILIARY_RATE_RAD_S,
    DEFAULT_MAIN_ACCELERATION_RAD_S2,
    DEFAULT_MAIN_DERIVATIVE_GAIN,
    default_articulation_gains,
    slew_limited_target,
)
from probe_drive_dynamics import ROBOT_ROOT, ROBOT_START, ROBOT_USD, _yaw_from_wxyz


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONTRACT = SCRIPT_DIR.parent / "configs" / "data_contract.yaml"
DEFAULT_PROTOCOL = (
    SCRIPT_DIR.parent / "configs" / "benchmark_v1" / "round5_measurement_protocol.json"
)
DEFAULT_MANIFEST = (
    SCRIPT_DIR.parent / "configs" / "benchmark_v1" / "round5_transition_manifest.json"
)
SCHEMA = "round5_transition_manifest_v1"
EXPECTED_PROTOCOL = "ball_action_v2_aux30"
ROLES = frozenset(("fit_core", "heldout_validation", "repeat_validation"))


@dataclass(frozen=True)
class SequenceStage:
    phase: str
    duration_s: float
    main_request_rad_s: float
    auxiliary_request_rad: float


@dataclass(frozen=True)
class TransitionSequence:
    sequence_id: str
    dataset_role: str
    stages: tuple[SequenceStage, ...]
    repeat_of: str | None = None
    sealed_before_fit: bool = False


@dataclass(frozen=True)
class TransitionManifest:
    source_path: Path
    payload: dict[str, object]
    sequences: tuple[TransitionSequence, ...]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: object, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _load_json(path: Path) -> dict[str, object]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"could not load JSON: {path}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def _atomic_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def load_transition_manifest(
    path: str | Path, protocol: Mapping[str, object]
) -> TransitionManifest:
    """Validate the frozen sequence split, limits, commands, and repetitions."""

    source = Path(path).resolve()
    payload = _load_json(source)
    if payload.get("schema_version") != SCHEMA:
        raise ValueError(f"transition manifest must use {SCHEMA}")
    if payload.get("action_protocol_id") != EXPECTED_PROTOCOL:
        raise ValueError(f"transition manifest must use {EXPECTED_PROTOCOL}")
    if protocol.get("schema_version") != "round5_measurement_protocol_v1":
        raise ValueError("Round 5 measurement protocol is required")
    procedure = payload.get("procedure")
    if not isinstance(procedure, Mapping):
        raise ValueError("transition procedure is required")
    raw_sequences = payload.get("sequences")
    if not isinstance(raw_sequences, list) or not raw_sequences:
        raise ValueError("transition sequences must be a non-empty list")

    sequences: list[TransitionSequence] = []
    seen: set[str] = set()
    for raw in raw_sequences:
        if not isinstance(raw, Mapping):
            raise ValueError("each transition sequence must be an object")
        sequence_id = raw.get("sequence_id")
        role = raw.get("dataset_role")
        if not isinstance(sequence_id, str) or not sequence_id or sequence_id in seen:
            raise ValueError("sequence_id values must be unique non-empty strings")
        if role not in ROLES:
            raise ValueError("dataset_role is invalid")
        raw_stages = raw.get("stages")
        if not isinstance(raw_stages, list) or len(raw_stages) != 2:
            raise ValueError("each sequence requires precondition and transition stages")
        stages: list[SequenceStage] = []
        for index, raw_stage in enumerate(raw_stages):
            if not isinstance(raw_stage, Mapping):
                raise ValueError("each sequence stage must be an object")
            expected_phase = "precondition" if index == 0 else "transition"
            if raw_stage.get("phase") != expected_phase:
                raise ValueError("sequence stages must be precondition then transition")
            duration = _finite(raw_stage.get("duration_s"), "duration_s")
            main = _finite(raw_stage.get("main_request_rad_s"), "main_request_rad_s")
            auxiliary = _finite(
                raw_stage.get("auxiliary_request_rad"), "auxiliary_request_rad"
            )
            if duration <= 0.0 or not -2.0 <= main <= 2.0:
                raise ValueError("stage duration/command lies outside the protocol")
            if abs(auxiliary) > math.radians(30.0) + 1e-12:
                raise ValueError("auxiliary request lies outside the protocol")
            stages.append(
                SequenceStage(expected_phase, duration, main, auxiliary)
            )
        repeat_of = raw.get("repeat_of")
        if repeat_of is not None and not isinstance(repeat_of, str):
            raise ValueError("repeat_of must be a string")
        sealed = bool(raw.get("sealed_before_fit", False))
        if role == "heldout_validation" and not sealed:
            raise ValueError("held-out sequences must be sealed before fit")
        if role == "repeat_validation" and repeat_of is None:
            raise ValueError("repeat validation requires repeat_of")
        if role != "repeat_validation" and repeat_of is not None:
            raise ValueError("repeat_of is reserved for repeat validation")
        seen.add(sequence_id)
        sequences.append(
            TransitionSequence(
                sequence_id=sequence_id,
                dataset_role=str(role),
                stages=tuple(stages),
                repeat_of=repeat_of,
                sealed_before_fit=sealed,
            )
        )

    counts = {role: sum(item.dataset_role == role for item in sequences) for role in ROLES}
    caps = {
        "fit_core": int(procedure.get("maximum_core_sequences", 0)),
        "heldout_validation": int(procedure.get("maximum_heldout_sequences", 0)),
        "repeat_validation": int(procedure.get("maximum_repeat_or_coverage_sequences", 0)),
    }
    for role, count in counts.items():
        if count > caps[role]:
            raise ValueError(f"{role} sequence count exceeds the frozen cap")
    if len(sequences) > int(procedure.get("maximum_total_new_sequences", 0)):
        raise ValueError("total sequence count exceeds the frozen cap")
    core = {item.sequence_id: item for item in sequences if item.dataset_role == "fit_core"}
    for item in sequences:
        if item.repeat_of is None:
            continue
        if item.repeat_of not in core or item.stages != core[item.repeat_of].stages:
            raise ValueError("repeat sequence must exactly match a fit-core sequence")
    horizon = max(float(value) for value in protocol["finite_horizons_s"])  # type: ignore[index]
    if any(item.stages[-1].duration_s + 1e-12 < horizon for item in sequences):
        raise ValueError("transition hold must cover every frozen horizon")
    return TransitionManifest(source, payload, tuple(sequences))


def build_sequence_schedule(
    sequence: TransitionSequence, *, physics_hz: int
) -> list[dict[str, object]]:
    """Expand one sequence while marking the only legal reset boundary."""

    if not isinstance(physics_hz, int) or physics_hz <= 0:
        raise ValueError("physics_hz must be a positive integer")
    schedule: list[dict[str, object]] = []
    for stage_index, stage in enumerate(sequence.stages):
        count = max(1, int(round(stage.duration_s * physics_hz)))
        for step in range(count):
            schedule.append(
                {
                    "phase": stage.phase,
                    "stage_index": stage_index,
                    "stage_step_index": step,
                    "stage_start_flag": step == 0,
                    "main_request_rad_s": stage.main_request_rad_s,
                    "auxiliary_request_rad": stage.auxiliary_request_rad,
                    "reset_flag": stage_index == 0 and step == 0,
                    "smoother_reset_flag": stage_index == 0 and step == 0,
                }
            )
    return schedule


def _controller_metadata(controller: object) -> dict[str, object]:
    requested_kp, requested_kd = default_articulation_gains()
    result: dict[str, object] = {
        "requested_kp": requested_kp.astype(float).tolist(),
        "requested_kd": requested_kd.astype(float).tolist(),
        "readback_kp": None,
        "readback_kd": None,
        "readback_available": False,
        "readback_error": None,
    }
    getter = getattr(controller, "get_gains", None)
    if not callable(getter):
        result["readback_error"] = "controller.get_gains unavailable"
        return result
    try:
        kp, kd = getter()
        kp = np.asarray(kp, dtype=np.float64)
        kd = np.asarray(kd, dtype=np.float64)
        if kp.shape != (3,) or kd.shape != (3,) or not np.isfinite([kp, kd]).all():
            raise ValueError("unexpected gain shape/value")
        result.update(
            {
                "readback_kp": kp.tolist(),
                "readback_kd": kd.tolist(),
                "readback_available": True,
            }
        )
    except Exception as error:  # Isaac versions expose different readback APIs.
        result["readback_error"] = f"{type(error).__name__}: {error}"
    return result


def run_collection(
    *,
    contract_path: Path,
    protocol_path: Path,
    manifest_path: Path,
    output_dir: Path,
    gui: bool,
) -> dict[str, object]:
    """Run every frozen sequence with a full reset only at sequence start."""

    from analyze_round5_motion_transitions import load_measurement_protocol
    from collection_schema import load_contract

    protocol = load_measurement_protocol(protocol_path)
    manifest = load_transition_manifest(manifest_path, protocol)
    contract = load_contract(contract_path)
    if contract.action_protocol_id != EXPECTED_PROTOCOL:
        raise ValueError(f"collection requires {EXPECTED_PROTOCOL}")
    profile_meta = manifest.payload.get("drive_profile")
    if not isinstance(profile_meta, Mapping):
        raise ValueError("drive profile identity is required")
    profile_path = Path(str(profile_meta.get("path"))).resolve()
    expected_hash = str(profile_meta.get("sha256"))
    actual_hash = _sha256(profile_path)
    if actual_hash != expected_hash:
        raise RuntimeError(
            f"frozen V1 SHA-256 mismatch: expected {expected_hash}, got {actual_hash}"
        )
    controller_meta = manifest.payload.get("controller")
    if not isinstance(controller_meta, Mapping):
        raise ValueError("controller metadata is required")
    expected_controller = {
        "main_acceleration_limit_rad_s2": DEFAULT_MAIN_ACCELERATION_RAD_S2,
        "main_derivative_gain": DEFAULT_MAIN_DERIVATIVE_GAIN,
        "auxiliary_kp": DEFAULT_AUXILIARY_KP,
        "auxiliary_kd": DEFAULT_AUXILIARY_KD,
        "auxiliary_target_rate_limit_rad_s": DEFAULT_AUXILIARY_RATE_RAD_S,
        "auxiliary_velocity_limit_rad_s": 0.30,
    }
    for key, expected in expected_controller.items():
        actual = _finite(controller_meta.get(key), f"controller.{key}")
        if not math.isclose(actual, expected, abs_tol=1e-9, rel_tol=0.0):
            raise ValueError(f"controller.{key} differs from the production value")

    output_dir.mkdir(parents=True, exist_ok=False)
    _atomic_json(
        output_dir / "resolved_transition_manifest.json",
        {
            **manifest.payload,
            "resolved_identity": {
                "manifest_path": str(manifest.source_path),
                "manifest_sha256": _sha256(manifest.source_path),
                "protocol_path": str(protocol_path.resolve()),
                "protocol_sha256": _sha256(protocol_path.resolve()),
                "contract_path": str(contract_path.resolve()),
                "contract_sha256": _sha256(contract_path.resolve()),
                "drive_profile_path": str(profile_path),
                "drive_profile_sha256": actual_hash,
                "robot_usd_path": str(ROBOT_USD.resolve()),
                "robot_usd_sha256": _sha256(ROBOT_USD.resolve()),
            },
        },
    )

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": not gui, "width": 1280, "height": 720})
    try:
        from omni.isaac.core import World
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.utils.types import ArticulationAction

        physics_hz = int(contract.schedule.physics_hz)
        dt = 1.0 / physics_hz
        minimum = np.asarray(contract.action_minimum, dtype=np.float64)
        maximum = np.asarray(contract.action_maximum, dtype=np.float64)
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=dt,
            rendering_dt=1.0 / contract.schedule.stereo_hz,
        )
        world.scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_ROOT)
        robot = world.scene.add(
            Articulation(
                prim_path=ROBOT_ROOT,
                name="round5_transition_ball_robot",
                position=ROBOT_START,
            )
        )
        world.reset()
        if robot.num_dof < 2:
            raise RuntimeError("Round 5 transition collection needs two joints")
        requested_kps, requested_kds = default_articulation_gains()
        zero_action = ArticulationAction(
            joint_velocities=np.asarray([0.0, np.nan, np.nan], dtype=np.float32),
            joint_positions=np.asarray([np.nan, 0.0, np.nan], dtype=np.float32),
        )
        summaries: list[dict[str, object]] = []
        for sequence_index, sequence in enumerate(manifest.sequences, start=1):
            print(f"[{sequence_index:02d}/{len(manifest.sequences):02d}] {sequence.sequence_id}", flush=True)
            world.reset()
            controller = robot.get_articulation_controller()
            controller.set_gains(kps=requested_kps, kds=requested_kds)
            robot.set_world_pose(
                position=ROBOT_START.copy(),
                orientation=np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            )
            robot.set_linear_velocity(np.zeros(3, dtype=np.float32))
            robot.set_angular_velocity(np.zeros(3, dtype=np.float32))
            robot.set_joint_positions(np.zeros(robot.num_dof, dtype=np.float32))
            robot.set_joint_velocities(np.zeros(robot.num_dof, dtype=np.float32))
            for _ in range(int(manifest.payload["procedure"]["settle_steps_before_each_sequence"])):  # type: ignore[index]
                controller.apply_action(zero_action)
                world.step(render=gui)

            smoother = ActionSmoother(
                dt_s=dt,
                max_main_acceleration_rad_s2=DEFAULT_MAIN_ACCELERATION_RAD_S2,
                max_auxiliary_rate_rad_s=DEFAULT_AUXILIARY_RATE_RAD_S,
                max_auxiliary_velocity_rad_s=0.30,
            )
            smoother.reset()
            schedule = build_sequence_schedule(sequence, physics_hz=physics_hz)
            samples: list[dict[str, object]] = []
            for step_index, row in enumerate(schedule):
                measured_positions = np.asarray(robot.get_joint_positions(), dtype=np.float64)
                measured_velocities = np.asarray(robot.get_joint_velocities(), dtype=np.float64)
                if measured_positions.size < 2 or measured_velocities.size < 2:
                    raise RuntimeError("incomplete pre-step joint state")
                requested = np.asarray(
                    [row["main_request_rad_s"], row["auxiliary_request_rad"]],
                    dtype=np.float64,
                )
                clipped = np.clip(requested, minimum, maximum)
                smoother_before = smoother.current_action
                expected_smoothed = np.asarray(
                    [
                        slew_limited_target(
                            smoother_before[0], clipped[0], DEFAULT_MAIN_ACCELERATION_RAD_S2, dt
                        ),
                        slew_limited_target(
                            smoother_before[1], clipped[1], DEFAULT_AUXILIARY_RATE_RAD_S, dt
                        ),
                    ]
                )
                supervisor_active = abs(float(measured_velocities[1])) > 0.30
                applied = smoother.step(
                    requested,
                    minimum,
                    maximum,
                    auxiliary_position_rad=float(measured_positions[1]),
                    auxiliary_velocity_rad_s=float(measured_velocities[1]),
                )
                controller.apply_action(
                    ArticulationAction(
                        joint_velocities=np.asarray([applied[0], np.nan, np.nan], dtype=np.float32),
                        joint_positions=np.asarray([np.nan, applied[1], np.nan], dtype=np.float32),
                    )
                )
                world.step(render=gui)
                position, quaternion = robot.get_world_pose()
                joint_position = np.asarray(robot.get_joint_positions(), dtype=np.float64)
                joint_velocity = np.asarray(robot.get_joint_velocities(), dtype=np.float64)
                linear_velocity = np.asarray(robot.get_linear_velocity(), dtype=np.float64)
                angular_velocity = np.asarray(robot.get_angular_velocity(), dtype=np.float64)
                values = np.concatenate(
                    (
                        np.asarray(position, dtype=np.float64),
                        np.asarray(quaternion, dtype=np.float64),
                        joint_position[:2],
                        joint_velocity[:2],
                        linear_velocity,
                        angular_velocity,
                    )
                )
                if not np.isfinite(values).all():
                    raise RuntimeError(f"non-finite state in {sequence.sequence_id}")
                samples.append(
                    {
                        "time_s": (step_index + 1) * dt,
                        **row,
                        "smoother_action_before_step": smoother_before.tolist(),
                        "smoothed_main_rad_s": float(expected_smoothed[0]),
                        "smoothed_auxiliary_rad": float(expected_smoothed[1]),
                        "final_main_rad_s": float(applied[0]),
                        "final_auxiliary_rad": float(applied[1]),
                        "main_joint_velocity_rad_s": float(joint_velocity[0]),
                        "auxiliary_joint_rad": float(joint_position[1]),
                        "auxiliary_joint_velocity_rad_s": float(joint_velocity[1]),
                        "auxiliary_tracking_error_rad": float(applied[1] - joint_position[1]),
                        "auxiliary_velocity_supervisor_active": bool(supervisor_active),
                        "navigation_heading_rad": _yaw_from_wxyz(quaternion),
                        "root_quaternion_world_wxyz": np.asarray(quaternion, dtype=float).tolist(),
                        "position_xy_m": np.asarray(position, dtype=float)[:2].tolist(),
                        "linear_velocity_world_xy_m_s": linear_velocity[:2].tolist(),
                        "linear_velocity_world_m_s": linear_velocity.tolist(),
                        "angular_velocity_world_rad_s": angular_velocity.tolist(),
                        "physical_contact": None,
                        "formal_collision": None,
                        "end_flag": step_index == len(schedule) - 1,
                    }
                )
            trace = {
                "schema_version": "round5_transition_trace_v1",
                "sequence": {
                    **asdict(sequence),
                    "stages": [asdict(stage) for stage in sequence.stages],
                },
                "contact_semantics": {
                    "physical_contact": None,
                    "formal_collision": None,
                    "reason": "open-plane probe has no obstacle contact filter or occupancy",
                },
                "time_alignment": protocol["time_alignment"],
                "samples": samples,
            }
            trace_path = output_dir / "traces" / f"{sequence.sequence_id}.json"
            _atomic_json(trace_path, trace)
            summaries.append(
                {
                    "sequence_id": sequence.sequence_id,
                    "dataset_role": sequence.dataset_role,
                    "repeat_of": sequence.repeat_of,
                    "sample_count": len(samples),
                    "duration_s": len(samples) * dt,
                    "trace_path": str(trace_path.resolve()),
                    "trace_sha256": _sha256(trace_path),
                    "completed": True,
                }
            )
        report = {
            "schema_version": "round5_transition_collection_v1",
            "measurement_timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "physics_hz": physics_hz,
            "sequence_count": len(summaries),
            "controller": expected_controller,
            "formal_runner_controller_match": {
                "default_articulation_gains_source": "action_smoothing.default_articulation_gains",
                "matches_round4_probe_requested_values": True,
                **_controller_metadata(controller),
            },
            "sequences": summaries,
        }
        _atomic_json(output_dir / "transition_collection.json", report)
        return report
    finally:
        simulation_app.close()


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    run_collection(
        contract_path=args.contract.resolve(),
        protocol_path=args.protocol.resolve(),
        manifest_path=args.manifest.resolve(),
        output_dir=args.out.resolve(),
        gui=bool(args.gui),
    )
    print(args.out.resolve() / "transition_collection.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
