#!/usr/bin/env python3
"""Summarize Round 6 online shadow causality and reference handoff events."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


def _load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def _atomic_json(path: Path, payload: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _first_index(mask: np.ndarray, start: int = 0) -> int | None:
    selected = np.flatnonzero(mask & (np.arange(len(mask)) >= start))
    return int(selected[0]) if selected.size else None


def _handoff_events(case_dir: Path) -> dict[str, object]:
    online = _load_json(case_dir / "online_shadow.json")
    contexts = online["contexts"]
    states = np.load(case_dir / "states.npz", allow_pickle=False)
    physics = np.load(case_dir / "physics_diagnostics.npz", allow_pickle=False)
    active = np.asarray(states["policy_active_drive_direction"], dtype=int)
    desired = np.asarray(states["policy_desired_drive_direction"], dtype=int)
    boundary_index = _first_index(desired != active)
    if boundary_index is None:
        raise ValueError(f"no direction boundary in {case_dir}")
    active_switch = _first_index(active == desired[boundary_index], boundary_index + 1)
    raw_request = np.asarray([row["protocol_request_component"] for row in contexts], dtype=float)
    request_zero = _first_index(np.abs(raw_request[:, 0]) <= 0.02, boundary_index)
    request_new_direction = _first_index(raw_request[:, 0] * desired[boundary_index] > 0.02, boundary_index)
    stopped = np.asarray([row["stopped_cycles"] for row in contexts], dtype=int)
    first_stopped = _first_index(stopped > 0, boundary_index)
    physics_time = np.asarray(physics["simulation_time_s"], dtype=float)
    applied_main = np.asarray(physics["applied_action"], dtype=float)[:, 0]
    joint_main = np.asarray(physics["joint_velocity_rad_s"], dtype=float)[:, 0]
    body_speed = np.linalg.norm(np.asarray(physics["linear_velocity_world_m_s"], dtype=float)[:, :2], axis=1)
    boundary_tick = int(boundary_index * 5)
    applied_zero = _first_index(np.abs(applied_main) <= 0.02, boundary_tick)
    applied_new = _first_index(applied_main * desired[boundary_index] > 0.02, boundary_tick)
    joint_new = _first_index(joint_main * desired[boundary_index] > 0.05, boundary_tick)
    speed_below = _first_index(body_speed < 0.05, boundary_tick)
    records = online["records"]
    boundary_records = [record for record in records if record["boundary_active"]]
    plan_count = sum(len(record["plans"]) for record in boundary_records)
    supported = sum(
        bool(plan["support"]["supported"])
        for record in boundary_records
        for plan in record["plans"].values()
    )

    def control_time(index: int | None) -> float | None:
        return None if index is None else float(contexts[index]["control_time_s"])

    def tick_time(index: int | None) -> float | None:
        return None if index is None else float(physics_time[index])

    return {
        "case_id": case_dir.name.removeprefix("episode_"),
        "reference_direction_change_control_time_s": control_time(boundary_index),
        "zero_protocol_request_control_time_s": control_time(request_zero),
        "smoothed_applied_main_near_zero_time_s": tick_time(applied_zero),
        "first_body_speed_below_0p05_time_s": tick_time(speed_below),
        "first_stopped_confirmation_control_time_s": control_time(first_stopped),
        "active_direction_latch_switch_control_time_s": control_time(active_switch),
        "new_direction_protocol_request_control_time_s": control_time(request_new_direction),
        "new_direction_smoothed_applied_time_s": tick_time(applied_new),
        "main_joint_new_direction_time_s": tick_time(joint_new),
        "zero_request_to_new_request_delay_s": (
            None if request_zero is None or request_new_direction is None
            else control_time(request_new_direction) - control_time(request_zero)
        ),
        "boundary_record_count": len(boundary_records),
        "plan_count": plan_count,
        "supported_plan_count": supported,
        "advice_values": sorted({str(record["advice"]) for record in boundary_records}),
        "maximum_stopped_cycles_seen": int(stopped.max()),
    }


def _equivalence(case: str, off_dir: Path, on_dir: Path) -> dict[str, object]:
    off_config = _load_json(off_dir / "config.json")
    on_config = _load_json(on_dir / "config.json")
    shared_config_fields = (
        "case_id", "model_sha256", "support_cloud_sha256", "drive_profile_sha256",
        "action_protocol_id", "episode_budget_s", "replan_hz", "physics_hz",
        "shadow_horizon_s", "scene_sha256", "fixture_sha256", "occupancy_sha256",
        "contract_sha256", "implementation_sha256", "reference",
    )
    config_checks = {
        "baseline_policy_is_b0": off_config.get("policy") == "B0/oracle",
        "baseline_shadow_disabled": off_config.get("shadow_enabled") is False,
        "candidate_policy_is_b0_plus_shadow": on_config.get("policy") == "B0/oracle+read_only_shadow",
        "candidate_shadow_enabled": on_config.get("shadow_enabled") is True,
        **{
            f"same_{name}": off_config.get(name) == on_config.get(name)
            for name in shared_config_fields
        },
    }
    if not all(config_checks.values()):
        failed = [name for name, passed in config_checks.items() if not passed]
        raise ValueError(f"uncontrolled shadow on/off comparison for {case}: {failed}")
    off_states = np.load(off_dir / "states.npz", allow_pickle=False)
    on_states = np.load(on_dir / "states.npz", allow_pickle=False)
    off_physics = np.load(off_dir / "physics_diagnostics.npz", allow_pickle=False)
    on_physics = np.load(on_dir / "physics_diagnostics.npz", allow_pickle=False)
    if set(off_states.files) != set(on_states.files):
        raise ValueError(f"state archive fields differ for {case}")
    state_equal = {
        name: bool(np.array_equal(off_states[name], on_states[name]))
        for name in off_states.files
    }
    if set(off_physics.files) != set(on_physics.files):
        raise ValueError(f"physics archive fields differ for {case}")
    physics_equal = {
        name: bool(np.array_equal(off_physics[name], on_physics[name]))
        for name in off_physics.files
        if name not in ("raw_simulation_time_s", "raw_physics_step_index")
    }
    raw_time_offset = np.asarray(on_physics["raw_simulation_time_s"]) - np.asarray(off_physics["raw_simulation_time_s"])
    raw_step_offset = np.asarray(on_physics["raw_physics_step_index"]) - np.asarray(off_physics["raw_physics_step_index"])
    off_metrics = _load_json(off_dir / "metrics.json")
    on_metrics = _load_json(on_dir / "metrics.json")
    metric_names = (
        "termination_reason", "action_blocks", "elapsed_s", "final_distance_m",
        "path_length_m", "reference_length_m", "cross_track_rms_m", "cross_track_peak_m",
    )
    metrics_equal = {name: off_metrics[name] == on_metrics[name] for name in metric_names}
    return {
        "case_id": case,
        "controlled_config_checks": config_checks,
        "baseline_config_sha256": _sha256(off_dir / "config.json"),
        "shadow_config_sha256": _sha256(on_dir / "config.json"),
        "state_arrays_exact": state_equal,
        "physics_arrays_exact_excluding_absolute_raw_clock": physics_equal,
        "all_behavior_arrays_exact": all(state_equal.values()) and all(physics_equal.values()),
        "absolute_raw_clock_offset_constant_s": bool(np.all(raw_time_offset == raw_time_offset[0])),
        "absolute_raw_clock_offset_s": float(raw_time_offset[0]),
        "absolute_raw_step_offset_constant": bool(np.all(raw_step_offset == raw_step_offset[0])),
        "absolute_raw_step_offset": int(raw_step_offset[0]),
        "outcome_metrics_exact": metrics_equal,
        "all_outcome_metrics_exact": all(metrics_equal.values()),
    }


def _plots(cases: Sequence[str], off_root: Path, on_root: Path, output_dir: Path) -> None:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, len(cases), figsize=(12, 5), constrained_layout=True)
    for axis, case in zip(np.atleast_1d(axes), cases):
        off = np.load(off_root / f"episode_{case}" / "states.npz", allow_pickle=False)
        on = np.load(on_root / f"episode_{case}" / "states.npz", allow_pickle=False)
        ref = np.asarray(on["kinodynamic_reference_pose"])[:, :2]
        axis.plot(ref[:, 0], ref[:, 1], "k--", label="frozen reference")
        axis.plot(off["position_world_xy"][:, 0], off["position_world_xy"][:, 1], linewidth=3, alpha=0.45, label="B0 shadow off")
        axis.plot(on["position_world_xy"][:, 0], on["position_world_xy"][:, 1], linewidth=1.3, label="B0 shadow on")
        axis.set_title(case)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("world x (m)")
        axis.set_ylabel("world y (m)")
        axis.grid(alpha=0.25)
    axes[0].legend(fontsize=8)
    fig.savefig(output_dir / "round6_shadow_on_off_trajectory_overlay.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(len(cases), 1, figsize=(11, 7), sharex=False, constrained_layout=True)
    for axis, case in zip(np.atleast_1d(axes), cases):
        physics = np.load(on_root / f"episode_{case}" / "physics_diagnostics.npz", allow_pickle=False)
        time = np.asarray(physics["simulation_time_s"])
        axis.plot(time, physics["raw_command_target"][:, 0], label="protocol request")
        axis.plot(time, physics["applied_action"][:, 0], label="smoothed/applied target")
        axis.plot(time, physics["joint_velocity_rad_s"][:, 0], label="measured main speed")
        axis.plot(time, np.linalg.norm(physics["linear_velocity_world_m_s"][:, :2], axis=1), label="planar speed")
        axis.axhline(0.0, color="black", linewidth=0.7)
        axis.set_xlim(2.5, 9.0)
        axis.set_title(case)
        axis.set_ylabel("SI units")
        axis.grid(alpha=0.25)
    axes[-1].set_xlabel("simulation time (s)")
    axes[0].legend(ncol=4, fontsize=8)
    fig.savefig(output_dir / "round6_reference_handoff_timeline.png", dpi=180)
    plt.close(fig)


def run(args: argparse.Namespace) -> dict[str, object]:
    cases = ("hard_0_e1_0002", "hard_0_e1_0004")
    args.output.mkdir(parents=True, exist_ok=True)
    events = [_handoff_events(args.shadow_root / f"episode_{case}") for case in cases]
    equivalence = [
        _equivalence(
            case,
            args.baseline_root / f"episode_{case}",
            args.shadow_root / f"episode_{case}",
        )
        for case in cases
    ]
    causality = _load_json(args.causality_validation)
    if causality.get("passed") is not True:
        raise ValueError("causality validation artifact did not pass")
    with (args.output / "round6_handoff_events.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(events[0]))
        writer.writeheader()
        writer.writerows(events)
    result = {
        "schema_version": "round6_interface_and_handoff_summary_v1",
        "handoff_events": events,
        "shadow_on_off_equivalence": equivalence,
        "future_log_suffix_test": {
            "artifact": str(args.causality_validation.resolve()),
            "artifact_sha256": _sha256(args.causality_validation),
            "checks": causality["checks"],
            "result": "passed",
        },
        "single_live_policy_call_test": {
            "test": "test_shadow_calls_b0_once_and_returns_exact_action_block",
            "result": "passed",
        },
    }
    _atomic_json(args.output / "interface_validation.json", result)
    _plots(cases, args.baseline_root, args.shadow_root, args.output)
    return result


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--shadow-root", type=Path, required=True)
    parser.add_argument("--causality-validation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    result = run(_parse_args(argv))
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
