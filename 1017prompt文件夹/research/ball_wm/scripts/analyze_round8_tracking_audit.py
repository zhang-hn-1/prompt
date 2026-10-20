#!/usr/bin/env python3
"""Analyze Round 8 instruction telemetry and independent local A/B geometry."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np


def _atomic_json(path: Path, payload: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    temporary.replace(path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _direction_segment(direction: np.ndarray, start_index: int) -> tuple[int, int]:
    value = int(direction[start_index])
    lower = start_index
    while lower > 0 and int(direction[lower - 1]) == value:
        lower -= 1
    upper = start_index
    while upper + 1 < direction.shape[0] and int(direction[upper + 1]) == value:
        upper += 1
    return lower, upper


def project_trace_on_segment(
    positions_xy: object,
    reference_xy: object,
    *,
    segment_start: int,
    segment_end: int,
    initial_segment_index: int | None = None,
) -> dict[str, np.ndarray]:
    """Project onto one frozen direction segment with topology-local continuity.

    The search state is owned by this evaluator.  It never accepts a policy
    path index.  After the first sample, one segment of backward motion and
    five segments of forward motion are considered, preserving measured
    regress while preventing jumps to a nearby nonlocal portion of a path.
    """

    positions = np.asarray(positions_xy, dtype=np.float64)
    reference = np.asarray(reference_xy, dtype=np.float64)
    if positions.ndim != 2 or positions.shape[1] != 2 or not np.isfinite(positions).all():
        raise ValueError("positions_xy must be finite with shape (N, 2)")
    if reference.ndim != 2 or reference.shape[1] != 2 or not np.isfinite(reference).all():
        raise ValueError("reference_xy must be finite with shape (M, 2)")
    if not 0 <= segment_start < segment_end < reference.shape[0]:
        raise ValueError("segment bounds must contain at least one reference edge")
    edges = reference[segment_start : segment_end + 1]
    deltas = np.diff(edges, axis=0)
    lengths = np.linalg.norm(deltas, axis=1)
    if np.any(lengths <= 1e-12):
        raise ValueError("evaluation segment must not contain zero-length edges")
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    local_index = None if initial_segment_index is None else int(initial_segment_index)
    progress: list[float] = []
    signed_error: list[float] = []
    absolute_error: list[float] = []
    edge_index: list[int] = []
    projected: list[np.ndarray] = []
    for position in positions:
        if local_index is None:
            candidates = range(lengths.shape[0])
        else:
            candidates = range(
                max(0, local_index - 1),
                min(lengths.shape[0], local_index + 6),
            )
        best: tuple[float, int, float, np.ndarray] | None = None
        for index in candidates:
            delta = deltas[index]
            fraction = float(
                np.clip(np.dot(position - edges[index], delta) / (lengths[index] ** 2), 0.0, 1.0)
            )
            point = edges[index] + fraction * delta
            distance = float(np.linalg.norm(position - point))
            candidate = (distance, index, fraction, point)
            if best is None or candidate[0] < best[0] - 1e-12:
                best = candidate
        assert best is not None
        distance, local_index, fraction, point = best
        tangent = deltas[local_index] / lengths[local_index]
        offset = position - point
        signed = float(tangent[0] * offset[1] - tangent[1] * offset[0])
        progress.append(float(cumulative[local_index] + fraction * lengths[local_index]))
        signed_error.append(signed)
        absolute_error.append(distance)
        edge_index.append(segment_start + local_index)
        projected.append(point)
    return {
        "progress_s_m": np.asarray(progress, dtype=np.float64),
        "signed_cross_track_m": np.asarray(signed_error, dtype=np.float64),
        "absolute_cross_track_m": np.asarray(absolute_error, dtype=np.float64),
        "reference_edge_index": np.asarray(edge_index, dtype=np.int64),
        "projected_xy": np.asarray(projected, dtype=np.float64),
    }


def _load_episode(path: Path) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], list[dict[str, object]]]:
    with np.load(path / "states.npz", allow_pickle=False) as archive:
        states = {name: np.asarray(archive[name]) for name in archive.files}
    with np.load(path / "physics_diagnostics.npz", allow_pickle=False) as archive:
        physics = {name: np.asarray(archive[name]) for name in archive.files}
    audit = json.loads((path / "tracking_audit.json").read_text(encoding="utf-8"))
    records = audit["records"]
    if len(records) != states["action_block"].shape[0]:
        raise ValueError("tracking audit is not action aligned")
    return states, physics, records


def _branch_event(states: dict[str, np.ndarray], records: list[dict[str, object]]) -> int:
    active = states["policy_active_drive_direction"]
    switches = np.flatnonzero(active[1:] != active[:-1]) + 1
    if switches.size == 0:
        raise ValueError("reference has no completed direction handoff")
    switch = int(switches[0])
    handoff_index = int(states["policy_reference_index"][switch])
    for index in range(switch + 1, len(records)):
        if (
            int(states["policy_reference_index"][index]) > handoff_index
            and int(states["policy_pending_drive_direction"][index]) == 0
            and records[index].get("source") == "tracking"
        ):
            return index
    raise ValueError("no post-handoff normal-tracking progress event")


def _normal_tracking_end(records: list[dict[str, object]], start: int) -> int:
    for index in range(start, len(records)):
        if records[index].get("source") != "tracking":
            return index
    return len(records)


def _count_supervisor(physics: dict[str, np.ndarray], start_s: float, end_s: float) -> tuple[int, int]:
    time = physics["simulation_time_s"] - physics["simulation_time_s"][0]
    mask = (time >= start_s - 1e-9) & (time < end_s - 1e-9)
    active = np.abs(physics["smoothed_target"][:, 1] - physics["applied_action"][:, 1]) > 1e-7
    return int(np.count_nonzero(active & mask)), int(np.count_nonzero(mask))


def _baseline_instruction_summary(
    states: dict[str, np.ndarray],
    physics: dict[str, np.ndarray],
    records: list[dict[str, object]],
) -> dict[str, object]:
    start = _branch_event(states, records)
    end = _normal_tracking_end(records, start)
    selected = records[start:end]
    denominator = len(selected)
    clipped = np.asarray([bool(row["protocol_clipped"]) for row in selected])
    outside = np.asarray(
        [bool(row["requested_curvature_outside_v1_support"]) for row in selected]
    )
    inverse_at_bound = np.asarray(
        [
            math.isclose(abs(float(row["v1_inverse_auxiliary_rad"])), math.pi / 6.0, abs_tol=1e-7)
            for row in selected
        ]
    )
    protocol_at_bound = np.asarray(
        [
            math.isclose(
                abs(float(row["policy_protocol_auxiliary_target_rad"])),
                math.pi / 6.0,
                abs_tol=1e-7,
            )
            for row in selected
        ]
    )
    cross_track = states["cross_track_error_m"][start:end]
    small_error = cross_track < 0.25
    supervisor_count, supervisor_denominator = _count_supervisor(
        physics, start / 10.0, end / 10.0
    )
    actual = np.asarray([float(row["actual_auxiliary_angle_rad"]) for row in selected])
    raw = np.asarray([float(row["policy_raw_auxiliary_target_rad"]) for row in selected])
    protocol = np.asarray(
        [float(row["policy_protocol_auxiliary_target_rad"]) for row in selected]
    )
    previous_applied = np.asarray(
        [float(row["previous_applied_auxiliary_rad"]) for row in selected]
    )
    return {
        "normal_tracking_start_action": start,
        "normal_tracking_end_action_exclusive": end,
        "normal_tracking_time_s": [start / 10.0, end / 10.0],
        "policy_block_denominator": denominator,
        "protocol_clipped_count": int(np.count_nonzero(clipped)),
        "protocol_clipped_fraction": float(np.mean(clipped)),
        "inverse_at_v1_boundary_count": int(np.count_nonzero(inverse_at_bound)),
        "requested_curvature_outside_v1_support_count": int(np.count_nonzero(outside)),
        "protocol_angle_at_bound_count": int(np.count_nonzero(protocol_at_bound)),
        "feedback_only_clip_count": int(np.count_nonzero(clipped & ~inverse_at_bound)),
        "small_cross_track_lt_0p25_count": int(np.count_nonzero(small_error)),
        "small_cross_track_protocol_clipped_count": int(np.count_nonzero(small_error & clipped)),
        "physics_tick_denominator": supervisor_denominator,
        "velocity_supervisor_active_count": supervisor_count,
        "velocity_supervisor_active_fraction": supervisor_count / supervisor_denominator,
        "actual_aux_error_vs_raw_mean_rad": float(np.mean(np.abs(actual - raw))),
        "actual_aux_error_vs_protocol_mean_rad": float(np.mean(np.abs(actual - protocol))),
        "actual_aux_error_vs_previous_applied_mean_rad": float(
            np.mean(np.abs(actual - previous_applied))
        ),
    }


def _window_metrics(
    states: dict[str, np.ndarray],
    records: list[dict[str, object]],
    *,
    start: int,
    duration_actions: int = 80,
) -> tuple[dict[str, object], dict[str, np.ndarray]]:
    end_action = min(start + duration_actions, len(records))
    directions = states["kinodynamic_reference_drive_direction"]
    reference = states["kinodynamic_reference_pose"][:, :2]
    post_direction = int(states["policy_active_drive_direction"][start])
    matching = np.flatnonzero(directions == post_direction)
    segment_start, segment_end = _direction_segment(directions, int(matching[0]))
    projection = project_trace_on_segment(
        states["position_world_xy"][start : end_action + 1],
        reference,
        segment_start=segment_start,
        segment_end=segment_end,
    )
    error = projection["absolute_cross_track_m"]
    progress = projection["progress_s_m"]
    action_records = records[start:end_action]
    checkpoints = []
    for offset in (0.5, 1.0):
        threshold = float(progress[0] + offset)
        reached = np.flatnonzero(progress >= threshold)
        checkpoints.append(
            {
                "offset_m": offset,
                "reached": bool(reached.size),
                "time_s": (float(reached[0]) / 10.0 if reached.size else None),
                "cross_track_m": (float(error[reached[0]]) if reached.size else None),
            }
        )
    protocol_aux = np.asarray(
        [float(row["policy_protocol_auxiliary_target_rad"]) for row in action_records]
    )
    candidate_active = np.asarray(
        [bool(row.get("candidate_active", False)) for row in action_records]
    )
    return (
        {
            "start_action": start,
            "end_action_exclusive": end_action,
            "duration_s": (end_action - start) / 10.0,
            "state_count": int(error.shape[0]),
            "cross_track_rms_m": float(np.sqrt(np.mean(np.square(error)))),
            "cross_track_peak_m": float(np.max(error)),
            "progress_delta_m": float(progress[-1] - progress[0]),
            "progress_regression_m": float(np.sum(np.maximum(0.0, -np.diff(progress)))),
            "segment_progress_start_m": float(progress[0]),
            "segment_progress_end_m": float(progress[-1]),
            "segment_length_m": float(
                np.linalg.norm(np.diff(reference[segment_start : segment_end + 1], axis=0), axis=1).sum()
            ),
            "segment_completed": bool(progress[-1] >= np.linalg.norm(np.diff(reference[segment_start : segment_end + 1], axis=0), axis=1).sum() - 0.10),
            "protocol_auxiliary_at_bound_fraction": float(
                np.mean(np.isclose(np.abs(protocol_aux), math.pi / 6.0, atol=1e-7))
            ),
            "candidate_active_fraction": float(np.mean(candidate_active)),
            "spatial_checkpoints": checkpoints,
        },
        projection,
    )


def _compare_case(baseline_path: Path, candidate_path: Path) -> tuple[dict[str, object], dict[str, np.ndarray]]:
    base_states, base_physics, base_records = _load_episode(baseline_path)
    cand_states, cand_physics, cand_records = _load_episode(candidate_path)
    base_start = _branch_event(base_states, base_records)
    cand_start = _branch_event(cand_states, cand_records)
    prefix_end = min(base_start, cand_start)
    prefix_fields = (
        "position_world_xy",
        "yaw_world_rad",
        "linear_velocity_world_m_s",
        "angular_velocity_world_rad_s",
        "joint_position_rad",
        "joint_velocity_rad_s",
        "action_block",
        "applied_action",
        "policy_reference_index",
        "policy_active_drive_direction",
        "policy_pending_drive_direction",
    )
    prefix_equal = {}
    for name in prefix_fields:
        limit = prefix_end if name in {"action_block", "policy_reference_index", "policy_active_drive_direction", "policy_pending_drive_direction"} else prefix_end + 1
        prefix_equal[name] = bool(
            base_states[name][:limit].shape == cand_states[name][:limit].shape
            and np.array_equal(base_states[name][:limit], cand_states[name][:limit])
        )
    base_metrics, base_projection = _window_metrics(base_states, base_records, start=base_start)
    cand_metrics, cand_projection = _window_metrics(cand_states, cand_records, start=cand_start)
    comparable = min(80, len(base_records) - base_start, len(cand_records) - cand_start)
    base_actions = base_states["action_block"][base_start : base_start + comparable]
    cand_actions = cand_states["action_block"][cand_start : cand_start + comparable]
    base_applied = base_physics["applied_action"][base_start * 5 : (base_start + comparable) * 5]
    cand_applied = cand_physics["applied_action"][cand_start * 5 : (cand_start + comparable) * 5]
    rms_reduction = 1.0 - cand_metrics["cross_track_rms_m"] / base_metrics["cross_track_rms_m"]
    gates = {
        "prefix_all_equal": bool(all(prefix_equal.values()) and base_start == cand_start),
        "rms_reduction_at_least_20_percent": bool(rms_reduction >= 0.20),
        "progress_not_lower_by_more_than_0p05_m": bool(
            cand_metrics["progress_delta_m"] >= base_metrics["progress_delta_m"] - 0.05
        ),
        "peak_not_worse_by_more_than_0p05_m": bool(
            cand_metrics["cross_track_peak_m"] <= base_metrics["cross_track_peak_m"] + 0.05
        ),
        "protocol_action_changed": bool(not np.array_equal(base_actions, cand_actions)),
        "applied_action_changed": bool(not np.array_equal(base_applied, cand_applied)),
    }
    return (
        {
            "branch_action_baseline": base_start,
            "branch_action_candidate": cand_start,
            "prefix_equal_by_field": prefix_equal,
            "baseline": base_metrics,
            "candidate": cand_metrics,
            "rms_reduction_fraction": float(rms_reduction),
            "protocol_action_changed_count": int(np.count_nonzero(base_actions != cand_actions)),
            "applied_action_changed_count": int(np.count_nonzero(base_applied != cand_applied)),
            "gates": gates,
            "local_gate_passed": bool(all(gates.values())),
        },
        {
            "base_xy": base_states["position_world_xy"][base_start : base_start + 81],
            "candidate_xy": cand_states["position_world_xy"][cand_start : cand_start + 81],
            "reference_xy": base_states["kinodynamic_reference_pose"][:, :2],
            "base_error": base_projection["absolute_cross_track_m"],
            "candidate_error": cand_projection["absolute_cross_track_m"],
        },
    )


def analyze(baseline_root: Path, candidate_root: Path | None, output_dir: Path) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=False)
    cases = sorted(path.name for path in baseline_root.iterdir() if path.is_dir())
    baseline = {}
    comparisons = {}
    plot_data = {}
    for case in cases:
        states, physics, records = _load_episode(baseline_root / case)
        baseline[case] = _baseline_instruction_summary(states, physics, records)
        if candidate_root is not None:
            comparisons[case], plot_data[case] = _compare_case(
                baseline_root / case, candidate_root / case
            )
    payload = {
        "schema_version": "round8_tracking_audit_analysis_v1",
        "baseline_root": str(baseline_root.resolve()),
        "candidate_root": str(candidate_root.resolve()) if candidate_root else None,
        "evaluator": {
            "source_sha256": _sha256(Path(__file__).resolve()),
            "window_s": 8.0,
            "progress_tolerance_m": 0.05,
            "rms_reduction_required": 0.20,
            "peak_degradation_tolerance_m": 0.05,
            "policy_path_index_used_for_scoring": False,
        },
        "baseline_instruction_chain": baseline,
        "local_ab": comparisons,
        "all_local_gates_passed": bool(comparisons) and all(
            row["local_gate_passed"] for row in comparisons.values()
        ),
    }
    _atomic_json(output_dir / "round8_tracking_audit.json", payload)
    with (output_dir / "round8_tracking_audit.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("case", "side", "rms_m", "peak_m", "progress_delta_m", "clip_fraction"))
        for case, row in comparisons.items():
            for side in ("baseline", "candidate"):
                metrics = row[side]
                writer.writerow((case, side, metrics["cross_track_rms_m"], metrics["cross_track_peak_m"], metrics["progress_delta_m"], metrics["protocol_auxiliary_at_bound_fraction"]))
    if plot_data:
        figure, axes = plt.subplots(len(plot_data), 2, figsize=(12, 5 * len(plot_data)), squeeze=False)
        for row_index, (case, values) in enumerate(plot_data.items()):
            axes[row_index, 0].plot(values["reference_xy"][:, 0], values["reference_xy"][:, 1], "k--", label="frozen reference")
            axes[row_index, 0].plot(values["base_xy"][:, 0], values["base_xy"][:, 1], label="B0")
            axes[row_index, 0].plot(values["candidate_xy"][:, 0], values["candidate_xy"][:, 1], label="candidate")
            axes[row_index, 0].axis("equal")
            axes[row_index, 0].set_title(case)
            axes[row_index, 0].legend()
            axes[row_index, 1].plot(np.arange(values["base_error"].shape[0]) / 10.0, values["base_error"], label="B0")
            axes[row_index, 1].plot(np.arange(values["candidate_error"].shape[0]) / 10.0, values["candidate_error"], label="candidate")
            axes[row_index, 1].set_title("independent cross-track, first 8 s")
            axes[row_index, 1].set_xlabel("seconds after branch")
            axes[row_index, 1].set_ylabel("m")
            axes[row_index, 1].legend()
        figure.tight_layout()
        figure.savefig(output_dir / "round8_local_ab.png", dpi=180)
        plt.close(figure)
    return payload


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--candidate-root", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    result = analyze(
        args.baseline_root.resolve(),
        args.candidate_root.resolve() if args.candidate_root else None,
        args.out.resolve(),
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
