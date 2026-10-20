"""Create Round 5 figures and sequence-level measured-transition summaries."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import analyze_round5_motion_transitions as round5


def _load(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected object: {path}")
    return payload


def _first_target_sign_time(
    samples: Sequence[Mapping[str, object]],
    *,
    start_index: int,
    measured_key: str,
    target: float,
) -> float | None:
    if abs(target) <= 1e-12:
        return None
    origin = float(samples[start_index]["time_s"])
    target_sign = math.copysign(1.0, target)
    for sample in samples[start_index:]:
        measured = float(sample[measured_key])
        if abs(measured) > 1e-6 and math.copysign(1.0, measured) == target_sign:
            return float(sample["time_s"]) - origin
    return None


def _sequence_summary(trace_path: Path) -> dict[str, object]:
    trace = _load(trace_path)
    samples = round5._trace_samples(trace)
    start_index, transition_index = round5._transition_boundary(samples)
    transition = samples[transition_index:]
    target_main = float(transition[0]["requested_main_rad_s"])
    target_aux = float(transition[0]["requested_auxiliary_rad"])
    position = np.asarray([sample["position_xy_m"] for sample in transition], dtype=float)
    main_velocity = np.asarray([sample["main_joint_velocity_rad_s"] for sample in transition], dtype=float)
    aux_velocity = np.asarray([sample["auxiliary_joint_velocity_rad_s"] for sample in transition], dtype=float)
    supervisor = np.asarray(
        [sample["auxiliary_velocity_supervisor_active"] for sample in transition], dtype=bool
    )
    return {
        "sequence_id": trace["sequence"]["sequence_id"],
        "dataset_role": trace["sequence"]["dataset_role"],
        "transition_request_main_rad_s": target_main,
        "transition_request_auxiliary_rad": target_aux,
        "start_main_joint_velocity_rad_s": float(samples[start_index]["main_joint_velocity_rad_s"]),
        "start_auxiliary_joint_rad": float(samples[start_index]["auxiliary_joint_rad"]),
        "main_target_sign_first_observed_s": _first_target_sign_time(
            samples,
            start_index=transition_index,
            measured_key="main_joint_velocity_rad_s",
            target=target_main,
        ),
        "auxiliary_target_sign_first_observed_s": _first_target_sign_time(
            samples,
            start_index=transition_index,
            measured_key="auxiliary_joint_rad",
            target=target_aux,
        ),
        "transition_path_length_m": float(
            np.linalg.norm(np.diff(position, axis=0), axis=1).sum()
        ),
        "main_joint_velocity_abs_peak_rad_s": float(np.max(np.abs(main_velocity))),
        "auxiliary_joint_velocity_abs_peak_rad_s": float(np.max(np.abs(aux_velocity))),
        "auxiliary_supervisor_active_fraction": float(np.mean(supervisor)),
        "trace_path": str(trace_path.resolve()),
        "trace_sha256": round5._sha256(trace_path),
    }


def _transition_trajectory_figure(trace_paths: Sequence[Path], output: Path) -> None:
    roles = ("fit_core", "heldout_validation", "repeat_validation")
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)
    for axis, role in zip(axes, roles):
        for path in trace_paths:
            trace = _load(path)
            if trace["sequence"]["dataset_role"] != role:
                continue
            samples = round5._trace_samples(trace)
            start_index, first = round5._transition_boundary(samples)
            start = samples[start_index]
            start_position = np.asarray(start["position_xy_m"], dtype=float)
            yaw = float(start["navigation_heading_rad"])
            local = np.vstack(
                [
                    round5._rotate_world_to_local(
                        np.asarray(sample["position_xy_m"], dtype=float) - start_position, yaw
                    )
                    for sample in samples[first:]
                ]
            )
            axis.plot(local[:, 0], local[:, 1], linewidth=1.4, label=trace["sequence"]["sequence_id"])
        axis.scatter([0.0], [0.0], marker="x", color="black", zorder=5)
        axis.set_title(role.replace("_", " "))
        axis.set_xlabel("local forward Δx (m)")
        axis.set_ylabel("local left Δy (m)")
        axis.axis("equal")
        axis.grid(alpha=0.25)
        axis.legend(fontsize=6, loc="best")
    fig.suptitle("Round 5 measured 6 s transition trajectories")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def _prediction_error_figure(evaluation: Mapping[str, object], output: Path) -> None:
    rows = [row for row in evaluation["rows"] if row["dataset_role"] == "heldout_validation"]
    horizons = sorted({float(row["horizon_s"]) for row in rows})
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), constrained_layout=True)
    for prefix, label, color in (("m0", "M0 V1", "#777777"), ("m1", "M1 stateful", "#1976d2")):
        endpoint = [
            np.mean([row[f"{prefix}_endpoint_position_error_m"] for row in rows if math.isclose(float(row["horizon_s"]), horizon)])
            for horizon in horizons
        ]
        heading = [
            np.mean([abs(row[f"{prefix}_root_heading_change_error_rad"]) for row in rows if math.isclose(float(row["horizon_s"]), horizon)])
            for horizon in horizons
        ]
        axes[0].plot(horizons, endpoint, "o-", color=color, label=label)
        axes[1].plot(horizons, heading, "o-", color=color, label=label)
    axes[0].set_ylabel("held-out endpoint MAE (m)")
    axes[1].set_ylabel("held-out heading-change MAE (rad)")
    for axis in axes:
        axis.set_xlabel("prediction horizon (s)")
        axis.grid(alpha=0.25)
        axis.legend()
    fig.suptitle("Independent whole-sequence prediction error")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def _shadow_error_figure(shadow: Mapping[str, object], output: Path) -> None:
    rows = shadow["rows"]
    episodes = sorted({str(row["episode_id"]) for row in rows})
    fig, axes = plt.subplots(1, len(episodes), figsize=(6 * len(episodes), 4.2), constrained_layout=True)
    axes = np.atleast_1d(axes)
    for axis, episode in zip(axes, episodes):
        episode_rows = [row for row in rows if row["episode_id"] == episode]
        horizons = sorted({float(row["horizon_s"]) for row in episode_rows})
        for prefix, label, color in (("m0", "M0 V1", "#777777"), ("m1", "M1 stateful", "#d84315")):
            values = [
                np.mean([row[f"{prefix}_endpoint_position_error_m"] for row in episode_rows if math.isclose(float(row["horizon_s"]), horizon)])
                for horizon in horizons
            ]
            axis.plot(horizons, values, "o-", color=color, label=label)
        axis.set_title(episode)
        axis.set_xlabel("prediction horizon (s)")
        axis.set_ylabel("action-conditioned endpoint MAE (m)")
        axis.grid(alpha=0.25)
        axis.legend()
    fig.suptitle("Frozen-reference action-conditioned shadow replay")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def _response_figure(trace_root: Path, output: Path) -> None:
    sequence_ids = (
        "core_main_reverse_to_forward_a0",
        "core_forward_aux_an30_to_ap30",
        "core_joint_rp_an30_to_fp_ap30",
    )
    fig, axes = plt.subplots(len(sequence_ids), 2, figsize=(12, 9), constrained_layout=True)
    for row_axes, sequence_id in zip(axes, sequence_ids):
        trace = _load(trace_root / "traces" / f"{sequence_id}.json")
        samples = round5._trace_samples(trace)
        _, first = round5._transition_boundary(samples)
        origin = float(samples[first]["time_s"])
        time = np.asarray([float(sample["time_s"]) - origin for sample in samples])
        row_axes[0].plot(time, [sample["requested_main_rad_s"] for sample in samples], "--", label="request")
        row_axes[0].plot(time, [sample["final_main_rad_s"] for sample in samples], label="final command")
        row_axes[0].plot(time, [sample["main_joint_velocity_rad_s"] for sample in samples], label="measured")
        row_axes[1].plot(time, [sample["requested_auxiliary_rad"] for sample in samples], "--", label="request")
        row_axes[1].plot(time, [sample["final_auxiliary_rad"] for sample in samples], label="final command")
        row_axes[1].plot(time, [sample["auxiliary_joint_rad"] for sample in samples], label="measured")
        for axis in row_axes:
            axis.axvline(0.0, color="black", linewidth=0.8)
            axis.grid(alpha=0.25)
            axis.set_xlabel("time from request switch (s)")
            axis.legend(fontsize=7)
        row_axes[0].set_ylabel(f"{sequence_id}\nmain (rad/s)")
        row_axes[1].set_ylabel("auxiliary (rad)")
    fig.suptitle("Measured state inheritance across command transitions")
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transition-root", type=Path, required=True)
    parser.add_argument("--evaluation", type=Path, required=True)
    parser.add_argument("--shadow", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    trace_paths = sorted((args.transition_root / "traces").glob("*.json"))
    summaries = [_sequence_summary(path) for path in trace_paths]
    round5._atomic_json(
        args.out / "round5_sequence_summary.json",
        {"schema_version": "round5_sequence_summary_v1", "sequences": summaries},
    )
    round5._write_csv(args.out / "round5_sequence_summary.csv", summaries)
    _transition_trajectory_figure(trace_paths, args.out / "round5_transition_trajectories.png")
    _prediction_error_figure(_load(args.evaluation), args.out / "round5_heldout_prediction_errors.png")
    _shadow_error_figure(_load(args.shadow), args.out / "round5_fixed_reference_shadow_errors.png")
    _response_figure(args.transition_root, args.out / "round5_transition_responses.png")
    print(args.out.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
