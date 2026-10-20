#!/usr/bin/env python3
"""Segment the Round 6 fixed-reference B0 runs using measured state and B0 state transitions."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import matplotlib.pyplot as plt
import numpy as np


def _wrap(value: float) -> float:
    return float((value + math.pi) % (2.0 * math.pi) - math.pi)


def _project_to_active_direction_run(
    point: np.ndarray,
    reference_pose: np.ndarray,
    directions: np.ndarray,
    reference_index: int,
    active_direction: int,
) -> tuple[float, float, float]:
    """Return global arc progress, cross-track, and interpolated reference yaw.

    Projection is limited to the contiguous reference-direction run containing
    the current B0 reference index.  This prevents a nearby path branch with a
    different direction or later handoff from inflating progress.
    """

    count = len(reference_pose)
    anchor = int(np.clip(reference_index, 0, count - 1))
    if int(directions[anchor]) != int(active_direction):
        matches = np.flatnonzero(directions == active_direction)
        anchor = int(matches[np.argmin(np.abs(matches - anchor))])
    begin = anchor
    while begin > 0 and int(directions[begin - 1]) == int(active_direction):
        begin -= 1
    end = anchor
    while end + 1 < count and int(directions[end + 1]) == int(active_direction):
        end += 1
    if end == begin:
        return (0.0, float(np.linalg.norm(point - reference_pose[begin, :2])), float(reference_pose[begin, 2]))

    segment = np.diff(reference_pose[:, :2], axis=0)
    lengths = np.linalg.norm(segment, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    best: tuple[float, float, float] | None = None
    for index in range(begin, end):
        delta = reference_pose[index + 1, :2] - reference_pose[index, :2]
        squared = float(delta @ delta)
        fraction = 0.0 if squared <= 1e-12 else float(np.clip((point - reference_pose[index, :2]) @ delta / squared, 0.0, 1.0))
        projection = reference_pose[index, :2] + fraction * delta
        distance = float(np.linalg.norm(point - projection))
        yaw = float(reference_pose[index, 2] + fraction * _wrap(float(reference_pose[index + 1, 2] - reference_pose[index, 2])))
        candidate = (float(cumulative[index] + fraction * lengths[index]), distance, _wrap(yaw))
        if best is None or candidate[1] < best[1]:
            best = candidate
    assert best is not None
    return best


def _phase_ranges(contexts: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    brake = next(i for i, item in enumerate(contexts) if item["braking_state"] == "direction_change")
    low = next(i for i in range(brake, len(contexts)) if int(contexts[i]["stopped_cycles"]) > 0)
    switch = next(i for i in range(brake + 1, len(contexts)) if contexts[i]["active_reference_direction"] != contexts[brake]["active_reference_direction"])
    switch_index = int(contexts[switch]["reference_index"])
    realigned = next(
        i for i in range(switch + 1, len(contexts))
        if int(contexts[i]["reference_index"]) > switch_index
    )
    infeasible = next(
        (i for i in range(realigned, len(contexts)) if contexts[i]["braking_state"] == "tracking_infeasible"),
        len(contexts),
    )
    return [
        {"phase": "pre_braking_same_direction", "start": 0, "end": brake},
        {"phase": "braking_before_low_speed", "start": brake, "end": low},
        {"phase": "low_speed_confirmation", "start": low, "end": switch},
        {"phase": "post_switch_realignment", "start": switch, "end": realigned},
        {"phase": "normal_same_direction_tracking", "start": realigned, "end": infeasible},
        {"phase": "terminal_sustained_tracking_loss", "start": infeasible, "end": len(contexts)},
    ]


def _sustained_loss_index(cross_track: np.ndarray, *, threshold_m: float = 0.5, samples: int = 10) -> int | None:
    high = cross_track > threshold_m
    for start in range(0, len(high) - samples + 1):
        if bool(np.all(high[start : start + samples])):
            return start
    return None


def _sustained_growth_index(
    cross_track: np.ndarray,
    *,
    net_growth_m: float = 0.10,
    samples: int = 10,
    minimum_non_decreasing_steps: int = 8,
) -> int | None:
    for start in range(0, len(cross_track) - samples + 1):
        window = cross_track[start : start + samples]
        if (
            window[-1] - window[0] >= net_growth_m
            and int(np.count_nonzero(np.diff(window) >= -1e-6))
            >= minimum_non_decreasing_steps
        ):
            return start
    return None


def _analyse_episode(episode: Path) -> tuple[dict[str, object], dict[str, np.ndarray]]:
    shadow = json.loads((episode / "online_shadow.json").read_text(encoding="utf-8"))
    contexts = shadow["contexts"]
    metrics = json.loads((episode / "metrics.json").read_text(encoding="utf-8"))
    with np.load(episode / "states.npz", allow_pickle=False) as archive:
        position = np.asarray(archive["position_world_xy"][:-1], dtype=float)
        yaw = np.asarray(archive["yaw_world_rad"][:-1], dtype=float)
        reference = np.asarray(archive["kinodynamic_reference_pose"], dtype=float)
        directions = np.asarray(archive["kinodynamic_reference_drive_direction"], dtype=int)
    with np.load(episode / "physics_diagnostics.npz", allow_pickle=False) as archive:
        smoothed = np.asarray(archive["smoothed_target"], dtype=float)
        applied = np.asarray(archive["applied_action"], dtype=float)
        physics_block = np.asarray(archive["action_block_index"], dtype=int)

    progress, cross, heading_error = [], [], []
    for index, item in enumerate(contexts):
        direction = -1 if item["active_reference_direction"] == "reverse" else 1
        projected_s, distance, reference_yaw = _project_to_active_direction_run(
            position[index], reference, directions, int(item["reference_index"]), direction
        )
        progress.append(projected_s)
        cross.append(distance)
        heading_error.append(abs(_wrap(float(yaw[index] - reference_yaw))))
    progress_a = np.asarray(progress)
    cross_a = np.asarray(cross)
    heading_a = np.asarray(heading_error)
    auxiliary_error = np.asarray(
        [abs(float(item["protocol_request_component"][1]) - float(item["measured_state"][1])) for item in contexts]
    )
    clipped = np.asarray([bool(item["request_clipped"]) for item in contexts])
    supervisor = np.abs(smoothed[:, 1] - applied[:, 1]) > 1e-7

    stages = []
    for phase in _phase_ranges(contexts):
        start, end = int(phase["start"]), int(phase["end"])
        if end <= start:
            continue
        physics_mask = (physics_block >= start) & (physics_block < end)
        stages.append(
            {
                "phase": phase["phase"],
                "start_block": start,
                "end_block_exclusive": end,
                "start_time_s": float(contexts[start]["control_time_s"]),
                "end_time_s": float(contexts[end - 1]["control_time_s"]) + 0.1,
                "duration_s": float((end - start) * 0.1),
                "episode_time_fraction": float((end - start) / len(contexts)),
                "signed_reference_progress_delta_m": float(progress_a[end - 1] - progress_a[start]),
                "reference_progress_enter_m": float(progress_a[start]),
                "reference_progress_exit_m": float(progress_a[end - 1]),
                "cross_track_enter_m": float(cross_a[start]),
                "cross_track_exit_m": float(cross_a[end - 1]),
                "cross_track_peak_m": float(np.max(cross_a[start:end])),
                "heading_error_enter_rad": float(heading_a[start]),
                "heading_error_exit_rad": float(heading_a[end - 1]),
                "heading_error_peak_rad": float(np.max(heading_a[start:end])),
                "auxiliary_target_error_mean_rad": float(np.mean(auxiliary_error[start:end])),
                "auxiliary_target_error_max_rad": float(np.max(auxiliary_error[start:end])),
                "protocol_clip_fraction": float(np.mean(clipped[start:end])),
                "auxiliary_supervisor_fraction_physics_ticks": float(np.mean(supervisor[physics_mask])) if np.any(physics_mask) else None,
            }
        )

    growth = _sustained_growth_index(cross_a)
    loss = _sustained_loss_index(cross_a)
    result = {
        "case_id": metrics["case_id"],
        "source_episode": str(episode.resolve()),
        "source_is_round6_actual_online_b0_shadow_run": True,
        "termination_reason": metrics["termination_reason"],
        "elapsed_s": metrics["elapsed_s"],
        "first_direction_change_completed": bool(metrics["actual_direction_change_completed"]),
        "subsequent_direction_change": {
            "status": "not_reached",
            "reason": "no_second_direction_change_state_before_timeout",
        },
        "sustained_tracking_loss_definition": "cross-track > 0.5 m for 10 consecutive 10 Hz samples",
        "sustained_error_growth_definition": "10 consecutive 10 Hz samples with net cross-track growth >= 0.10 m and at least 8 of 9 non-decreasing steps",
        "first_sustained_error_growth": None if growth is None else {
            "block": growth,
            "time_s": float(contexts[growth]["control_time_s"]),
            "reference_progress_m": float(progress_a[growth]),
            "position_world_xy_m": position[growth].tolist(),
            "cross_track_start_m": float(cross_a[growth]),
            "cross_track_end_m": float(cross_a[growth + 9]),
        },
        "first_sustained_tracking_loss": None if loss is None else {
            "block": loss,
            "time_s": float(contexts[loss]["control_time_s"]),
            "reference_progress_m": float(progress_a[loss]),
            "position_world_xy_m": position[loss].tolist(),
            "cross_track_m": float(cross_a[loss]),
        },
        "stages": stages,
    }
    arrays = {
        "position": position,
        "reference": reference,
        "time": np.asarray([item["control_time_s"] for item in contexts], dtype=float),
        "cross": cross_a,
        "heading": heading_a,
        "progress": progress_a,
    }
    return result, arrays


def _plot(results: Sequence[tuple[dict[str, object], dict[str, np.ndarray]]], path: Path) -> None:
    figure, axes = plt.subplots(len(results), 2, figsize=(12, 4.6 * len(results)), constrained_layout=True)
    if len(results) == 1:
        axes = np.asarray([axes])
    colors = plt.get_cmap("tab10")
    for row, (result, arrays) in enumerate(results):
        ax_path, ax_error = axes[row]
        reference, position = arrays["reference"], arrays["position"]
        ax_path.plot(reference[:, 0], reference[:, 1], "k--", linewidth=1.5, label="fixed reference")
        for index, stage in enumerate(result["stages"]):
            start, end = stage["start_block"], stage["end_block_exclusive"]
            ax_path.plot(position[start:end, 0], position[start:end, 1], color=colors(index), linewidth=2, label=stage["phase"])
            ax_error.axvspan(arrays["time"][start], arrays["time"][end - 1] + 0.1, color=colors(index), alpha=0.10)
        ax_path.set_title(f"{result['case_id']}: measured trajectory")
        ax_path.set_aspect("equal", adjustable="box")
        ax_path.set_xlabel("world x (m)")
        ax_path.set_ylabel("world y (m)")
        ax_path.grid(alpha=0.25)
        ax_path.legend(fontsize=7, loc="best")
        ax_error.plot(arrays["time"], arrays["cross"], label="cross-track (m)")
        ax_error.plot(arrays["time"], arrays["heading"], label="|heading error| (rad)")
        ax_error.axhline(0.5, color="k", linestyle="--", linewidth=1, label="sustained-loss threshold")
        ax_error.set_title(f"{result['case_id']}: errors by stage")
        ax_error.set_xlabel("time (s)")
        ax_error.grid(alpha=0.25)
        ax_error.legend(fontsize=8)
    figure.savefig(path, dpi=170)
    plt.close(figure)


def run(input_root: Path, output_dir: Path) -> dict[str, object]:
    episodes = sorted(input_root.glob("episode_*"))
    analysed = [_analyse_episode(episode) for episode in episodes]
    output_dir.mkdir(parents=True, exist_ok=False)
    payload = {
        "schema_version": "round7_full_reference_failure_stages_v1",
        "measurement": {
            "progress": "independent closest projection constrained to current contiguous drive-direction run",
            "cross_track": "Euclidean distance from measured XY to that projection",
            "heading_error": "absolute wrapped root-yaw minus interpolated reference-yaw",
            "phases": "B0 state transitions; realignment ends when reference index first advances after direction switch",
        },
        "episodes": [item[0] for item in analysed],
    }
    (output_dir / "round7_reference_failure_stages.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    rows = [dict(case_id=episode["case_id"], **stage) for episode in payload["episodes"] for stage in episode["stages"]]
    with (output_dir / "round7_reference_failure_stages.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    _plot(analysed, output_dir / "round7_reference_failure_stages.png")
    return payload


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    result = run(args.input_root, args.output_dir)
    print(json.dumps({episode["case_id"]: episode["first_sustained_tracking_loss"] for episode in result["episodes"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
