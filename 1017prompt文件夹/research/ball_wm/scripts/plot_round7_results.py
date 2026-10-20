#!/usr/bin/env python3
"""Plot the frozen Round 7 M2 validation metrics from the versioned JSON bundle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    payload = json.loads(args.validation.read_text(encoding="utf-8"))
    rows = [row for row in payload["m2_fresh_validation_rows"] if row["horizon_s"] == 1.5]
    labels = [row["sequence_id"].replace("r7_fresh_", "") for row in rows]
    main = [abs(row["terminal_physical_state_error"]["main_joint_velocity_rad_s"]) for row in rows]
    aux = [abs(row["terminal_physical_state_error"]["auxiliary_joint_rad"]) for row in rows]
    trajectory = [row["trajectory_rms_error_m"] for row in rows]

    figure, axes = plt.subplots(1, 2, figsize=(13, 4.8), constrained_layout=True)
    x = np.arange(len(rows))
    width = 0.25
    axes[0].bar(x - width, trajectory, width, label="trajectory RMS (m)")
    axes[0].bar(x, main, width, label="terminal main |error| (rad/s)")
    axes[0].bar(x + width, aux, width, label="terminal aux |error| (rad)")
    axes[0].axhline(0.10, color="C0", linestyle="--", linewidth=1)
    axes[0].axhline(0.08, color="C1", linestyle="--", linewidth=1)
    axes[0].axhline(0.05, color="C2", linestyle="--", linewidth=1)
    axes[0].set_xticks(x, labels, rotation=20, ha="right")
    axes[0].set_title("M2 fresh whole-sequence gates at 1.5 s")
    axes[0].grid(axis="y", alpha=0.25)
    axes[0].legend(fontsize=8)

    events = [
        row for row in payload["m2_fresh_event_matrix"]
        if row["horizon_s"] == 6.0 and row["actual_status"] == "observed_within_horizon"
    ]
    event_labels = [f"{row['sequence_id'].replace('r7_fresh_', '')}\n{row['event']}" for row in events]
    actual = [row["actual_event_time_s"] for row in events]
    predicted = [row["predicted_event_time_s"] for row in events]
    ex = np.arange(len(events))
    axes[1].bar(ex - 0.18, actual, 0.36, label="measured")
    axes[1].bar(ex + 0.18, predicted, 0.36, label="M2")
    axes[1].set_xticks(ex, event_labels, rotation=25, ha="right", fontsize=7)
    axes[1].set_ylabel("event time (s)")
    axes[1].set_title("Observed fresh events (6 s observation)")
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].legend()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=170)
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
