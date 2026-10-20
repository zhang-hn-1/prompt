from __future__ import annotations

import numpy as np

from benchmark.drive_model import DriveModel
from scripts.diagnose_e1_failures import (
    _classify_failure,
    _longest_true_duration,
    _sample_reference_motion,
)


def test_longest_true_duration_reports_total_and_longest_intervals() -> None:
    time_s = np.arange(7, dtype=np.float64)
    total, longest = _longest_true_duration(
        np.asarray([False, True, True, False, True, True, True]), time_s
    )
    assert total == 3.0
    assert longest == 2.0


def test_reference_motion_reconstructs_curved_planner_primitive() -> None:
    drive = DriveModel.from_json("research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json")
    pose = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [0.299, 0.008, 0.05],
            [0.598, 0.030, 0.10],
        ],
        dtype=np.float64,
    )
    command = np.asarray([[1.0, 0.0], [1.0, 0.0], [0.0, 0.0]], dtype=np.float64)
    samples = _sample_reference_motion(pose, command, drive)
    assert samples.shape[1] == 3
    assert np.all(np.isfinite(samples))
    # The reconstruction follows an arc rather than a straight chord.
    assert np.max(np.abs(samples[:, 1])) > 1e-4


def test_classification_distinguishes_tracking_collision_from_map_failure() -> None:
    base = {
        "initial_contact": False,
        "initial_collision": False,
        "reference_map_collision_free": True,
        "max_cross_track_error_m": 0.32,
        "persistent_contact": False,
        "longest_stalled_duration_s": 0.0,
    }
    category, evidence = _classify_failure(
        metrics={"termination_reason": "collision"},
        summary=base,
    )
    assert category == "trajectory_tracking_collision"
    assert "tracking error" in evidence


def test_classification_prioritizes_static_reference_failure() -> None:
    category, _ = _classify_failure(
        metrics={"termination_reason": "timeout"},
        summary={
            "initial_contact": False,
            "initial_collision": False,
            "reference_map_collision_free": False,
            "max_cross_track_error_m": 0.0,
            "persistent_contact": False,
            "longest_stalled_duration_s": 0.0,
        },
    )
    assert category == "planner_reference_infeasible"
