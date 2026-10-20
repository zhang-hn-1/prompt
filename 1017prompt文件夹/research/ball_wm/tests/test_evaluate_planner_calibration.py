"""Isaac-free aggregation tests for planner calibration artifacts."""

import numpy as np
import pytest

from training.action_protocol import ActionProtocol

try:
    from training.evaluate_planner_calibration import summarize_calibration_contexts
except ModuleNotFoundError:
    summarize_calibration_contexts = None

try:
    from training.evaluate_planner_calibration import manifest_goal_offset
except ImportError:
    manifest_goal_offset = None

try:
    from training.evaluate_planner_calibration import manifest_navigation_protocol
except ImportError:
    manifest_navigation_protocol = None

try:
    from training.evaluate_planner_calibration import _fixed_action_candidates
except ImportError:
    _fixed_action_candidates = None


def _aux30_protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )


def test_calibration_summary_detects_matching_cem_top_choice():
    """Catches a report that cannot tell when the CEM score picks the real best candidate."""

    assert summarize_calibration_contexts is not None, "calibration summary is not available"
    result = summarize_calibration_contexts(
        [
            {
                "candidates": [
                    {"name": "zero", "predicted_total_cost": 2.0, "actual_final_distance_m": 2.0},
                    {"name": "random", "predicted_total_cost": 3.0, "actual_final_distance_m": 3.0},
                    {"name": "cem", "predicted_total_cost": 1.0, "actual_final_distance_m": 1.0},
                ]
            }
        ]
    )

    assert result["top1_match_rate"] == pytest.approx(1.0)
    assert result["cem_better_than_zero_rate"] == pytest.approx(1.0)
    assert result["cem_better_than_random_rate"] == pytest.approx(1.0)
    assert result["spearman_rho_mean"] == pytest.approx(1.0)


def test_calibration_summary_rejects_mismatched_candidate_sets():
    """Catches aggregate wins computed over incomparable candidate sets."""

    assert summarize_calibration_contexts is not None, "calibration summary is not available"
    with pytest.raises(ValueError, match="candidate names"):
        summarize_calibration_contexts(
            [
                {
                    "candidates": [
                        {"name": "zero", "predicted_total_cost": 2.0, "actual_final_distance_m": 2.0},
                        {"name": "cem", "predicted_total_cost": 1.0, "actual_final_distance_m": 1.0},
                    ]
                },
                {
                    "candidates": [
                        {"name": "zero", "predicted_total_cost": 2.0, "actual_final_distance_m": 2.0},
                        {"name": "random", "predicted_total_cost": 1.0, "actual_final_distance_m": 1.0},
                    ]
                },
            ]
        )


def test_calibration_reads_goal_offset_from_the_frozen_manifest(tmp_path):
    """Catches calibration silently changing a strict suite's PointGoal horizon."""

    assert manifest_goal_offset is not None, "manifest goal-offset reader is not available"
    path = tmp_path / "contexts.json"
    path.write_text(
        '{"goal_offset": 20, "min_initial_goal_distance_m": 0.5, "require_initial_clearance": true}'
    )

    assert manifest_goal_offset(path) == 20


def test_calibration_inherits_all_frozen_navigation_protocol_fields(tmp_path):
    """Catches calibration asking the manifest loader for different clearance/distance constraints."""

    assert manifest_navigation_protocol is not None, "manifest protocol reader is not available"
    path = tmp_path / "contexts.json"
    path.write_text(
        '{"goal_offset": 20, "min_initial_goal_distance_m": 0.5, '
        '"min_initial_clearance_m": 0.2, "require_initial_clearance": true}'
    )

    assert manifest_navigation_protocol(path) == (20, 0.5, 0.2, True)


def test_calibration_defaults_missing_manifest_clearance_to_zero(tmp_path):
    """Old manifests remain readable with the legacy zero-threshold protocol."""

    assert manifest_navigation_protocol is not None, "manifest protocol reader is not available"
    path = tmp_path / "legacy-contexts.json"
    path.write_text(
        '{"goal_offset": 20, "min_initial_goal_distance_m": 0.5, "require_initial_clearance": true}'
    )

    assert manifest_navigation_protocol(path) == (20, 0.5, 0.0, True)


def test_fixed_calibration_candidates_use_v2_protocol_bounds():
    fixed = _fixed_action_candidates(5, np.random.default_rng(3072), _aux30_protocol())

    assert np.all(fixed["aux_positive"][:, 1::2] == pytest.approx(0.5235987756))
    assert np.all(fixed["aux_negative"][:, 1::2] == pytest.approx(-0.5235987756))
    for actions in fixed.values():
        assert np.all(actions[:, 0::2] >= -2.0)
        assert np.all(actions[:, 0::2] <= 2.0)
        assert np.all(actions[:, 1::2] >= -0.5235987756)
        assert np.all(actions[:, 1::2] <= 0.5235987756)
