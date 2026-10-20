import pytest

from training.evaluate_candidate_ranking import summarize_candidate_ranking


def test_candidate_ranking_summary_reports_perfect_ordering_and_spread():
    result = summarize_candidate_ranking(
        [
            {
                "predicted_goal_distance_m": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
                "target_goal_distance_m": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],
                "collision": [[0, 0, 0]] * 8,
            }
        ]
    )

    assert result["usable_contexts"] == 1
    assert result["pairwise_accuracy_mean"] == pytest.approx(1.0)
    assert result["spearman_rho_mean"] == pytest.approx(1.0)
    assert result["top1_match_rate"] == pytest.approx(1.0)
    assert result["predicted_goal_distance_std_mean"] > 0.0


def test_candidate_ranking_summary_counts_collision_as_worse_than_clear_candidate():
    result = summarize_candidate_ranking(
        [
            {
                "predicted_goal_distance_m": [0.2, 1.0, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9],
                "target_goal_distance_m": [0.2, 0.1, 0.3, 0.5, 0.6, 0.7, 0.8, 0.9],
                "collision": [[0, 0, 0], [1, 1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
            }
        ]
    )

    assert result["collision_candidate_fraction"] == pytest.approx(1 / 8)
    assert result["top1_match_rate"] == pytest.approx(1.0)
    assert result["pairwise_accuracy_mean"] == pytest.approx(1.0)


def test_candidate_ranking_summary_rejects_mismatched_shapes():
    with pytest.raises(ValueError, match="same candidate count"):
        summarize_candidate_ranking(
            [
                {
                    "predicted_goal_distance_m": [0.1] * 8,
                    "target_goal_distance_m": [0.1] * 9,
                    "collision": [[0, 0]] * 8,
                }
            ]
        )
