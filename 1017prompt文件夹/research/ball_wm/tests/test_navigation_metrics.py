"""Pure metric tests for calibration and strict navigation comparison."""

import numpy as np
import pytest

try:
    from training.navigation_metrics import spearman_rank_correlation
except ImportError:
    spearman_rank_correlation = None

try:
    from training.navigation_metrics import bootstrap_interval
except ImportError:
    bootstrap_interval = None

try:
    from training.navigation_metrics import cluster_bootstrap_interval, episode_termination
except ImportError:
    cluster_bootstrap_interval = None
    episode_termination = None


def test_spearman_rank_correlation_is_one_for_identical_cost_ordering():
    """Catches a ranking metric that compares raw scales rather than candidate order."""

    assert spearman_rank_correlation is not None, "Spearman metric is not available"
    assert spearman_rank_correlation([0.1, 0.2, 0.3], [1.0, 2.0, 3.0]) == pytest.approx(1.0)


def test_spearman_rank_correlation_is_finite_for_tied_predictions():
    """Catches undefined calibration metrics when CEM candidates have equal predicted costs."""

    assert spearman_rank_correlation is not None, "Spearman metric is not available"
    assert spearman_rank_correlation([1.0, 1.0, 2.0], [1.0, 2.0, 3.0]) == pytest.approx(0.8660254)


def test_bootstrap_interval_is_reproducible_and_bounds_estimate():
    """Catches non-deterministic confidence intervals or bounds that omit the raw estimate."""

    assert bootstrap_interval is not None, "bootstrap interval is not available"
    first = bootstrap_interval([0.0, 1.0, 1.0, 0.0], np.mean, seed=7, samples=1000, confidence=0.95)
    assert first["low"] <= first["estimate"] <= first["high"]
    assert first == bootstrap_interval([0.0, 1.0, 1.0, 0.0], np.mean, seed=7, samples=1000, confidence=0.95)


def test_episode_termination_prioritizes_collision_over_success_and_timeout():
    assert episode_termination is not None
    assert episode_termination(0.1, True, 200, 200, 0.2) == "collision"
    assert episode_termination(0.1, False, 200, 200, 0.2) == "success"
    assert episode_termination(1.0, False, 200, 200, 0.2) == "timeout"


def test_cluster_bootstrap_resamples_scenes_and_is_reproducible():
    assert cluster_bootstrap_interval is not None
    episodes = [
        {"scene_id": "a", "success": 1.0},
        {"scene_id": "a", "success": 1.0},
        {"scene_id": "b", "success": 0.0},
    ]

    first = cluster_bootstrap_interval(
        episodes,
        field="success",
        cluster_field="scene_id",
        seed=3072,
        samples=200,
    )

    assert first["num_clusters"] == 2
    assert first["estimate"] == pytest.approx(2.0 / 3.0)
    assert first == cluster_bootstrap_interval(
        episodes,
        field="success",
        cluster_field="scene_id",
        seed=3072,
        samples=200,
    )
