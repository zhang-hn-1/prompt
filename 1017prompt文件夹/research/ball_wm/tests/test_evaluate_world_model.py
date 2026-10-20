"""Isaac-free tests for held-out future world-model prediction metrics."""

import numpy as np
import pytest
from types import SimpleNamespace

try:
    from training.evaluate_world_model import (
        _bev_confusion,
        select_future_prediction_test_indices,
        summarize_bev_confusions,
        summarize_future_predictions,
    )
except ModuleNotFoundError:
    _bev_confusion = None
    select_future_prediction_test_indices = None
    summarize_bev_confusions = None
    summarize_future_predictions = None


def test_future_prediction_test_scene_fraction_selects_multiple_disjoint_scenes():
    """The held-out evaluator must support more than one unseen scene."""

    assert select_future_prediction_test_indices is not None, "scene selection helper is not available"
    dataset = SimpleNamespace(
        samples=list(range(8)),
        sample_scenes=["scene_a", "scene_a", "scene_b", "scene_b", "scene_c", "scene_c", "scene_d", "scene_d"],
    )
    dataset.scene_splits = lambda ratios, seed, unseen_scene_test: _scene_splits(
        dataset.sample_scenes, ratios, seed, unseen_scene_test
    )

    test_indices, held_out_scenes = select_future_prediction_test_indices(
        dataset,
        seed=3072,
        test_scene_fraction=0.5,
    )

    assert len(held_out_scenes) == 2
    assert len(test_indices) == 4
    assert {dataset.sample_scenes[index] for index in test_indices} == set(held_out_scenes)


def _scene_splits(sample_scenes, ratios, seed, unseen_scene_test):
    rng = np.random.default_rng(seed)
    scenes = sorted(set(sample_scenes))
    rng.shuffle(scenes)
    train_count = max(1, int(round(ratios[0] * len(scenes))))
    val_count = max(0, int(round(ratios[1] * len(scenes))))
    train_scenes = set(scenes[:train_count])
    val_scenes = set(scenes[train_count : train_count + val_count])
    test_scenes = set(scenes[train_count + val_count :])
    indices = lambda selected: sorted(i for i, scene in enumerate(sample_scenes) if scene in selected)
    return indices(train_scenes), indices(val_scenes), indices(test_scenes)


def test_future_prediction_metrics_report_each_requested_horizon():
    """Catches a report that omits a requested future horizon or averages XY incorrectly."""

    assert summarize_future_predictions is not None, "future-world-model metric aggregator is not available"
    metrics = summarize_future_predictions(
        feature_cosines={1: [0.9], 3: [0.7], 5: [0.5]},
        feature_errors={1: [0.1], 3: [0.3], 5: [0.5]},
        pose_predictions={1: [np.array([1.0, 0.0, 0.0, 1.0])]},
        pose_targets={1: [np.array([0.0, 0.0, 0.0, 1.0])]},
        bev_confusions={1: np.eye(3, dtype=np.int64)},
        horizons=(1, 3, 5),
    )

    assert set(metrics["per_horizon"]) == {"1", "3", "5"}
    assert metrics["per_horizon"]["1"]["feature_cosine"] == pytest.approx(0.9)
    assert metrics["per_horizon"]["1"]["xy_mae_m"] == pytest.approx(0.5)
    assert metrics["per_horizon"]["1"]["yaw_mae_deg"] == pytest.approx(0.0)


def test_future_prediction_metrics_use_wrapped_yaw_and_finite_empty_bev_scores():
    """Catches NaN empty-class IoU/F1 and sin/cos yaw metrics that ignore angle geometry."""

    assert summarize_future_predictions is not None, "future-world-model metric aggregator is not available"
    metrics = summarize_future_predictions(
        feature_cosines={1: [1.0]},
        feature_errors={1: [0.0]},
        pose_predictions={1: [np.array([0.0, 0.0, 1.0, 0.0])]},
        pose_targets={1: [np.array([0.0, 0.0, 0.0, 1.0])]},
        bev_confusions={1: np.diag([8, 0, 4]).astype(np.int64)},
        horizons=(1,),
    )

    horizon = metrics["per_horizon"]["1"]
    assert horizon["yaw_mae_deg"] == pytest.approx(90.0)
    assert horizon["occupied_iou"] == pytest.approx(0.0)
    assert horizon["occupied_f1"] == pytest.approx(0.0)
    assert all(
        np.isfinite(float(value))
        for value in horizon.values()
        if not isinstance(value, list)
    )


def test_future_prediction_metrics_report_occupied_calibration():
    """Distinguishes occupied under-prediction from a genuinely accurate BEV map."""

    assert summarize_future_predictions is not None, "future-world-model metric aggregator is not available"
    metrics = summarize_future_predictions(
        feature_cosines={1: [1.0]},
        feature_errors={1: [0.0]},
        pose_predictions={1: [np.zeros(4)]},
        pose_targets={1: [np.zeros(4)]},
        bev_confusions={1: np.array([[70, 5, 0], [20, 5, 0], [0, 0, 0]], dtype=np.int64)},
        horizons=(1,),
    )

    horizon = metrics["per_horizon"]["1"]
    assert horizon["target_occupied_fraction"] == pytest.approx(0.25)
    assert horizon["predicted_occupied_fraction"] == pytest.approx(0.10)
    assert horizon["occupied_precision"] == pytest.approx(0.5)
    assert horizon["occupied_recall"] == pytest.approx(0.2)


def test_bev_confusion_summary_is_separate_for_teacher_decoder_diagnosis():
    """Teacher-forced BEV scores isolate decoder/target error from rollout error."""

    assert summarize_bev_confusions is not None, "BEV confusion summary is not available"
    metrics = summarize_bev_confusions(
        {1: np.array([[70, 5, 0], [20, 5, 0], [0, 0, 0]], dtype=np.int64)},
        horizons=(1,),
    )
    assert metrics["1"]["occupied_iou"] == pytest.approx(5.0 / 30.0)
    assert metrics["1"]["target_occupied_fraction"] == pytest.approx(0.25)
    assert metrics["1"]["predicted_occupied_fraction"] == pytest.approx(0.10)


def test_bev_confusion_can_be_restricted_to_visible_cells():
    assert _bev_confusion is not None, "BEV confusion helper is not available"
    labels = np.array([0, 1, 1, 0], dtype=np.int64)
    predicted = np.array([0, 0, 1, 1], dtype=np.int64)
    visible = np.array([True, True, False, False])

    confusion = _bev_confusion(labels[visible], predicted[visible])

    np.testing.assert_array_equal(confusion, np.array([[1, 0, 0], [1, 0, 0], [0, 0, 0]]))
