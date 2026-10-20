from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from training.joint_jepa import JointModelConfig
from training.visualize_predictions import (
    _future_actions,
    binary_metrics,
    multiclass_metrics,
    render_comparison,
)


def test_multiclass_metrics_reports_accuracy_and_per_class_iou():
    target = np.asarray([[0, 1, 2, 2]], dtype=np.uint8)
    predicted = np.asarray([[0, 1, 1, 2]], dtype=np.uint8)

    metrics = multiclass_metrics(predicted, target, num_classes=3)

    assert metrics["accuracy"] == 0.75
    np.testing.assert_allclose(metrics["per_class_iou"], [1.0, 0.5, 0.5])
    assert metrics["mean_iou"] == 2.0 / 3.0
    assert metrics["predicted_occupied_fraction"] == 0.5


def test_binary_metrics_handles_empty_positive_union():
    target = np.zeros((2, 2), dtype=np.uint8)
    predicted = np.zeros((2, 2), dtype=np.uint8)

    metrics = binary_metrics(predicted, target)

    assert metrics["accuracy"] == 1.0
    assert metrics["iou"] is None


def test_render_comparison_returns_rgb_contact_sheet():
    rgb = np.zeros((16, 16, 3), dtype=np.uint8)
    bev_target = np.zeros((8, 8), dtype=np.uint8)
    bev_target[2:4, 2:4] = 1
    bev_predicted = bev_target.copy()
    mask = np.zeros((8, 8), dtype=np.uint8)
    metrics = {
        "bev": {
            "accuracy": 1.0,
            "mean_iou": 1.0,
            "per_class_iou": [1.0, 1.0, None],
            "predicted_occupied_fraction": 0.25,
        },
        "current": {"accuracy": 1.0, "iou": None},
        "history": {"accuracy": 1.0, "iou": None},
    }

    image = render_comparison(
        rgb,
        rgb,
        bev_target,
        bev_predicted,
        mask,
        mask,
        mask,
        mask,
        metrics,
    )

    assert image.mode == "RGB"
    assert image.size == (960, 700)


def test_future_actions_start_at_last_context_action():
    sample = {"action": torch.arange(6 * 10, dtype=torch.float32).reshape(6, 10)}
    actions = _future_actions(sample, history=5, horizon=3)
    assert torch.equal(actions, sample["action"][4:7])


def test_old_checkpoint_config_uses_legacy_defaults():
    config = JointModelConfig(**{"image_size": 32, "embed_dim": 32, "bev_size": 16})
    assert config.bev_decoder == "mlp"
    assert config.rollout_horizon == 1
