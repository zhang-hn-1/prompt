from pathlib import Path
import sys

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import training.train_joint as train_joint
from training.joint_jepa import JointModelConfig
from training.train_joint import _load_config


def test_candidate_config_is_separated_from_joint_model_config():
    split = getattr(train_joint, "split_training_config", None)
    if split is None:
        pytest.fail("split_training_config is not available")
    model_values, candidate_values = split(
        {
            "embed_dim": 64,
            "candidate_root": "/tmp/candidates",
            "candidate_batch_ratio": 0.25,
        }
    )
    assert model_values == {"embed_dim": 64}
    assert candidate_values["candidate_root"] == "/tmp/candidates"


def test_candidate_training_loss_combines_pose_and_ranking_terms():
    candidate_loss = getattr(train_joint, "candidate_training_losses", None)
    if candidate_loss is None:
        pytest.fail("candidate_training_losses is not available")

    class FakeModel(torch.nn.Module):
        def rollout(self, context, action_sequence, history_size):
            del context, history_size
            return action_sequence[..., :4]

        def decode_relative_pose(self, future_embeddings):
            return future_embeddings

    batch = {
        "pixels_left": torch.zeros(1, 5, 3, 8, 8),
        "pixels_right": torch.zeros(1, 5, 3, 8, 8),
        "proprio": torch.zeros(1, 5, 17),
        "context_action": torch.zeros(1, 5, 10),
        "candidate_action": torch.zeros(1, 8, 5, 10),
        "goal_relative_xy": torch.zeros(1, 2),
        "future_relative_pose": torch.zeros(1, 8, 5, 4),
        "goal_distance_m": torch.arange(8, dtype=torch.float32).reshape(1, 8),
        "collision": torch.zeros(1, 8, 5, dtype=torch.uint8),
    }
    losses = candidate_loss(
        FakeModel(),
        batch,
        pose_weight=1.0,
        ranking_weight=1.0,
        margin_m=0.05,
    )
    assert set(("candidate_loss", "candidate_pose_loss", "candidate_ranking_loss", "candidate_ranking_accuracy")) <= set(losses)
    assert torch.isfinite(losses["candidate_loss"])


def test_load_config_accepts_repository_yaml(tmp_path: Path):
    config = tmp_path / "train.yaml"
    config.write_text("image_size: 32\nembed_dim: 64\n")

    assert _load_config(config) == {"image_size": 32, "embed_dim": 64}


def test_load_config_rejects_non_mapping(tmp_path: Path):
    config = tmp_path / "bad.yaml"
    config.write_text("- image_size\n")

    with pytest.raises(ValueError, match="mapping"):
        _load_config(config)


def test_local_pose_config_selects_required_future_horizons():
    """Catches an experiment config that forgets either pose supervision or the five-step rollout."""

    config_path = Path(__file__).resolve().parents[1] / "configs" / "train_joint_pose_local.yaml"
    config = JointModelConfig(**_load_config(config_path))

    assert config.pose_head_enabled is True
    assert tuple(config.pose_horizons) == (1, 3, 5)
    assert config.rollout_horizon == 5
    assert config.bev_decoder == "spatial"


def test_candidate_config_keeps_five_step_pose_and_ranking_protocol():
    config_path = Path(__file__).resolve().parents[1] / "configs" / "train_joint_pose_candidate.yaml"
    loaded = _load_config(config_path)
    model_values, candidate_values = train_joint.split_training_config(loaded)
    config = JointModelConfig(**model_values)

    assert config.pose_head_enabled is True
    assert config.rollout_horizon == 5
    assert tuple(candidate_values["candidate_pose_horizons"]) == (1, 3, 5)
    assert candidate_values["candidate_ranking_margin_m"] == pytest.approx(0.05)


def test_visibility_config_keeps_full_geometry_target_and_enables_union_weights():
    config_path = Path(__file__).resolve().parents[1] / "configs" / "train_joint_pose_visibility.yaml"
    config = JointModelConfig(**_load_config(config_path))

    assert config.bev_visibility_mode == "union"
    assert config.bev_visible_weight == pytest.approx(4.0)
    assert config.bev_unobserved_weight == pytest.approx(0.25)


def test_epoch_totals_include_rollout_pose_loss():
    """Catches a trainer that computes pose supervision but silently omits it from logs/checkpoints."""

    loss_keys = getattr(train_joint, "_loss_total_keys", None)
    if loss_keys is None:
        pytest.fail("_loss_total_keys is not available")

    assert "rollout_pose_loss" in loss_keys()


def test_training_artifacts_copy_dataset_action_protocol():
    class Dataset:
        action_protocol_id = "ball_action_v2_aux30"
        controls_per_block = 5

    assert train_joint.training_protocol_metadata(Dataset()) == {
        "action_protocol_id": "ball_action_v2_aux30",
        "controls_per_block": 5,
    }


def test_v2_training_rejects_unlabelled_initial_checkpoint():
    with pytest.raises(ValueError, match="ball_action_v1_aux16"):
        train_joint.require_initial_checkpoint_protocol(
            {"model": {}}, "ball_action_v2_aux30"
        )
