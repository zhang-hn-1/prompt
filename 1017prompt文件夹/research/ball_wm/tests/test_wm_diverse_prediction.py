"""Causal, scene-disjoint offline motion evaluation contracts."""

import json
from dataclasses import asdict
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from evaluate_wm_diverse_prediction import (
    CausalEvaluationDataset,
    evaluate_model,
    motion_baselines,
    pose_errors,
    select_window_indices,
    summarize_rows,
    main,
)
from training.ball_wm_dataset import BallWMDataset, _load_episode
from training.joint_jepa import JointModelConfig, build_joint_model
from test_ball_wm_dataset import _write_synthetic_episode


def test_fixed_stride_restarts_per_episode_and_uses_only_selected_scene():
    dataset = SimpleNamespace(
        samples=[(episode, start) for episode in range(4) for start in range(12)],
        sample_scenes=[scene for scene in ("train", "val", "test", "val") for _ in range(12)],
    )
    split = {"train": ["train"], "val": ["val"], "test": ["test"]}

    assert select_window_indices(dataset, split, "val") == [12, 17, 22, 36, 41, 46]
    with pytest.raises(ValueError, match="val or test"):
        select_window_indices(dataset, split, "train")
    with pytest.raises(ValueError, match="disjoint"):
        select_window_indices(dataset, {**split, "val": ["train"]}, "val")


def test_causal_loader_never_opens_future_images_and_aligns_request_blocks(tmp_path):
    episode = _write_synthetic_episode(
        tmp_path, "ep0000", "val", num_states=50, num_frames=10,
        action_protocol_id="ball_action_v2_aux30",
    )
    with np.load(episode / "states.npz") as loaded:
        states = {key: loaded[key] for key in loaded.files}
    states["pos"][:, 0] = np.arange(50) * 0.02
    states["quat"][:] = [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)]
    states["lin_vel"][:] = [1.0, 0.0, 0.0]
    states["commanded_action"][:, 0] = np.arange(50)
    states["commanded_action"][:, 1] = -np.arange(50)
    states["applied_action"][:] = 0.25
    np.savez_compressed(episode / "states.npz", **states)
    for frame in range(5, 10):
        for side in ("left", "right"):
            (episode / f"{side}_{frame:06d}.png").unlink()
    _load_episode.cache_clear()
    source = BallWMDataset(tmp_path, history_frames=5, rollout_horizon=5, image_size=(16, 16), action_source="commanded_action")

    sample = CausalEvaluationDataset(source, [0])[0]

    assert sample["pixels_left"].shape == (5, 3, 16, 16)
    assert sample["pixels_right"].shape == (5, 3, 16, 16)
    assert sample["proprio"].shape == (5, 17)
    assert sample["action"].shape == (5, 10)
    assert sample["future_action"].shape == (5, 10)
    np.testing.assert_array_equal(sample["future_action"][0], [25, -25, 26, -26, 27, -27, 28, -28, 29, -29])
    np.testing.assert_array_equal(sample["future_action"][-1], [45, -45, 46, -46, 47, -47, 48, -48, 49, -49])
    torch.testing.assert_close(sample["action"][-1], sample["future_action"][0])
    torch.testing.assert_close(sample["proprio"][-1, -2:], torch.tensor([0.25, 0.25]))
    np.testing.assert_allclose(sample["future_relative_pose"][:, :2], [[0, -.1], [0, -.2], [0, -.3], [0, -.4], [0, -.5]], atol=1e-6)
    np.testing.assert_allclose(sample["baseline_cv"][:, :2], [[0, -.1], [0, -.2], [0, -.3], [0, -.4], [0, -.5]], atol=1e-6)
    assert sample["future_bev"].shape == (5, 80, 80)


def test_velocity_baseline_rotates_world_velocity_and_uses_past_wrapped_yaw_rate():
    zero, cv = motion_baselines(
        world_velocity=np.array([1., 2., 0.]),
        current_yaw=math.pi / 2,
        previous_yaw=math.radians(89),
        observed_dt=0.1,
        future_dt=np.array([.1, .2, .3, .4, .5]),
    )
    np.testing.assert_allclose(zero, np.tile([0, 0, 0, 1], (5, 1)))
    np.testing.assert_allclose(cv[[0, 2, 4], :2], [[.2, -.1], [.6, -.3], [1., -.5]])
    np.testing.assert_allclose(np.arctan2(cv[[0, 2, 4], 2], cv[[0, 2, 4], 3]), np.deg2rad([1, 3, 5]))
    _, wrapped = motion_baselines(np.zeros(3), math.radians(-179), math.radians(179), .1, np.array([.1]))
    assert math.atan2(wrapped[0, 2], wrapped[0, 3]) == pytest.approx(math.radians(2))


def test_pose_error_is_euclidean_and_wraps_across_pi():
    prediction = np.array([3, 4, math.sin(math.radians(179)), math.cos(math.radians(179))])
    target = np.array([0, 0, math.sin(math.radians(-179)), math.cos(math.radians(-179))])
    result = pose_errors(prediction, target)
    assert result["displacement_error_m"] == pytest.approx(5.)
    assert result["yaw_abs_error_rad"] == pytest.approx(math.radians(2))
    with pytest.raises(ValueError, match="finite"):
        pose_errors(np.full(4, np.nan), target)


class _ActionPoseModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.seen = []
        self.bev_head = None

    def rollout(self, context, actions, history_size):
        assert set(context) == {"pixels_left", "pixels_right", "proprio", "action"}
        assert all(value.shape[1] == 5 and torch.isfinite(value).all() for value in context.values())
        assert history_size == 5
        self.seen.append((context, actions.detach().clone()))
        xy = actions[..., :2].cumsum(dim=2) * 0.1
        yaw = actions[..., 1].cumsum(dim=2) * 0.2
        return torch.cat((xy, yaw.sin().unsqueeze(-1), yaw.cos().unsqueeze(-1)), dim=-1)

    def decode_relative_pose(self, embeddings):
        return embeddings


def _batch():
    return {
        "pixels_left": torch.zeros(1, 5, 3, 8, 8),
        "pixels_right": torch.zeros(1, 5, 3, 8, 8),
        "proprio": torch.zeros(1, 5, 17),
        "action": torch.zeros(1, 5, 10),
        "future_action": torch.ones(1, 5, 10),
        "future_relative_pose": torch.tensor([[[0., 0., 0., 1.]] * 5]),
        "baseline_zero": torch.tensor([[[0., 0., 0., 1.]] * 5]),
        "baseline_cv": torch.tensor([[[0., 0., 0., 1.]] * 5]),
        "future_bev": torch.ones(1, 5, 2, 2, dtype=torch.long),
        "episode": ["ep0000"], "scene": ["val"], "start_frame": torch.tensor([0]),
    }


def test_evaluation_runs_one_causal_context_for_recorded_and_counterfactual_requests():
    model = _ActionPoseModel()
    result = evaluate_model(model, [_batch()], "cpu")

    assert len(model.seen) == 1
    assert model.seen[0][1].shape == (1, 3, 5, 10)
    metrics = result["aggregate"]["per_horizon"]["0.5"]
    assert metrics["model"]["displacement_error_m_mean"] == pytest.approx(math.sqrt(.5))
    assert metrics["zero_motion"]["displacement_error_m_mean"] == 0.
    assert metrics["recursive_occupied_bev"] is None
    assert result["aggregate"]["action_diagnostics"]["prefix_causality_passed"]
    assert result["aggregate"]["action_diagnostics"]["negated_request_pose_xy_change_m_mean"] > 0.
    assert result["per_episode"][0]["window_start_frames"] == [0]
    json.dumps(result, allow_nan=False)


def test_future_labels_change_scores_but_cannot_change_model_inputs_or_predictions():
    first_model, second_model = _ActionPoseModel(), _ActionPoseModel()
    first = _batch()
    second = {name: value.clone() if torch.is_tensor(value) else value for name, value in first.items()}
    second["future_relative_pose"][..., :2] = 1000.
    second["future_bev"][:] = 2
    a = evaluate_model(first_model, [first], "cpu")
    b = evaluate_model(second_model, [second], "cpu")
    for name in first_model.seen[0][0]:
        torch.testing.assert_close(first_model.seen[0][0][name], second_model.seen[0][0][name])
    torch.testing.assert_close(first_model.seen[0][1], second_model.seen[0][1])
    assert a["aggregate"]["action_diagnostics"] == b["aggregate"]["action_diagnostics"]
    assert a["aggregate"]["per_horizon"]["0.5"]["model"] != b["aggregate"]["per_horizon"]["0.5"]["model"]


def test_summary_weights_windows_and_pools_bev_intersection_union():
    def row(error, intersection, union):
        return {
            "per_horizon": {str(h): {
                **{name: {"displacement_error_m": error, "yaw_abs_error_rad": 0.}
                   for name in ("model", "zero_motion", "constant_velocity")},
                "recursive_occupied_bev": {"intersection": intersection, "union": union},
            } for h in (.1, .3, .5)},
            "action_diagnostics": {"prefix_latent_max_abs_delta": 0., "negated_request_pose_xy_change_m": .2, "negated_request_pose_yaw_change_rad": 0., "request_change_l1_mean": 1.},
        }
    result = summarize_rows([row(1., 1, 2), row(3., 3, 3), row(3., 3, 3), row(3., 3, 3)])
    assert result["window_count"] == 4
    assert result["per_horizon"]["0.1"]["model"]["displacement_error_m_mean"] == 2.5
    assert result["per_horizon"]["0.1"]["recursive_occupied_bev"]["occupied_iou"] == pytest.approx(10 / 11)
    with pytest.raises(ValueError, match="no windows"):
        summarize_rows([])


def test_prefix_diagnostic_detects_future_suffix_leaking_into_earlier_predictions():
    class LeakingModel(_ActionPoseModel):
        def rollout(self, context, actions, history_size):
            output = super().rollout(context, actions, history_size)
            output[..., 0] += actions[..., 0].sum(dim=2, keepdim=True)
            return output

    report = evaluate_model(LeakingModel(), [_batch()], "cpu")

    assert not report["aggregate"]["action_diagnostics"]["prefix_causality_passed"]
    assert report["aggregate"]["action_diagnostics"]["prefix_latent_max_abs_delta"] > 0


def test_cli_evaluates_real_checkpoint_on_cpu_without_future_or_other_split_rgb(tmp_path):
    """Exercises real encode/rollout/pose/BEV heads and the checkpoint CLI path."""

    from PIL import Image

    for scene in ("train", "val", "test"):
        episode = _write_synthetic_episode(
            tmp_path / "data" / scene, "ep0000", scene,
            num_states=50, num_frames=10, action_protocol_id="ball_action_v2_aux30",
        )
        for frame in range(10):
            labels = np.zeros((16, 16), dtype=np.uint8)
            labels[3:5, 6:8] = 1
            Image.fromarray(labels).save(episode / f"geometry_bev_{frame:06d}.png")
            if scene != "val" or frame >= 5:
                for side in ("left", "right"):
                    (episode / f"{side}_{frame:06d}.png").unlink()
    config = JointModelConfig(
        image_size=32, embed_dim=192, bev_size=16, history_size=5,
        rollout_horizon=5, pose_head_enabled=True, action_source="commanded_action",
        predictor_depth=1, predictor_heads=2, predictor_hidden_dim=32,
        predictor_dim_head=8, predictor_mlp_dim=64,
    )
    torch.manual_seed(67)
    model = build_joint_model(config)
    for block in model.predictor.transformer.layers:
        torch.nn.init.normal_(block.adaLN_modulation[-1].weight, std=.2)
    checkpoint = tmp_path / "model.ckpt"
    torch.save({"config": asdict(config), "model": model.state_dict(), "epoch": 2,
                "action_source": "commanded_action", "action_protocol_id": "ball_action_v2_aux30"}, checkpoint)
    manifest = tmp_path / "split.json"
    manifest.write_text(json.dumps({"train": ["train"], "val": ["val"], "test": ["test"]}))
    output = tmp_path / "scores.json"

    report = main([
        "--checkpoint", str(checkpoint), "--data_root", str(tmp_path / "data"),
        "--scene_split_manifest", str(manifest), "--split", "val",
        "--output", str(output), "--device", "cpu",
    ])

    assert report["aggregate"]["window_count"] == 1
    assert report["checkpoint_epoch"] == 2
    assert report["aggregate"]["action_diagnostics"]["prefix_causality_passed"]
    assert report["aggregate"]["action_diagnostics"]["negated_request_pose_xy_change_m_mean"] > 0
    for metrics in report["aggregate"]["per_horizon"].values():
        assert 0 <= metrics["recursive_occupied_bev"]["occupied_iou"] <= 1
    assert json.loads(output.read_text()) == report
