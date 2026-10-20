"""Checkpoint-frame propagation through recorded and online evaluation paths."""

import importlib
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from test_candidate_rollout_dataset import _write_context, _write_formal_episode
from training.ball_wm_dataset import BallWMDataset
from training.candidate_rollout_dataset import CandidateRolloutDataset
from training.evaluate_navigation import (
    IsaacNavigationRuntime, _observation_from_state, _recorded_context_sample,
    _recorded_goal_info, advance_model_context, build_model_context,
)


@pytest.fixture()
def measured_episode(tmp_path):
    root = tmp_path / "formal"
    episode = _write_formal_episode(root, action_protocol_id="ball_action_v2_aux30")
    with np.load(episode / "states.npz") as data:
        states = {key: data[key] for key in data.files}
    half = np.sqrt(0.5)
    states["quat"][:] = (half, 0, 0, half)
    states["lin_vel"][:] = (1, 2, 3)
    states["ang_vel"][:] = (4, 5, 6)
    states["applied_action"][:] = (0.75, 0.1)
    states["commanded_action"][:] = (0.25, 0.2)
    np.savez_compressed(episode / "states.npz", **states)
    return root, states


def _dataset(root, frame="base"):
    return BallWMDataset(root, history_frames=5, num_preds=1, rollout_horizon=5,
                         image_size=(16, 16), proprio_velocity_frame=frame)


@pytest.mark.parametrize("frame,expected", [
    ("base", (2, -1, 3, 5, -4, 6)), ("world", (1, 2, 3, 4, 5, 6)),
])
def test_recorded_context_and_goal_use_the_checkpoint_velocity_frame(measured_episode, frame, expected):
    root, _ = measured_episode
    dataset = _dataset(root, frame)
    context = _recorded_context_sample(dataset, 0, 0)
    goal = _recorded_goal_info(dataset, 0, 4)
    np.testing.assert_allclose(context["proprio"][:, :6], np.tile(expected, (5, 1)), atol=1e-6)
    np.testing.assert_allclose(goal["proprio"][0, 0, :6], expected, atol=1e-6)
    np.testing.assert_array_equal(goal["action"], np.zeros((1, 1, 10)))


def _snapshot(states, **changes):
    values = dict(
        left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        position=states["pos"][24], quaternion_wxyz=states["quat"][24],
        linear_velocity=states["lin_vel"][24], angular_velocity=states["ang_vel"][24],
        joint_positions=states["joint_pos"][24], joint_velocities=states["joint_vel"][24],
        previous_action=states["applied_action"][24], clearance_m=1.0, image_size=(16, 16),
    )
    values.update(changes)
    return _observation_from_state(**values)


def test_current_feedback_uses_measured_base_frame_without_rotating_old_rows(measured_episode):
    root, states = measured_episode
    dataset = _dataset(root)
    context = build_model_context(_recorded_context_sample(dataset, 0, 0), 5)
    half = np.sqrt(0.5)
    current = _snapshot(states, quaternion_wxyz=np.asarray((half, half, 0, 0)),
                        proprio_velocity_frame="base")
    advanced = advance_model_context(context, np.zeros(10, dtype=np.float32), current)
    np.testing.assert_allclose(advanced["proprio"][0, :-1, :6], np.tile((2, -1, 3, 5, -4, 6), (4, 1)), atol=1e-6)
    np.testing.assert_allclose(advanced["proprio"][0, -1, :6], (1, 3, -2, 4, 6, -5), atol=1e-6)
    np.testing.assert_array_equal(advanced["proprio"][0, -1, 15:], states["applied_action"][24])


def test_default_live_observation_remains_world_frame(measured_episode):
    _, states = measured_episode
    current = _snapshot(states)
    np.testing.assert_array_equal(current.proprio[:6], (1, 2, 3, 4, 5, 6))


def test_navigation_capture_forwards_selected_frame_and_current_quaternion(tmp_path, measured_episode, monkeypatch):
    _, states = measured_episode
    monkeypatch.setattr(IsaacNavigationRuntime, "_initialize", lambda _self: None)
    scene = tmp_path / "scene.usd"
    scene.write_text("test-only scene path")
    runtime = IsaacNavigationRuntime(scene, 16, 0.4, "cpu", proprio_velocity_frame="base")
    runtime._robot = SimpleNamespace(
        get_world_pose=lambda: (states["pos"][24], states["quat"][24]),
        get_linear_velocity=lambda: states["lin_vel"][24],
        get_angular_velocity=lambda: states["ang_vel"][24],
        get_joint_positions=lambda: states["joint_pos"][24],
        get_joint_velocities=lambda: states["joint_vel"][24],
    )
    runtime._occupancy = SimpleNamespace(grid=np.zeros((2, 2)), global_origin_xy=np.zeros(2))
    runtime._contract = SimpleNamespace(bev=object())
    runtime._crop_global_occupancy = lambda *_args: np.zeros((2, 2))
    runtime._obstacle_clearance = lambda *_args: 1.0
    runtime._left_camera = runtime._right_camera = object()
    runtime._get_rgb = lambda _camera: np.zeros((480, 640, 3), dtype=np.uint8)
    observation = runtime._capture_observation()
    np.testing.assert_allclose(observation.proprio[:6], (2, -1, 3, 5, -4, 6), atol=1e-6)


def test_candidate_dataset_forwards_frame_and_keeps_default_legacy(measured_episode, tmp_path):
    root, _ = measured_episode
    candidates = tmp_path / "candidate"
    _write_context(candidates, root, action_protocol_id="ball_action_v2_aux30")
    base = CandidateRolloutDataset(candidates, root, image_size=(16, 16), proprio_velocity_frame="base")[0]
    legacy = CandidateRolloutDataset(candidates, root, image_size=(16, 16))[0]
    np.testing.assert_allclose(base["proprio"][:, :6], np.tile((2, -1, 3, 5, -4, 6), (5, 1)), atol=1e-6)
    np.testing.assert_array_equal(legacy["proprio"][:, :6], np.tile((1, 2, 3, 4, 5, 6), (5, 1)))
    np.testing.assert_array_equal(base["candidate_action"], legacy["candidate_action"])
    np.testing.assert_array_equal(base["proprio"][:, 6:], legacy["proprio"][:, 6:])


class _VisualProbe(torch.nn.Module):
    """Replace expensive neural prediction, leaving evaluator data handling real."""

    def rollout(self, context, future_action, history_size):
        self.context = context
        self.future_action = future_action
        return torch.zeros((1, 1, 5, 1))

    def bev_head(self, _predicted):
        return torch.zeros((1, 1, 5, 3, 80, 80))

    def current_head(self, _predicted):
        return torch.zeros((1, 1, 5, 1, 80, 80))

    history_head = current_head


def test_visual_evaluator_uses_checkpoint_frame_and_commanded_actions(measured_episode, tmp_path, monkeypatch):
    from training import visualize_predictions

    root, _ = measured_episode
    checkpoint = tmp_path / "visual.ckpt"
    torch.save({"model": {}, "action_protocol_id": "ball_action_v2_aux30", "config": {
        "image_size": 16, "rollout_horizon": 5, "proprio_velocity_frame": "base",
        "action_source": "commanded_action",
    }}, checkpoint)
    probe = _VisualProbe()
    monkeypatch.setattr(visualize_predictions, "build_joint_model", lambda _config: probe)
    report = visualize_predictions.evaluate_visuals(checkpoint, root, tmp_path / "visual", 1, 0, "cpu")
    np.testing.assert_allclose(probe.context["proprio"][0, :, :6], np.tile((2, -1, 3, 5, -4, 6), (5, 1)), atol=1e-6)
    np.testing.assert_array_equal(probe.future_action[0, 0, :, 0::2], np.full((5, 5), 0.25))
    np.testing.assert_array_equal(probe.context["proprio"][0, :, 15], np.full(5, 0.75))
    assert report["num_samples"] == 1
    assert Path(report["records"][0]["image"]).is_file()


@pytest.mark.parametrize("consumer", ["evaluate_world_model", "evaluate_pointgoal", "evaluate_navigation",
                                      "evaluate_planner_calibration", "visualize_predictions", "evaluate_candidate_ranking"])
def test_consumers_reject_checkpoint_frame_conflicts_before_reading_data(consumer, tmp_path, monkeypatch):
    module = importlib.import_module(f"training.{consumer}")
    checkpoint = tmp_path / "conflict.ckpt"
    torch.save({"model": {}, "action_protocol_id": "ball_action_v2_aux30",
                "proprio_velocity_frame": "base", "config": {
                    "proprio_velocity_frame": "world", "pose_head_enabled": True, "rollout_horizon": 5,
                }}, checkpoint)
    data = tmp_path / "no-data"
    output = tmp_path / "out.json"
    import training.joint_jepa as joint_jepa
    monkeypatch.setattr(joint_jepa, "build_joint_model", lambda _config: torch.nn.Identity())
    if hasattr(module, "build_joint_model"):
        monkeypatch.setattr(module, "build_joint_model", lambda _config: torch.nn.Identity())
    with pytest.raises(ValueError, match="proprio_velocity_frame|velocity frame"):
        if consumer == "evaluate_world_model":
            module.run_future_prediction_evaluation(checkpoint_path=checkpoint, data_root=data,
                output_path=output, device_name="cpu")
        elif consumer == "evaluate_pointgoal":
            monkeypatch.setattr(sys, "argv", ["evaluate", "--checkpoint", str(checkpoint), "--data_root", str(data), "--device", "cpu"])
            module.main()
        elif consumer == "evaluate_navigation":
            module.run_navigation_evaluation(checkpoint, data, output, None, 1, 5, 0.25, 1,
                5, 8, 2, 1, 0.0, "cem", 1, "cpu")
        elif consumer == "evaluate_planner_calibration":
            module.run_planner_calibration(checkpoint_path=checkpoint, data_root=data,
                context_manifest_path=tmp_path / "manifest.json", output_path=output,
                scene_name=None, device_name="cpu")
        elif consumer == "visualize_predictions":
            module.evaluate_visuals(checkpoint, data, tmp_path / "visual", 1, 0, "cpu")
        else:
            module.evaluate_candidate_ranking(checkpoint_path=checkpoint, candidate_root=data,
                formal_data_root=data, device_name="cpu")
