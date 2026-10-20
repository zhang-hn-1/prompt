"""Pure loader tests for candidate rollout contexts."""

import json
from pathlib import Path

import numpy as np
import pytest
import torch
from PIL import Image

from training.candidate_rollout_dataset import CandidateRolloutDataset
from training.candidate_rollout_schema import write_candidate_context_atomic


def _write_formal_episode(
    root: Path,
    scene: str = "scene_a",
    episode_name: str = "ep0000",
    action_protocol_id: str | None = None,
) -> Path:
    episode = root / scene / episode_name
    episode.mkdir(parents=True)
    num_states = 70
    num_frames = 12
    states = {
        "physics_step": np.arange(1, num_states + 1, dtype=np.int64),
        "sim_time": np.arange(num_states, dtype=np.float64) * 0.02,
        "state_timestamp": np.arange(num_states, dtype=np.float64) * 0.02,
        "pos": np.zeros((num_states, 3), dtype=np.float32),
        "quat": np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (num_states, 1)),
        "lin_vel": np.zeros((num_states, 3), dtype=np.float32),
        "ang_vel": np.zeros((num_states, 3), dtype=np.float32),
        "joint_pos": np.zeros((num_states, 3), dtype=np.float32),
        "joint_vel": np.zeros((num_states, 3), dtype=np.float32),
        "commanded_action": np.zeros((num_states, 2), dtype=np.float32),
        "applied_action": np.zeros((num_states, 2), dtype=np.float32),
        "action_command_timestamp": np.arange(num_states, dtype=np.float64) * 0.02,
        "action_applied_timestamp": np.arange(num_states, dtype=np.float64) * 0.02,
        "stereo_frame_id": np.arange(num_frames, dtype=np.int64),
        "stereo_physics_step": (np.arange(num_frames) + 1) * 5,
        "left_timestamp": np.arange(num_frames, dtype=np.float64) * 0.1,
        "right_timestamp": np.arange(num_frames, dtype=np.float64) * 0.1,
        "cam_left_pose": np.tile(np.eye(4), (num_frames, 1, 1)).astype(np.float32),
        "cam_right_pose": np.tile(np.eye(4), (num_frames, 1, 1)).astype(np.float32),
        "left_filename": np.array([f"left_{i:06d}.png" for i in range(num_frames)]),
        "right_filename": np.array([f"right_{i:06d}.png" for i in range(num_frames)]),
        "collision": np.zeros(num_frames, dtype=bool),
        "minimum_obstacle_distance": np.full(num_frames, 1.0, dtype=np.float32),
    }
    np.savez_compressed(episode / "states.npz", **states)
    for frame in range(num_frames):
        Image.new("RGB", (16, 16), (frame, frame, frame)).save(episode / f"left_{frame:06d}.png")
        Image.new("RGB", (16, 16), (frame, frame, frame)).save(episode / f"right_{frame:06d}.png")
        bev = np.zeros((80, 80), dtype=np.uint8)
        Image.fromarray(bev).save(episode / f"geometry_bev_{frame:06d}.png")
        Image.fromarray(bev).save(episode / f"current_visible_{frame:06d}.png")
        Image.fromarray(bev).save(episode / f"history_observed_{frame:06d}.png")
    metadata = {"schema_version": "1.2.0", "scene": scene}
    if action_protocol_id is not None:
        metadata.update(action_protocol_id=action_protocol_id, controls_per_block=5)
    (episode / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    return episode


def _write_context(
    root: Path,
    formal_root: Path,
    scene: str = "scene_a",
    context_name: str = "context_0000",
    action_protocol_id: str | None = None,
) -> None:
    arrays = {
        "action": np.zeros((8, 5, 10), dtype=np.float32),
        "future_relative_pose": np.zeros((8, 5, 4), dtype=np.float32),
        "goal_distance_m": np.arange(8, dtype=np.float32),
        "collision": np.zeros((8, 5), dtype=np.uint8),
        "clearance_m": np.full((8, 5), np.inf, dtype=np.float32),
    }
    arrays["future_relative_pose"][..., 3] = 1.0
    metadata = {
        "schema_version": "candidate_rollout_v1",
        "source_episode": f"{scene}/ep0000",
        "context_start": 0,
        "history_frames": 5,
        "rollout_horizon": 5,
        "action_block": 5,
        "goal_relative_xy": [1.0, 0.0],
        "scene": scene,
        "seed": 1,
    }
    if action_protocol_id is not None:
        metadata.update(action_protocol_id=action_protocol_id, controls_per_block=5)
    write_candidate_context_atomic(root / scene / context_name, metadata, arrays)


@pytest.fixture()
def synthetic_roots(tmp_path: Path):
    formal_root = tmp_path / "formal"
    candidate_root = tmp_path / "candidate"
    _write_formal_episode(formal_root)
    _write_context(candidate_root, formal_root)
    return candidate_root, formal_root


def test_candidate_dataset_loads_context_and_preserves_candidate_alignment(synthetic_roots):
    candidate_root, formal_root = synthetic_roots
    dataset = CandidateRolloutDataset(
        candidate_root, formal_root, image_size=(16, 16), expected_candidates=8
    )
    sample = dataset[0]
    assert sample["pixels_left"].shape == (5, 3, 16, 16)
    assert sample["context_action"].shape == (5, 10)
    assert sample["candidate_action"].shape == (8, 5, 10)
    assert sample["future_relative_pose"].shape == (8, 5, 4)
    torch.testing.assert_close(sample["goal_distance_m"], torch.arange(8, dtype=torch.float32))


def test_candidate_dataset_never_discovers_partial_context_or_path_escape(tmp_path: Path):
    formal_root = tmp_path / "formal"
    _write_formal_episode(formal_root)
    partial = tmp_path / "candidate" / "scene_a" / "context_0001.partial"
    partial.mkdir(parents=True)
    (partial / "context.json").write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError, match="source_episode"):
        _write_context(tmp_path / "candidate", formal_root, context_name="context_0000")
        metadata_path = tmp_path / "candidate" / "scene_a" / "context_0000" / "context.json"
        values = json.loads(metadata_path.read_text())
        values["source_episode"] = "../ep0000"
        metadata_path.write_text(json.dumps(values))
        CandidateRolloutDataset(tmp_path / "candidate", formal_root)


def test_candidate_dataset_rejects_protocol_different_from_formal_data(tmp_path: Path):
    formal_root = tmp_path / "formal"
    candidate_root = tmp_path / "candidate"
    _write_formal_episode(
        formal_root, action_protocol_id="ball_action_v2_aux30"
    )
    _write_context(candidate_root, formal_root)

    with pytest.raises(ValueError, match="action protocol"):
        CandidateRolloutDataset(candidate_root, formal_root, image_size=(16, 16))
