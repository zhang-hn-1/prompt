import json
from pathlib import Path
import sys

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from training.ball_wm_dataset import BallWMDataset, make_train_val_test_split
from training.action_protocol import LEGACY_ACTION_PROTOCOL_ID
from training.dataset_paths import discover_episode_paths
from training.dataset_stats import main as dataset_stats_main
from training.dataset_stats import summarize


def _write_synthetic_episode(
    root: Path,
    name: str,
    scene: str,
    num_states: int = 30,
    num_frames: int = 6,
    action_protocol_id: str | None = None,
    controls_per_block: int = 5,
) -> Path:
    episode = root / name
    episode.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(hash(name) % (2**32))
    applied = np.stack(
        [np.arange(num_states, dtype=np.float32) * 0.1, np.arange(num_states, dtype=np.float32) * 0.05],
        axis=1,
    )
    quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (num_states, 1))
    states = {
        "physics_step": np.arange(1, num_states + 1, dtype=np.int64),
        "sim_time": np.arange(num_states, dtype=np.float64) * 0.02,
        "state_timestamp": np.arange(num_states, dtype=np.float64) * 0.02,
        "pos": np.zeros((num_states, 3), dtype=np.float32),
        "quat": quat,
        "lin_vel": np.zeros((num_states, 3), dtype=np.float32),
        "ang_vel": np.zeros((num_states, 3), dtype=np.float32),
        "joint_pos": np.zeros((num_states, 3), dtype=np.float32),
        "joint_vel": np.zeros((num_states, 3), dtype=np.float32),
        "commanded_action": applied.copy(),
        "applied_action": applied.copy(),
        "action_command_timestamp": np.arange(num_states, dtype=np.float64) * 0.02,
        "action_applied_timestamp": np.arange(num_states, dtype=np.float64) * 0.02,
        "stereo_frame_id": np.arange(num_frames, dtype=np.int64),
        "stereo_physics_step": (np.arange(num_frames) + 1) * 5,
        "left_timestamp": np.arange(num_frames, dtype=np.float64) * 0.1,
        "right_timestamp": np.arange(num_frames, dtype=np.float64) * 0.1,
        "cam_left_pose": np.tile(np.eye(4), (num_frames, 1, 1)).astype(np.float32),
        "cam_right_pose": np.tile(np.eye(4), (num_frames, 1, 1)).astype(np.float32),
        "left_filename": np.array([f"left_{f:06d}.png" for f in range(num_frames)]),
        "right_filename": np.array([f"right_{f:06d}.png" for f in range(num_frames)]),
        "collision": np.zeros(num_frames, dtype=bool),
        "minimum_obstacle_distance": np.full(num_frames, 1.0, dtype=np.float32),
    }
    np.savez_compressed(episode / "states.npz", **states)

    for frame in range(num_frames):
        color = tuple(int(c) for c in rng.integers(0, 255, 3))
        Image.new("RGB", (16, 16), color).save(episode / f"left_{frame:06d}.png")
        Image.new("RGB", (16, 16), color).save(episode / f"right_{frame:06d}.png")
        bev = np.zeros((80, 80), dtype=np.uint8)
        bev[10, 10] = 1
        Image.fromarray(bev).save(episode / f"geometry_bev_{frame:06d}.png")
        Image.fromarray((bev > 0).astype(np.uint8)).save(episode / f"current_visible_{frame:06d}.png")
        Image.fromarray((bev > 0).astype(np.uint8)).save(episode / f"history_observed_{frame:06d}.png")

    metadata = {
        "schema_version": "1.1.0",
        "scene": scene,
        "rates_hz": {"physics": 50, "control": 50, "state": 50, "stereo": 10},
        "image_width_px": 640,
        "image_height_px": 480,
    }
    if action_protocol_id is not None:
        metadata["action_protocol_id"] = action_protocol_id
        metadata["controls_per_block"] = controls_per_block
    (episode / "metadata.json").write_text(json.dumps(metadata))
    return episode


@pytest.fixture()
def synthetic_root(tmp_path: Path) -> Path:
    _write_synthetic_episode(tmp_path, "ep0000", "scene_a")
    _write_synthetic_episode(tmp_path, "ep0001", "scene_b")
    return tmp_path


def test_dataset_returns_contract_shapes(synthetic_root: Path):
    ds = BallWMDataset(synthetic_root, history_frames=3, num_preds=1, image_size=(16, 16))
    sample = ds[0]
    assert sample["pixels_left"].shape == (4, 3, 16, 16)
    assert sample["pixels_right"].shape == (4, 3, 16, 16)
    assert sample["action"].shape == (4, 10)
    assert sample["proprio"].shape == (4, 17)
    assert sample["geometry_bev"].shape == (4, 80, 80)
    assert sample["current_visible"].shape == (4, 80, 80)
    assert sample["history_observed"].shape == (4, 80, 80)
    assert sample["collision"].shape == (4,)
    assert sample["clearance"].shape == (4,)


def test_unlabelled_dataset_is_explicitly_identified_as_legacy(synthetic_root: Path):
    dataset = BallWMDataset(
        synthetic_root, history_frames=3, num_preds=1, image_size=(16, 16)
    )

    assert dataset.action_protocol_id == LEGACY_ACTION_PROTOCOL_ID


def test_dataset_exposes_matching_v2_protocol(tmp_path: Path):
    _write_synthetic_episode(
        tmp_path,
        "ep0000",
        "scene_a",
        action_protocol_id="ball_action_v2_aux30",
    )

    dataset = BallWMDataset(
        tmp_path, history_frames=3, num_preds=1, image_size=(16, 16)
    )

    assert dataset.action_protocol_id == "ball_action_v2_aux30"
    assert dataset.controls_per_block == 5


def test_dataset_rejects_protocol_control_count_mismatch(tmp_path: Path):
    _write_synthetic_episode(
        tmp_path,
        "ep0000",
        "scene_a",
        action_protocol_id="ball_action_v2_aux30",
        controls_per_block=4,
    )

    with pytest.raises(ValueError, match="controls_per_block"):
        BallWMDataset(
            tmp_path, history_frames=3, num_preds=1, image_size=(16, 16)
        )


def test_dataset_rejects_mixed_action_protocols(tmp_path: Path):
    _write_synthetic_episode(tmp_path, "ep0000", "scene_a")
    _write_synthetic_episode(
        tmp_path,
        "ep0001",
        "scene_b",
        action_protocol_id="ball_action_v2_aux30",
    )

    with pytest.raises(ValueError, match="mixed action protocols"):
        BallWMDataset(
            tmp_path, history_frames=3, num_preds=1, image_size=(16, 16)
        )


def test_dataset_returns_three_future_frames_for_rollout_horizon(synthetic_root: Path):
    ds = BallWMDataset(
        synthetic_root,
        history_frames=3,
        num_preds=1,
        rollout_horizon=3,
        image_size=(16, 16),
    )
    sample = ds[0]
    assert sample["pixels_left"].shape[0] == 6
    assert sample["geometry_bev"].shape[0] == 6
    assert sample["frame_ids"].tolist() == [0, 1, 2, 3, 4, 5]


def test_action_block_alignment(synthetic_root: Path):
    ds = BallWMDataset(synthetic_root, history_frames=3, num_preds=1, image_size=(16, 16))
    states = np.load(synthetic_root / "ep0000" / "states.npz", allow_pickle=False)
    sample = ds[0]
    np.testing.assert_allclose(
        sample["action"][0].numpy(),
        states["applied_action"][5:10].reshape(-1),
        atol=1e-6,
    )


def test_dataset_derives_future_relative_pose_from_context_last_frame_without_writing_episode(
    synthetic_root: Path,
):
    """Catches labels aligned to the wrong frame or a loader that mutates formal trajectory files."""

    episode_path = synthetic_root / "ep0000"
    with np.load(episode_path / "states.npz", allow_pickle=False) as loaded:
        states = {name: loaded[name].copy() for name in loaded.files}
    aligned_indices = [5 * (frame + 1) - 1 for frame in range(6)]
    states["pos"][:, :] = 0.0
    states["pos"][aligned_indices[2], :2] = [0.0, 0.0]
    states["pos"][aligned_indices[3], :2] = [0.1, 0.0]
    states["pos"][aligned_indices[4], :2] = [0.2, 0.0]
    states["pos"][aligned_indices[5], :2] = [0.3, 0.0]
    np.savez_compressed(episode_path / "states.npz", **states)
    before = (episode_path / "states.npz").read_bytes()

    dataset = BallWMDataset(
        synthetic_root,
        history_frames=3,
        num_preds=1,
        rollout_horizon=3,
        image_size=(16, 16),
    )
    sample = dataset[0]

    assert sample["future_relative_pose"].shape == (3, 4)
    np.testing.assert_allclose(sample["future_relative_pose"][:, 0].numpy(), [0.1, 0.2, 0.3], atol=1e-6)
    np.testing.assert_allclose(sample["future_relative_pose"][:, 1].numpy(), [0.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(sample["future_relative_pose"][:, 2].numpy(), [0.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(sample["future_relative_pose"][:, 3].numpy(), [1.0, 1.0, 1.0], atol=1e-6)
    assert (episode_path / "states.npz").read_bytes() == before


def test_proprio_gravity_identity_quat_is_down(synthetic_root: Path):
    ds = BallWMDataset(synthetic_root, history_frames=3, num_preds=1, image_size=(16, 16))
    sample = ds[0]
    np.testing.assert_allclose(sample["proprio"][0, 6:9].numpy(), [0.0, 0.0, -1.0], atol=1e-5)


def test_scene_split_is_disjoint_and_keeps_unseen_test(synthetic_root: Path):
    ds = BallWMDataset(synthetic_root, history_frames=3, num_preds=1, image_size=(16, 16))
    train, val, test = make_train_val_test_split(ds, ratios=(0.5, 0.0, 0.5), seed=0)
    assert set(train).isdisjoint(test)
    assert set(val).isdisjoint(test)
    assert train and test
    train_scenes = {ds.sample_scenes[i] for i in train}
    test_scenes = {ds.sample_scenes[i] for i in test}
    assert train_scenes.isdisjoint(test_scenes)


def test_dataset_accepts_multiple_roots(tmp_path: Path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    _write_synthetic_episode(first, "ep0000", "scene_a")
    _write_synthetic_episode(second, "ep0001", "scene_b")

    ds = BallWMDataset([first, second], history_frames=3, num_preds=1, image_size=(16, 16))

    assert len(ds.episode_paths) == 2
    assert len(ds) > 0
    assert set(ds.sample_scenes) == {"scene_a", "scene_b"}


def test_dataset_accepts_scene_root_with_nested_episodes(tmp_path: Path):
    scene = tmp_path / "scene_easy_0"
    _write_synthetic_episode(scene, "ep0000", "scene_a")

    ds = BallWMDataset(scene.parent, history_frames=3, num_preds=1, image_size=(16, 16))

    assert len(ds.episode_paths) == 1
    assert len(ds) > 0


def test_episode_discovery_excludes_partial_and_unrelated_directories(tmp_path: Path):
    _write_synthetic_episode(tmp_path / "scene_easy_0", "ep0000", "scene_a")
    _write_synthetic_episode(tmp_path / "scene_easy_0", "ep0001.partial", "scene_a")
    _write_synthetic_episode(tmp_path / "scene_easy_0", "notes", "scene_a")

    assert discover_episode_paths(tmp_path) == [tmp_path / "scene_easy_0" / "ep0000"]


def test_dataset_stats_discovers_nested_scene_episodes(tmp_path: Path):
    _write_synthetic_episode(tmp_path / "scene_easy_0", "ep0000", "scene_a")
    _write_synthetic_episode(tmp_path / "scene_easy_1", "ep0000", "scene_b")

    stats = summarize(tmp_path)

    assert stats["episodes"] == 2
    assert stats["state_samples"] == 60
    assert stats["stereo_frames"] == 12
    assert stats["elevation_frames"] == 0
    assert stats["collision_rate"] == 0.0


def test_dataset_stats_reports_episode_trajectory_metrics(tmp_path: Path):
    episode = _write_synthetic_episode(tmp_path / "scene_easy_0", "ep0000", "scene_a")

    stats = summarize(tmp_path)

    assert len(stats["episodes_detail"]) == 1
    detail = stats["episodes_detail"][0]
    assert detail["path"] == str(episode)
    assert detail["state_samples"] == 30
    assert detail["stereo_frames"] == 6
    assert detail["collision_frames"] == 0
    assert detail["trajectory_length_m"] == 0.0
    assert detail["displacement_m"] == 0.0
    assert detail["elevation_enabled"] is False
    assert detail["elevation_frames"] == 0


def test_dataset_stats_can_write_manifest(tmp_path: Path, capsys):
    _write_synthetic_episode(tmp_path / "scene_easy_0", "ep0000", "scene_a")
    output = tmp_path / "manifest.json"

    assert dataset_stats_main(["--root", str(tmp_path), "--output", str(output)]) == 0

    saved = json.loads(output.read_text())
    assert saved["episodes"] == 1
    assert saved["episodes_detail"][0]["scene"] == "scene_a"
    assert json.loads(capsys.readouterr().out)["episodes"] == 1


def test_empty_dataset_stats_is_strict_json(tmp_path: Path):
    stats = summarize(tmp_path)

    assert stats["applied_action_min"] is None
    assert stats["applied_action_max"] is None
    json.dumps(stats, allow_nan=False)
