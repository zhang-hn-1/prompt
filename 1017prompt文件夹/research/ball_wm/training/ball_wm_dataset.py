"""PyTorch dataset for the schema-1.1 ball robot collector output.

Each episode directory contains ``states.npz``, ``metadata.json``, paired
``left_*.png``/``right_*.png`` RGB images, and optional BEV/mask PNGs.  The
dataset slices aligned sub-trajectories at 10 Hz stereo cadence, with each
visual transition carrying a block of 5 applied 50 Hz actions (frame skip).
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from PIL import Image
import torch
from .action_protocol import LEGACY_ACTION_PROTOCOL_ID, declared_action_protocol_id
from .dataset_paths import discover_episode_paths
from .pose_utils import relative_planar_pose

IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406])
IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225])


@dataclass(frozen=True)
class SampleConfig:
    history_frames: int = 5
    num_preds: int = 1
    rollout_horizon: int = 1
    image_size: tuple[int, int] = (224, 224)
    action_block: int = 5
    with_bev: bool = True
    with_visibility: bool = True
    normalize: bool = True

    def __post_init__(self) -> None:
        if self.history_frames < 1 or self.num_preds < 1 or self.rollout_horizon < 1:
            raise ValueError("history_frames, num_preds, and rollout_horizon must be positive")
        if self.action_block < 1:
            raise ValueError("action_block must be positive")
        if len(self.image_size) != 2 or any(int(v) <= 0 for v in self.image_size):
            raise ValueError("image_size must be a positive (H, W)")


def _load_image(path: Path, size: tuple[int, int]) -> torch.Tensor:
    image = Image.open(path).convert("RGB").resize(size, Image.LANCZOS)
    tensor = torch.from_numpy(np.asarray(image, dtype=np.float32)).permute(2, 0, 1) / 255.0
    return tensor


def _normalize_image(tensor: torch.Tensor) -> torch.Tensor:
    return (tensor - IMAGENET_MEAN[:, None, None]) / IMAGENET_STD[:, None, None]


def _gravity_in_base(quat: np.ndarray) -> np.ndarray:
    """Return world gravity direction expressed in the base frame."""

    w, x, y, z = quat
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return rotation.T @ np.array([0.0, 0.0, -1.0])


def _proprio17(states: dict[str, np.ndarray], state_index: int) -> np.ndarray:
    quat = states["quat"][state_index].astype(np.float64)
    gravity = _gravity_in_base(quat)
    return np.concatenate(
        [
            states["lin_vel"][state_index].astype(np.float32),
            states["ang_vel"][state_index].astype(np.float32),
            gravity.astype(np.float32),
            states["joint_pos"][state_index].astype(np.float32),
            states["joint_vel"][state_index].astype(np.float32),
            states["applied_action"][state_index].astype(np.float32),
        ],
        axis=0,
    )


@lru_cache(maxsize=32)
def _load_episode(path: str) -> dict[str, Any]:
    episode = Path(path)
    with np.load(episode / "states.npz", allow_pickle=False) as loaded:
        states = {key: loaded[key] for key in loaded.files}
    metadata = json.loads((episode / "metadata.json").read_text())
    num_frames = len(states["stereo_frame_id"])
    return {"states": states, "metadata": metadata, "num_frames": num_frames, "path": episode}


class BallWMDataset(torch.utils.data.Dataset):
    """Aligned stereo/state/action/BEV sub-trajectories from collector episodes."""

    def __init__(
        self,
        root: str | Path | Sequence[str | Path],
        history_frames: int = 5,
        num_preds: int = 1,
        rollout_horizon: int = 1,
        image_size: tuple[int, int] = (224, 224),
        action_block: int = 5,
        with_bev: bool = True,
        with_visibility: bool = True,
        normalize: bool = True,
    ) -> None:
        roots = [root] if isinstance(root, (str, Path)) else list(root)
        self.roots = [Path(path) for path in roots]
        self.config = SampleConfig(
            history_frames=history_frames,
            num_preds=num_preds,
            rollout_horizon=rollout_horizon,
            image_size=image_size,
            action_block=action_block,
            with_bev=with_bev,
            with_visibility=with_visibility,
            normalize=normalize,
        )
        self.episode_paths = discover_episode_paths(self.roots)
        if not self.episode_paths:
            raise FileNotFoundError(f"no schema-1.1/1.2 episodes found under {self.roots}")

        episode_metadata = [
            _load_episode(str(episode))["metadata"] for episode in self.episode_paths
        ]
        protocol_ids = {
            declared_action_protocol_id(metadata) for metadata in episode_metadata
        }
        if len(protocol_ids) != 1:
            raise ValueError(
                f"dataset contains mixed action protocols: {sorted(protocol_ids)}"
            )
        self.action_protocol_id = next(iter(protocol_ids))
        controls_per_block_values: set[int] = set()
        for metadata in episode_metadata:
            raw_controls = metadata.get("controls_per_block")
            if raw_controls is None:
                if self.action_protocol_id != LEGACY_ACTION_PROTOCOL_ID:
                    raise ValueError(
                        "controls_per_block is required for labelled action protocols"
                    )
                raw_controls = self.config.action_block
            if isinstance(raw_controls, bool) or not isinstance(raw_controls, int):
                raise ValueError("controls_per_block must be a positive integer")
            controls_per_block_values.add(raw_controls)
        if len(controls_per_block_values) != 1:
            raise ValueError(
                "dataset contains mixed controls_per_block values: "
                f"{sorted(controls_per_block_values)}"
            )
        self.controls_per_block = next(iter(controls_per_block_values))
        if self.controls_per_block != self.config.action_block:
            raise ValueError(
                "dataset controls_per_block does not match requested action_block: "
                f"{self.controls_per_block} != {self.config.action_block}"
            )

        self.samples: list[tuple[int, int]] = []
        self.sample_scenes: list[str] = []
        self.sample_episodes: list[int] = []
        for episode_index, episode in enumerate(self.episode_paths):
            data = _load_episode(str(episode))
            num_frames = data["num_frames"]
            history = self.config.history_frames
            preds = self.config.num_preds
            horizon = self.config.rollout_horizon
            max_start = num_frames - history - max(preds, horizon)
            if max_start < 0:
                continue
            scene = data["metadata"].get("scene", str(episode))
            for start in range(max_start + 1):
                self.samples.append((episode_index, start))
                self.sample_scenes.append(scene)
                self.sample_episodes.append(episode_index)

    def __len__(self) -> int:
        return len(self.samples)

    def _frame_arrays(self, episode_index: int, start: int) -> dict[str, torch.Tensor]:
        episode = self.episode_paths[episode_index]
        data = _load_episode(str(episode))
        states = data["states"]
        config = self.config
        history = config.history_frames
        preds = config.num_preds
        total = history + max(preds, config.rollout_horizon)

        left_images = torch.stack(
            [
                _normalize_image(_load_image(episode / f"left_{f:06d}.png", config.image_size))
                if config.normalize
                else _load_image(episode / f"left_{f:06d}.png", config.image_size)
                for f in range(start, start + total)
            ]
        )
        right_images = torch.stack(
            [
                _normalize_image(_load_image(episode / f"right_{f:06d}.png", config.image_size))
                if config.normalize
                else _load_image(episode / f"right_{f:06d}.png", config.image_size)
                for f in range(start, start + total)
            ]
        )

        action_rows = []
        proprio_rows = []
        for frame in range(start, start + total):
            next_start = config.action_block * (frame + 1)
            next_end = next_start + config.action_block
            if next_end <= len(states["applied_action"]):
                action_rows.append(states["applied_action"][next_start:next_end].reshape(-1))
            else:
                action_rows.append(np.zeros(config.action_block * 2, dtype=np.float32))
            state_index = config.action_block * (frame + 1) - 1
            proprio_rows.append(_proprio17(states, state_index))

        result = {
            "pixels_left": left_images,
            "pixels_right": right_images,
            "action": torch.from_numpy(np.stack(action_rows).astype(np.float32)),
            "proprio": torch.from_numpy(np.stack(proprio_rows).astype(np.float32)),
        }

        if config.with_bev:
            bev = torch.stack(
                [
                    torch.from_numpy(
                        np.asarray(
                            Image.open(episode / f"geometry_bev_{f:06d}.png").convert("L"),
                            dtype=np.int64,
                        )
                    )
                    for f in range(start, start + total)
                ]
            )
            result["geometry_bev"] = bev
        if config.with_visibility:
            for name in ("current_visible", "history_observed"):
                masks = torch.stack(
                    [
                        torch.from_numpy(
                            np.asarray(
                                Image.open(episode / f"{name}_{f:06d}.png").convert("L"),
                                dtype=np.float32,
                            )
                        )
                        for f in range(start, start + total)
                    ]
                )
                result[name] = masks

        result["collision"] = torch.from_numpy(
            states["collision"][start : start + total].astype(np.float32)
        )
        result["clearance"] = torch.from_numpy(
            states["minimum_obstacle_distance"][start : start + total].astype(np.float32)
        )
        result["frame_ids"] = torch.from_numpy(
            states["stereo_frame_id"][start : start + total].astype(np.int64)
        )
        context_last_frame = start + history - 1
        source_state_index = config.action_block * (context_last_frame + 1) - 1
        if source_state_index >= len(states["pos"]) or source_state_index >= len(states["quat"]):
            raise ValueError("context frame does not have an aligned pose state")
        relative_labels: list[np.ndarray] = []
        for horizon_step in range(1, config.rollout_horizon + 1):
            target_frame = context_last_frame + horizon_step
            target_state_index = config.action_block * (target_frame + 1) - 1
            if target_state_index >= len(states["pos"]) or target_state_index >= len(states["quat"]):
                raise ValueError("future frame does not have an aligned pose state")
            relative_labels.append(
                relative_planar_pose(
                    states["pos"][source_state_index],
                    states["quat"][source_state_index],
                    states["pos"][target_state_index],
                    states["quat"][target_state_index],
                )
            )
        result["future_relative_pose"] = torch.from_numpy(np.stack(relative_labels).astype(np.float32))
        return result

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        episode_index, start = self.samples[index]
        return self._frame_arrays(episode_index, start)

    def scene_splits(
        self,
        ratios: tuple[float, float, float] = (0.70, 0.15, 0.15),
        seed: int = 0,
        unseen_scene_test: bool = True,
    ) -> tuple[list[int], list[int], list[int]]:
        """Deterministic train/validation/test sample indices."""

        scenes: dict[str, list[int]] = {}
        for index, scene in enumerate(self.sample_scenes):
            scenes.setdefault(scene, []).append(index)
        scene_names = sorted(scenes)
        rng = np.random.default_rng(seed)
        rng.shuffle(scene_names)

        if len(scene_names) == 1:
            indices = np.arange(len(self.samples))
            rng.shuffle(indices)
            train_count = int(round(ratios[0] * len(indices)))
            val_count = int(round(ratios[1] * len(indices)))
            return (
                sorted(indices[:train_count].tolist()),
                sorted(indices[train_count : train_count + val_count].tolist()),
                sorted(indices[train_count + val_count :].tolist()),
            )

        total = len(scene_names)
        train_count = max(1, int(round(ratios[0] * total)))
        val_count = max(0, int(round(ratios[1] * total)))
        train_scenes = set(scene_names[:train_count])
        val_scenes = set(scene_names[train_count : train_count + val_count])
        test_scenes = set(scene_names[train_count + val_count :])
        if unseen_scene_test and not test_scenes:
            last_scene = scene_names[-1]
            test_scenes = {last_scene}
            train_scenes.discard(last_scene)
            val_scenes.discard(scene_names[-1])

        train = [i for scene in train_scenes for i in scenes[scene]]
        val = [i for scene in val_scenes for i in scenes[scene]]
        test = [i for scene in test_scenes for i in scenes[scene]]
        return sorted(train), sorted(val), sorted(test)


def make_train_val_test_split(
    dataset: BallWMDataset,
    ratios: tuple[float, float, float] = (0.70, 0.15, 0.15),
    seed: int = 0,
    unseen_scene_test: bool = True,
) -> tuple[list[int], list[int], list[int]]:
    return dataset.scene_splits(ratios, seed, unseen_scene_test)
