"""Isaac-free dataset for multi-candidate action consequences."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Sequence

import torch

from .ball_wm_dataset import BallWMDataset
from .candidate_rollout_schema import CandidateContextRef, load_candidate_context


class CandidateRolloutDataset(torch.utils.data.Dataset):
    """Context histories paired with K actions and simulator-measured outcomes."""

    def __init__(
        self,
        candidate_root: str | Path,
        formal_data_root: str | Path,
        image_size: tuple[int, int] = (224, 224),
        normalize: bool = True,
        expected_horizon: int = 5,
        expected_candidates: int | None = None,
        proprio_velocity_frame: str = "world",
    ) -> None:
        if int(expected_horizon) != 5:
            raise ValueError("candidate protocol requires expected_horizon=5")
        if expected_candidates is not None and int(expected_candidates) < 8:
            raise ValueError("expected_candidates must be at least 8")
        self.candidate_root = Path(candidate_root)
        self.formal_data_root = Path(formal_data_root)
        self.expected_candidates = None if expected_candidates is None else int(expected_candidates)
        self._formal = BallWMDataset(
            self.formal_data_root,
            history_frames=5,
            num_preds=1,
            rollout_horizon=5,
            image_size=image_size,
            action_block=5,
            with_bev=False,
            with_visibility=False,
            normalize=normalize,
            proprio_velocity_frame=proprio_velocity_frame,
        )
        self.context_paths = self._discover_contexts()
        if not self.context_paths:
            raise FileNotFoundError(f"no finalized candidate contexts under {self.candidate_root}")
        self.sample_scenes: list[str] = []
        self._refs: list[CandidateContextRef] = []
        for context_path in self.context_paths:
            metadata, arrays = load_candidate_context(context_path, self.formal_data_root)
            ref = CandidateContextRef.from_mapping(metadata)
            if ref.action_protocol_id != self._formal.action_protocol_id:
                raise ValueError(
                    "candidate action protocol does not match formal data: "
                    f"{ref.action_protocol_id} != {self._formal.action_protocol_id}"
                )
            if ref.controls_per_block != self._formal.controls_per_block:
                raise ValueError(
                    "candidate controls_per_block does not match formal data"
                )
            if self.expected_candidates is not None and arrays["action"].shape[0] != self.expected_candidates:
                raise ValueError("candidate context has an unexpected candidate count")
            source_episode = (self.formal_data_root.resolve() / ref.source_episode).resolve()
            if source_episode not in {path.resolve() for path in self._formal.episode_paths}:
                raise ValueError(f"source episode is not discoverable by formal dataset: {source_episode}")
            self._refs.append(ref)
            self.sample_scenes.append(ref.scene)
        self.action_protocol_id = self._formal.action_protocol_id
        self.controls_per_block = self._formal.controls_per_block

    def _discover_contexts(self) -> list[Path]:
        if not self.candidate_root.is_dir():
            return []
        contexts: list[Path] = []
        for metadata_path in self.candidate_root.rglob("context.json"):
            context_path = metadata_path.parent
            if context_path.name.endswith(".partial"):
                continue
            if re.fullmatch(r"context_\d+", context_path.name) and (context_path / "candidates.npz").is_file():
                contexts.append(context_path)
        return sorted(contexts)

    def __len__(self) -> int:
        return len(self.context_paths)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        context_path = self.context_paths[index]
        metadata, arrays = load_candidate_context(context_path, self.formal_data_root)
        ref = self._refs[index]
        source_episode = (self.formal_data_root.resolve() / ref.source_episode).resolve()
        try:
            episode_index = next(
                episode_index
                for episode_index, path in enumerate(self._formal.episode_paths)
                if path.resolve() == source_episode
            )
        except StopIteration as exc:
            raise ValueError(f"source episode disappeared: {source_episode}") from exc
        episode_data = self._formal._frame_arrays(episode_index, ref.context_start)
        history = ref.history_frames
        return {
            "pixels_left": episode_data["pixels_left"][:history],
            "pixels_right": episode_data["pixels_right"][:history],
            "proprio": episode_data["proprio"][:history],
            "context_action": episode_data["action"][:history],
            "candidate_action": torch.from_numpy(arrays["action"]),
            "goal_relative_xy": torch.tensor(ref.goal_relative_xy, dtype=torch.float32),
            "future_relative_pose": torch.from_numpy(arrays["future_relative_pose"]),
            "goal_distance_m": torch.from_numpy(arrays["goal_distance_m"]),
            "collision": torch.from_numpy(arrays["collision"]),
            "clearance": torch.from_numpy(arrays["clearance_m"]),
        }

    def scene_splits(
        self,
        ratios: tuple[float, float, float] = (0.70, 0.15, 0.15),
        seed: int = 0,
    ) -> tuple[list[int], list[int], list[int]]:
        """Return scene-disjoint context indices."""

        if len(ratios) != 3 or any(float(value) < 0.0 for value in ratios):
            raise ValueError("ratios must contain three non-negative values")
        if abs(sum(float(value) for value in ratios) - 1.0) > 1e-6:
            raise ValueError("ratios must sum to one")
        scenes: dict[str, list[int]] = {}
        for index, scene in enumerate(self.sample_scenes):
            scenes.setdefault(scene, []).append(index)
        scene_names = sorted(scenes)
        generator = torch.Generator().manual_seed(int(seed))
        permutation = torch.randperm(len(scene_names), generator=generator).tolist()
        shuffled = [scene_names[index] for index in permutation]
        if len(shuffled) <= 1:
            indices = list(range(len(self)))
            split = int(round(float(ratios[0]) * len(indices)))
            val_split = split + int(round(float(ratios[1]) * len(indices)))
            return indices[:split], indices[split:val_split], indices[val_split:]
        train_count = max(1, int(round(float(ratios[0]) * len(shuffled))))
        val_count = int(round(float(ratios[1]) * len(shuffled)))
        train_scenes = set(shuffled[:train_count])
        val_scenes = set(shuffled[train_count : train_count + val_count])
        test_scenes = set(shuffled[train_count + val_count :])
        if not test_scenes:
            last = shuffled[-1]
            test_scenes = {last}
            train_scenes.discard(last)
            val_scenes.discard(last)
        return (
            sorted(index for scene in train_scenes for index in scenes[scene]),
            sorted(index for scene in val_scenes for index in scenes[scene]),
            sorted(index for scene in test_scenes for index in scenes[scene]),
        )
