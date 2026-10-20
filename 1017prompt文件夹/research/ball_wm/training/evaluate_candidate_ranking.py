"""Evaluate action-conditioned world-model candidate ranking without Isaac."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import torch
from torch.utils.data import DataLoader, Subset

from .action_protocol import require_action_protocol
from .candidate_losses import candidate_goal_distances, candidate_ranking_accuracy
from .candidate_rollout_dataset import CandidateRolloutDataset
from .joint_jepa import JointModelConfig, build_joint_model
from .navigation_metrics import spearman_rank_correlation


def _finite_candidate_vector(value: object, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim != 1 or len(array) < 8 or not np.isfinite(array).all() or (array < 0.0).any():
        raise ValueError(f"{name} must contain at least eight finite non-negative candidates")
    return array


def _collision_candidates(value: object, candidate_count: int) -> np.ndarray:
    collision = np.asarray(value)
    if collision.ndim == 1:
        if collision.shape != (candidate_count,):
            raise ValueError("collision must have the same candidate count")
    elif collision.ndim == 2:
        if collision.shape[0] != candidate_count:
            raise ValueError("collision must have the same candidate count")
        collision = collision.any(axis=1)
    else:
        raise ValueError("collision must have shape (K,) or (K, H)")
    if not np.isin(collision, (0, 1, False, True)).all():
        raise ValueError("collision must be binary")
    return collision.astype(bool, copy=False)


def _valid_pair_count(target: np.ndarray, collision: np.ndarray, margin_m: float) -> int:
    count = 0
    for left in range(len(target)):
        for right in range(left + 1, len(target)):
            if collision[left] != collision[right] or (
                not collision[left] and abs(float(target[left]) - float(target[right])) > float(margin_m)
            ):
                count += 1
    return count


def summarize_candidate_ranking(
    contexts: Sequence[Mapping[str, object]],
    *,
    margin_m: float = 0.05,
) -> dict[str, object]:
    """Aggregate ranking metrics over independently frozen contexts."""

    if not np.isfinite(float(margin_m)) or float(margin_m) < 0.0:
        raise ValueError("margin_m must be finite and non-negative")
    pairwise: list[float] = []
    correlations: list[float] = []
    top1: list[float] = []
    spreads: list[float] = []
    collision_fractions: list[float] = []
    valid_pairs = 0
    for context in contexts:
        if not isinstance(context, Mapping):
            raise ValueError("candidate ranking context must be a mapping")
        predicted = _finite_candidate_vector(
            context.get("predicted_goal_distance_m"), "predicted_goal_distance_m"
        )
        target = _finite_candidate_vector(context.get("target_goal_distance_m"), "target_goal_distance_m")
        if len(predicted) != len(target):
            raise ValueError("predicted and target distances must have the same candidate count")
        collision = _collision_candidates(context.get("collision"), len(target))
        predicted_tensor = torch.from_numpy(predicted.astype(np.float32)).unsqueeze(0)
        target_tensor = torch.from_numpy(target.astype(np.float32)).unsqueeze(0)
        collision_tensor = torch.from_numpy(collision.astype(np.uint8)).unsqueeze(0)
        pairwise.append(
            float(
                candidate_ranking_accuracy(
                    predicted_tensor,
                    target_tensor,
                    collision_tensor,
                    margin_m=float(margin_m),
                ).item()
            )
        )
        correlations.append(spearman_rank_correlation(predicted.tolist(), target.tolist()))
        predicted_winner = int(np.argmin(predicted))
        actual_winner = min(range(len(target)), key=lambda index: (bool(collision[index]), float(target[index])))
        top1.append(float(predicted_winner == actual_winner))
        spreads.append(float(np.std(predicted)))
        collision_fractions.append(float(np.mean(collision)))
        valid_pairs += _valid_pair_count(target, collision, float(margin_m))

    if not contexts:
        raise ValueError("candidate ranking requires at least one context")
    return {
        "num_contexts": len(contexts),
        "usable_contexts": len(contexts),
        "pairwise_accuracy_mean": float(np.mean(pairwise)),
        "spearman_rho_mean": float(np.mean(correlations)),
        "top1_match_rate": float(np.mean(top1)),
        "predicted_goal_distance_std_mean": float(np.mean(spreads)),
        "collision_candidate_fraction": float(np.mean(collision_fractions)),
        "valid_pair_count": int(valid_pairs),
        "ranking_margin_m": float(margin_m),
    }


def evaluate_candidate_ranking(
    *,
    checkpoint_path: str | Path,
    candidate_root: str | Path,
    formal_data_root: str | Path,
    output_path: str | Path | None = None,
    split: str = "test",
    batch_size: int = 8,
    num_workers: int = 0,
    seed: int = 3072,
    device_name: str = "cuda",
    expected_candidates: int | None = None,
    margin_m: float = 0.05,
) -> dict[str, object]:
    """Run candidate ranking evaluation with a frozen world-model checkpoint."""

    if split not in {"train", "val", "test", "all"}:
        raise ValueError("split must be train, val, test, or all")
    if batch_size < 1 or num_workers < 0:
        raise ValueError("batch_size must be positive and num_workers non-negative")
    torch.manual_seed(int(seed))
    device = torch.device(device_name if not device_name.startswith("cuda") or torch.cuda.is_available() else "cpu")
    checkpoint = torch.load(Path(checkpoint_path), map_location=device, weights_only=False)
    if not isinstance(checkpoint, Mapping) or "config" not in checkpoint or "model" not in checkpoint:
        raise ValueError("checkpoint must contain config and model")
    config = JointModelConfig(**checkpoint["config"])
    if not config.pose_head_enabled:
        raise ValueError("candidate ranking evaluation requires an enabled pose head")
    if int(config.rollout_horizon) < 5:
        raise ValueError("candidate ranking evaluation requires rollout_horizon >= 5")
    model = build_joint_model(config).to(device)
    model.load_state_dict(checkpoint["model"])
    model.eval()

    dataset = CandidateRolloutDataset(
        candidate_root,
        formal_data_root,
        image_size=(config.image_size, config.image_size),
        normalize=True,
        expected_horizon=5,
        expected_candidates=expected_candidates,
    )
    require_action_protocol(
        checkpoint, dataset.action_protocol_id, artifact="checkpoint"
    )
    if split == "all":
        indices = list(range(len(dataset)))
    else:
        splits = dataset.scene_splits(seed=int(seed))
        indices = {"train": splits[0], "val": splits[1], "test": splits[2]}[split]
    if not indices:
        raise ValueError(f"candidate ranking split {split!r} has no contexts")
    loader = DataLoader(
        Subset(dataset, indices),
        batch_size=int(batch_size),
        shuffle=False,
        num_workers=int(num_workers),
        pin_memory=device.type == "cuda",
    )
    rows: list[dict[str, object]] = []
    with torch.inference_mode():
        for raw_batch in loader:
            batch = {name: value.to(device, non_blocking=True) for name, value in raw_batch.items()}
            context = {
                "pixels_left": batch["pixels_left"],
                "pixels_right": batch["pixels_right"],
                "proprio": batch["proprio"],
                "action": batch["context_action"],
            }
            predicted_future = model.rollout(
                context,
                batch["candidate_action"],
                history_size=5,
            )
            predicted_pose = model.decode_relative_pose(predicted_future)
            predicted_distance = candidate_goal_distances(
                predicted_pose,
                batch["goal_relative_xy"],
            )
            for index in range(predicted_distance.size(0)):
                rows.append(
                    {
                        "predicted_goal_distance_m": predicted_distance[index].detach().cpu().tolist(),
                        "target_goal_distance_m": batch["goal_distance_m"][index].detach().cpu().tolist(),
                        "collision": batch["collision"][index].detach().cpu().tolist(),
                    }
                )
    summary = summarize_candidate_ranking(rows, margin_m=float(margin_m))
    summary.update(
        {
            "split": split,
            "sample_count": len(rows),
            "scene_count": len({dataset.sample_scenes[index] for index in indices}),
            "checkpoint": str(checkpoint_path),
            "candidate_root": str(candidate_root),
            "formal_data_root": str(formal_data_root),
            "device": str(device),
            "seed": int(seed),
            "action_protocol_id": dataset.action_protocol_id,
            "controls_per_block": dataset.controls_per_block,
        }
    )
    if output_path is not None:
        output = Path(output_path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    return summary


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--candidate_root", type=Path, required=True)
    parser.add_argument("--formal_data_root", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--split", choices=("train", "val", "test", "all"), default="test")
    parser.add_argument("--batch_size", type=int, default=8)
    parser.add_argument("--num_workers", type=int, default=0)
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--expected_candidates", type=int, default=None)
    parser.add_argument("--margin_m", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        summary = evaluate_candidate_ranking(
            checkpoint_path=args.checkpoint,
            candidate_root=args.candidate_root,
            formal_data_root=args.formal_data_root,
            output_path=args.output,
            split=args.split,
            batch_size=args.batch_size,
            num_workers=args.num_workers,
            seed=args.seed,
            device_name=args.device,
            expected_candidates=args.expected_candidates,
            margin_m=args.margin_m,
        )
    except Exception as exc:
        print(f"candidate ranking evaluation failed: {type(exc).__name__}: {exc}")
        return 1
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
