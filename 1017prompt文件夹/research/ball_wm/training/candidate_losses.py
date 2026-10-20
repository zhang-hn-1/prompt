"""Pure PyTorch objectives for action-conditioned candidate rollouts."""

from __future__ import annotations

from typing import Iterable

import torch
from torch.nn import functional as F


def _validate_distance(value: torch.Tensor, name: str) -> torch.Tensor:
    if not torch.is_tensor(value) or value.ndim != 2 or value.size(1) < 2:
        raise ValueError(f"{name} must have shape (B, K) with K >= 2")
    if not torch.isfinite(value).all() or (value < 0.0).any():
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def candidate_goal_distances(
    predicted_relative_pose: torch.Tensor,
    goal_relative_xy: torch.Tensor,
) -> torch.Tensor:
    """Compute predicted final goal distance for each candidate."""

    if (
        not torch.is_tensor(predicted_relative_pose)
        or predicted_relative_pose.ndim != 4
        or predicted_relative_pose.size(-1) != 4
    ):
        raise ValueError("predicted_relative_pose must have shape (B, K, H, 4)")
    if not torch.is_tensor(goal_relative_xy) or goal_relative_xy.ndim != 2:
        raise ValueError("goal_relative_xy must have shape (B, 2)")
    if predicted_relative_pose.size(0) != goal_relative_xy.size(0) or goal_relative_xy.size(1) != 2:
        raise ValueError("predicted_relative_pose and goal_relative_xy batch shapes must match")
    if not torch.isfinite(predicted_relative_pose).all() or not torch.isfinite(goal_relative_xy).all():
        raise ValueError("predicted_relative_pose and goal_relative_xy must be finite")
    return torch.linalg.vector_norm(
        predicted_relative_pose[..., -1, :2] - goal_relative_xy[:, None, :], dim=-1
    )


def candidate_pose_loss(
    predicted_relative_pose: torch.Tensor,
    target_relative_pose: torch.Tensor,
    horizons: Iterable[int] = (1, 3, 5),
) -> torch.Tensor:
    """Calculate Huber XY plus yaw sin/cos MSE over selected horizons."""

    if (
        not torch.is_tensor(predicted_relative_pose)
        or not torch.is_tensor(target_relative_pose)
        or predicted_relative_pose.ndim != 4
        or target_relative_pose.shape != predicted_relative_pose.shape
        or predicted_relative_pose.size(-1) != 4
    ):
        raise ValueError("predicted and target pose tensors must have matching shape (B, K, H, 4)")
    if not torch.isfinite(predicted_relative_pose).all() or not torch.isfinite(target_relative_pose).all():
        raise ValueError("predicted and target pose tensors must be finite")
    selected = tuple(int(horizon) for horizon in horizons)
    if not selected or len(set(selected)) != len(selected) or any(
        horizon < 1 or horizon > predicted_relative_pose.size(2) for horizon in selected
    ):
        raise ValueError("horizons must be non-empty, unique, and one-indexed within H")
    indices = torch.as_tensor([horizon - 1 for horizon in selected], device=predicted_relative_pose.device)
    predicted = predicted_relative_pose.index_select(2, indices)
    target = target_relative_pose.index_select(2, indices)
    return F.huber_loss(predicted[..., :2], target[..., :2]) + F.mse_loss(
        predicted[..., 2:4], target[..., 2:4]
    )


def _collision_mask(collision: torch.Tensor | None, shape: tuple[int, int]) -> torch.Tensor:
    if collision is None:
        return torch.zeros(shape, dtype=torch.bool)
    if not torch.is_tensor(collision) or collision.ndim not in (2, 3):
        raise ValueError("collision must have shape (B, K) or (B, K, H)")
    if tuple(collision.shape[:2]) != shape:
        raise ValueError("collision batch/candidate shape must match distances")
    collision_values = collision.float()
    if not torch.isfinite(collision_values).all() or not torch.isin(
        collision_values, collision_values.new_tensor((0.0, 1.0))
    ).all():
        raise ValueError("collision must be binary and finite")
    return collision.bool().any(dim=-1) if collision.ndim == 3 else collision.bool()


def _rank_pairs(
    predicted_distance: torch.Tensor,
    target_distance: torch.Tensor,
    collision: torch.Tensor | None,
    margin_m: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    _validate_distance(predicted_distance, "predicted_distance")
    _validate_distance(target_distance, "target_distance")
    if predicted_distance.shape != target_distance.shape:
        raise ValueError("predicted_distance and target_distance must have the same shape")
    if not torch.isfinite(torch.as_tensor(margin_m)) or margin_m < 0.0:
        raise ValueError("margin_m must be finite and non-negative")
    batch, candidates = predicted_distance.shape
    upper = torch.triu(
        torch.ones((candidates, candidates), dtype=torch.bool, device=predicted_distance.device), diagonal=1
    ).unsqueeze(0)
    collision_mask = _collision_mask(collision, (batch, candidates))
    collision_i = collision_mask[:, :, None]
    collision_j = collision_mask[:, None, :]
    collision_i_better = (~collision_i) & collision_j
    collision_j_better = collision_i & (~collision_j)
    target_i_better = target_distance[:, :, None] + float(margin_m) < target_distance[:, None, :]
    target_j_better = target_distance[:, None, :] + float(margin_m) < target_distance[:, :, None]
    i_better = collision_i_better | ((~collision_i_better) & (~collision_j_better) & target_i_better)
    j_better = collision_j_better | ((~collision_i_better) & (~collision_j_better) & target_j_better)
    valid = upper & (i_better | j_better)
    good_i = i_better & ~j_better
    predicted_i = predicted_distance[:, :, None].expand(-1, -1, candidates)
    predicted_j = predicted_distance[:, None, :].expand(-1, candidates, -1)
    good = torch.where(good_i, predicted_i, predicted_j)
    bad = torch.where(good_i, predicted_j, predicted_i)
    return good[valid], bad[valid], valid


def pairwise_candidate_ranking_loss(
    predicted_distance: torch.Tensor,
    target_distance: torch.Tensor,
    collision: torch.Tensor | None = None,
    margin_m: float = 0.05,
) -> torch.Tensor:
    """Penalize predicted candidate distances that reverse valid target pairs."""

    good, bad, valid = _rank_pairs(predicted_distance, target_distance, collision, margin_m)
    if not valid.any():
        return predicted_distance.sum() * 0.0
    return F.relu(float(margin_m) - (bad - good)).mean()


def candidate_ranking_accuracy(
    predicted_distance: torch.Tensor,
    target_distance: torch.Tensor,
    collision: torch.Tensor | None = None,
    margin_m: float = 0.05,
) -> torch.Tensor:
    """Return the fraction of valid pairs whose predicted margin has the right sign."""

    good, bad, valid = _rank_pairs(predicted_distance, target_distance, collision, margin_m)
    if not valid.any():
        return predicted_distance.new_zeros(())
    return (bad - good >= float(margin_m)).to(dtype=predicted_distance.dtype).mean()
