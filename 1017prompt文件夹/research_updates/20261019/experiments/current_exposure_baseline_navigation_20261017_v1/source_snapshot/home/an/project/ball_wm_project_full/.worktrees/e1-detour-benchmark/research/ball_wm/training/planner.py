"""Cross-Entropy Method (CEM) planner in the joint world-model latent space."""

from __future__ import annotations

from dataclasses import dataclass
import math

import torch
import torch.nn.functional as F

from .action_protocol import ActionProtocol


@dataclass
class PlanResult:
    actions: torch.Tensor  # (T, action_dim)
    cost: float
    elite_costs: torch.Tensor
    goal_cost: float
    safety_cost: float
    smoothness_cost: float = 0.0
    fallback_used: bool = False
    predicted_relative_trajectory: torch.Tensor | None = None


@dataclass
class CandidateScores:
    """All CEM candidate costs plus the pose rollout used by pose-aware planning."""

    total_costs: torch.Tensor
    goal_costs: torch.Tensor
    safety_costs: torch.Tensor
    smoothness_costs: torch.Tensor
    predicted_relative_pose: torch.Tensor | None


def action_bounds(protocol: ActionProtocol) -> torch.Tensor:
    """Return per-component (min, max) rows for a 10-D action block."""

    if not isinstance(protocol, ActionProtocol):
        raise TypeError("protocol must be an ActionProtocol")
    return protocol.block_bounds_torch()


def _clamp_actions(actions: torch.Tensor, bounds: torch.Tensor) -> torch.Tensor:
    return torch.clamp(actions, bounds[:, 0], bounds[:, 1])


def _primitive_action_anchors(horizon: int, bounds: torch.Tensor) -> torch.Tensor:
    """Return deterministic persistent motion primitives for every CEM population.

    The action protocol repeats five ``(main, auxiliary)`` controls in each
    model block.  Gaussian samples cover local perturbations well, but can
    miss the structured, persistent commands that are present in the rollout
    ranking data.  Keeping the same small primitive set in every population
    makes CEM robust to that coverage gap while retaining random exploration.
    """

    if isinstance(horizon, bool) or int(horizon) < 1:
        raise ValueError("horizon must be a positive integer")
    if bounds.shape != (10, 2):
        raise ValueError("bounds must have shape (10, 2)")
    anchors = torch.zeros(8, int(horizon), 10, device=bounds.device, dtype=bounds.dtype)
    main_forward = 0.70 * bounds[0, 1]
    main_reverse = 0.70 * bounds[0, 0]
    aux_left = 0.70 * bounds[1, 1]
    aux_right = 0.70 * bounds[1, 0]

    anchors[1, ..., 0::2] = main_forward
    anchors[2, ..., 0::2] = main_reverse
    anchors[3, ..., 1::2] = aux_left
    anchors[4, ..., 1::2] = aux_right
    anchors[5, ..., 0::2] = main_forward
    anchors[5, ..., 1::2] = aux_left
    anchors[6, ..., 0::2] = main_forward
    anchors[6, ..., 1::2] = aux_right

    split = min(3, int(horizon))
    anchors[7, :split, 0::2] = main_forward
    anchors[7, split:, 0::2] = main_reverse
    anchors[7, :split, 1::2] = aux_left
    anchors[7, split:, 1::2] = aux_right
    return _clamp_actions(anchors, bounds)


def latent_goal_cost(predicted: torch.Tensor, goal_emb: torch.Tensor) -> torch.Tensor:
    """MSE between final predicted embeddings and the goal embedding."""

    last = predicted[..., -1:, :]
    target = goal_emb[..., -1:, :].detach()
    if target.dim() < last.dim():
        target = target.unsqueeze(-2)
    target = target.expand_as(last)
    return F.mse_loss(last, target, reduction="none").sum(dim=-1).mean(dim=-1)  # (B,S)


def relative_pose_goal_cost(
    predicted_relative_pose: torch.Tensor,
    goal_relative_xy: torch.Tensor,
) -> torch.Tensor:
    """Return final base-frame PointGoal distance for every candidate rollout."""

    if predicted_relative_pose.ndim != 4 or predicted_relative_pose.size(-1) != 4:
        raise ValueError("predicted_relative_pose must have shape (B, S, T, 4)")
    if goal_relative_xy.ndim != 2 or goal_relative_xy.shape != (
        predicted_relative_pose.size(0),
        2,
    ):
        raise ValueError("goal_relative_xy must have shape (B, 2)")
    if not torch.isfinite(predicted_relative_pose).all() or not torch.isfinite(goal_relative_xy).all():
        raise ValueError("pose predictions and goals must be finite")
    endpoint_xy = predicted_relative_pose[..., -1, :2]
    return torch.linalg.vector_norm(endpoint_xy - goal_relative_xy[:, None, :], dim=-1)


def action_smoothness_cost(actions: torch.Tensor) -> torch.Tensor:
    """Return a per-candidate action-change penalty for sequences shaped (S, T, A)."""

    if actions.ndim != 3 or actions.size(1) < 1:
        raise ValueError("actions must have shape (S, T, A) with positive horizon")
    if not torch.isfinite(actions).all():
        raise ValueError("actions must be finite")
    if actions.size(1) == 1:
        return actions.new_zeros(actions.size(0))
    return actions.diff(dim=1).pow(2).mean(dim=(1, 2))


def local_bev_trajectory_safety_cost(
    current_bev_logits: torch.Tensor,
    predicted_relative_pose: torch.Tensor,
    resolution_m: float,
    robot_radius_m: float,
    unknown_weight: float,
) -> torch.Tensor:
    """Score candidate base-frame trajectories against the current local BEV risk raster."""

    if current_bev_logits.ndim != 4 or current_bev_logits.size(1) < 3:
        raise ValueError("current_bev_logits must have shape (B, at_least_3, H, W)")
    if predicted_relative_pose.ndim != 4 or predicted_relative_pose.size(-1) != 4:
        raise ValueError("predicted_relative_pose must have shape (B, S, T, 4)")
    if current_bev_logits.size(0) != predicted_relative_pose.size(0):
        raise ValueError("BEV logits and pose rollouts must have the same batch size")
    if (
        not torch.isfinite(current_bev_logits).all()
        or not torch.isfinite(predicted_relative_pose).all()
        or not torch.isfinite(torch.as_tensor([resolution_m, robot_radius_m, unknown_weight])).all()
        or resolution_m <= 0.0
        or robot_radius_m < 0.0
        or unknown_weight <= 0.0
    ):
        raise ValueError("BEV inputs must be finite with positive resolution/unknown weight and non-negative radius")

    batch, _, height, width = current_bev_logits.shape
    _, samples, horizon, _ = predicted_relative_pose.shape
    probability = F.softmax(current_bev_logits, dim=1)
    risk = (probability[:, 1] + float(unknown_weight) * probability[:, 2]).clamp(0.0, 1.0)

    radius_cells = int(torch.ceil(torch.as_tensor(float(robot_radius_m) / float(resolution_m))).item())
    offsets = torch.arange(-radius_cells, radius_cells + 1, device=risk.device, dtype=risk.dtype)
    offset_x, offset_y = torch.meshgrid(offsets, offsets, indexing="ij")
    keep = offset_x.square() + offset_y.square() <= (float(robot_radius_m) / float(resolution_m)) ** 2 + 1e-8
    footprint = torch.stack((offset_x[keep], offset_y[keep]), dim=-1) * float(resolution_m)

    xy = predicted_relative_pose[..., :2].unsqueeze(-2) + footprint.view(1, 1, 1, -1, 2)
    rows = float(height // 2) - xy[..., 0] / float(resolution_m)
    cols = float(width // 2) - xy[..., 1] / float(resolution_m)
    in_bounds = (rows >= 0.0) & (rows <= float(height - 1)) & (cols >= 0.0) & (cols <= float(width - 1))

    if width == 1:
        col_grid = torch.zeros_like(cols)
    else:
        col_grid = 2.0 * cols / float(width - 1) - 1.0
    if height == 1:
        row_grid = torch.zeros_like(rows)
    else:
        row_grid = 2.0 * rows / float(height - 1) - 1.0
    grid = torch.stack((col_grid, row_grid), dim=-1).reshape(batch, samples * horizon * footprint.size(0), 1, 2)
    sampled = F.grid_sample(
        risk.unsqueeze(1),
        grid,
        mode="bilinear",
        padding_mode="zeros",
        align_corners=True,
    ).reshape(batch, samples, horizon, footprint.size(0))
    sampled = torch.where(in_bounds, sampled, torch.ones_like(sampled))
    return sampled.mean(dim=(-1, -2))


def bev_safety_cost(
    model: torch.nn.Module,
    predicted: torch.Tensor,
    radius_cells: int = 5,
) -> torch.Tensor:
    """Mean occupied probability inside the robot footprint over rollouts."""

    logits = model.bev_head(predicted)  # (B,S,T,3,H,W)
    prob = F.softmax(logits, dim=3)[..., 1, :, :]  # (B,S,T,H,W)
    center = prob.size(-1) // 2
    region = prob[
        ...,
        center - radius_cells : center + radius_cells + 1,
        center - radius_cells : center + radius_cells + 1,
    ]
    return region.mean(dim=(-3, -2, -1))  # (B,S)


def score_action_sequences(
    model: torch.nn.Module,
    context: dict[str, torch.Tensor],
    action_sequences: torch.Tensor,
    *,
    planner_mode: str,
    goal_emb: torch.Tensor | None,
    goal_relative_xy: torch.Tensor | None,
    safety_weight: float,
    smoothness_weight: float,
    history_size: int,
    bev_resolution_m: float,
    robot_radius_m: float,
    unknown_weight: float,
) -> CandidateScores:
    """Score candidate action sequences under either legacy or odometry PointGoal semantics."""

    if planner_mode not in {"latent", "pose_bev"}:
        raise ValueError("planner_mode must be 'latent' or 'pose_bev'")
    if action_sequences.ndim != 4 or action_sequences.size(-1) != 10:
        raise ValueError("action_sequences must have shape (B, S, T, 10)")
    if action_sequences.size(0) != context["action"].size(0):
        raise ValueError("action sequences and context must have matching batch size")
    if safety_weight < 0.0 or smoothness_weight < 0.0:
        raise ValueError("safety_weight and smoothness_weight must be non-negative")
    predicted = model.rollout(context, action_sequences, history_size=history_size)
    batch, samples = predicted.shape[:2]
    predicted_relative_pose: torch.Tensor | None = None

    if planner_mode == "latent":
        if goal_emb is None or goal_relative_xy is not None:
            raise ValueError("latent planner mode requires goal_emb and no goal_relative_xy")
        goal_costs = latent_goal_cost(predicted, goal_emb)
        safety_costs = (
            bev_safety_cost(model, predicted)
            if safety_weight > 0.0
            else torch.zeros_like(goal_costs)
        )
    else:
        if goal_relative_xy is None or goal_emb is not None:
            raise ValueError("pose_bev planner mode requires goal_relative_xy and no goal_emb")
        if getattr(model, "pose_head", None) is None:
            raise ValueError("pose_bev planner mode requires an enabled pose head")
        predicted_relative_pose = model.decode_relative_pose(predicted)
        goal_costs = relative_pose_goal_cost(predicted_relative_pose, goal_relative_xy)
        if safety_weight > 0.0:
            encoded = model.encode({key: value for key, value in context.items() if torch.is_tensor(value)})
            current_emb = encoded["emb"][:, -1]
            current_bev_logits = model.bev_head(current_emb)
            safety_costs = local_bev_trajectory_safety_cost(
                current_bev_logits,
                predicted_relative_pose,
                resolution_m=bev_resolution_m,
                robot_radius_m=robot_radius_m,
                unknown_weight=unknown_weight,
            )
        else:
            safety_costs = torch.zeros_like(goal_costs)

    smoothness_costs = action_smoothness_cost(action_sequences.reshape(batch * samples, *action_sequences.shape[2:]))
    smoothness_costs = smoothness_costs.reshape(batch, samples)
    total_costs = goal_costs + float(safety_weight) * safety_costs + float(smoothness_weight) * smoothness_costs
    return CandidateScores(
        total_costs=total_costs,
        goal_costs=goal_costs,
        safety_costs=safety_costs,
        smoothness_costs=smoothness_costs,
        predicted_relative_pose=predicted_relative_pose,
    )


def plan(
    model: torch.nn.Module,
    context: dict[str, torch.Tensor],
    *,
    action_protocol: ActionProtocol,
    goal_emb: torch.Tensor | None = None,
    horizon: int = 5,
    samples: int = 300,
    elites: int = 30,
    iterations: int = 10,
    safety_weight: float = 0.1,
    history_size: int = 5,
    sigma: float = 1.0,
    device: torch.device | None = None,
    planner_mode: str = "latent",
    goal_relative_xy: torch.Tensor | None = None,
    smoothness_weight: float = 0.0,
    safety_threshold: float = float("inf"),
    bev_resolution_m: float = 0.1,
    robot_radius_m: float = 0.4,
    unknown_weight: float = 1.0,
) -> PlanResult:
    """CEM optimization over future 10-D action blocks with explicit safe-stop fallback."""

    device = device or next(model.parameters()).device
    context = {key: value.to(device) for key, value in context.items()}
    if math.isnan(float(safety_threshold)) or float(safety_threshold) < 0.0:
        raise ValueError("safety_threshold must be non-negative or positive infinity")
    if goal_emb is not None:
        goal_emb = goal_emb.to(device)
    if goal_relative_xy is not None:
        goal_relative_xy = goal_relative_xy.to(device)
    bounds = action_bounds(action_protocol).to(device)
    action_dim = action_protocol.action_block_dim
    mean = torch.zeros(horizon, action_dim, device=device)
    scale = torch.full((horizon, action_dim), sigma, device=device)

    best_actions = None
    best_cost = float("inf")
    best_elite_costs = None
    best_goal_cost = 0.0
    best_safety_cost = 0.0
    best_smoothness_cost = 0.0
    best_relative_trajectory = None
    fallback_used = False
    for _ in range(iterations):
        actions = _clamp_actions(
            mean + scale * torch.randn(samples, horizon, action_dim, device=device),
            bounds,
        )
        # Keep a neutral baseline in every CEM population.  Without this
        # anchor, a high-variance population can return a sampled action whose
        # predicted cost is worse than simply stopping, which is unsafe for
        # local PointGoal planning and makes calibration incomparable to zero.
        zero_anchor = torch.zeros(1, horizon, action_dim, device=device)
        primitive_anchors = _primitive_action_anchors(horizon, bounds)
        actions = torch.cat((actions, zero_anchor, primitive_anchors[1:]), dim=0)
        scores = score_action_sequences(
            model,
            context,
            actions.unsqueeze(0),
            planner_mode=planner_mode,
            goal_emb=goal_emb,
            goal_relative_xy=goal_relative_xy,
            safety_weight=safety_weight,
            smoothness_weight=smoothness_weight,
            history_size=history_size,
            bev_resolution_m=bev_resolution_m,
            robot_radius_m=robot_radius_m,
            unknown_weight=unknown_weight,
        )
        flat_cost = scores.total_costs[0]
        safe = scores.safety_costs[0] <= float(safety_threshold)
        if not bool(safe.any()):
            fallback_used = True
            break
        masked_cost = flat_cost.masked_fill(~safe, float("inf"))
        elite_indices = torch.topk(masked_cost, k=min(elites, int(safe.sum())), largest=False).indices
        elite_actions = actions[elite_indices]
        elite_costs = masked_cost[elite_indices]
        mean = elite_actions.mean(dim=0)
        # Use population std so a single safe elite yields zero variance
        # instead of NaN from the unbiased estimator's zero degrees of freedom.
        scale = elite_actions.std(dim=0, unbiased=False).clamp_min(1e-3)
        chosen = int(elite_indices[0])
        if float(elite_costs[0]) < best_cost:
            best_cost = float(elite_costs[0])
            best_actions = actions[chosen].clone()
            best_elite_costs = elite_costs.clone()
            best_goal_cost = float(scores.goal_costs[0, chosen])
            best_safety_cost = float(scores.safety_costs[0, chosen])
            best_smoothness_cost = float(scores.smoothness_costs[0, chosen])
            if scores.predicted_relative_pose is not None:
                best_relative_trajectory = scores.predicted_relative_pose[0, chosen].detach().clone()

    if fallback_used:
        best_actions = torch.zeros(horizon, 10, device=device)
        best_elite_costs = torch.zeros(1, device=device)
        if goal_relative_xy is not None:
            best_goal_cost = float(torch.linalg.vector_norm(goal_relative_xy[0]))
        best_safety_cost = 1.0
        best_smoothness_cost = 0.0
        best_cost = best_goal_cost + float(safety_weight) * best_safety_cost
    elif best_actions is None:
        best_actions = torch.zeros(horizon, 10, device=device)
        best_elite_costs = torch.zeros(1, device=device)
    return PlanResult(
        actions=best_actions,
        cost=best_cost,
        elite_costs=best_elite_costs,
        goal_cost=best_goal_cost,
        safety_cost=best_safety_cost,
        smoothness_cost=best_smoothness_cost,
        fallback_used=fallback_used,
        predicted_relative_trajectory=best_relative_trajectory,
    )
