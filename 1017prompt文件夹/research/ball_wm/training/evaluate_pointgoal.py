"""Offline PointGoal planning evaluation against held-out episodes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.action_protocol import ActionProtocol, load_action_protocol, require_action_protocol
    from training.ball_wm_dataset import BallWMDataset, _load_episode
    from training.joint_jepa import JointModelConfig, build_joint_model
    from training.planner import bev_safety_cost, latent_goal_cost, plan
else:
    from .action_protocol import ActionProtocol, load_action_protocol, require_action_protocol
    from .ball_wm_dataset import BallWMDataset, _load_episode
    from .joint_jepa import JointModelConfig, build_joint_model
    from .planner import bev_safety_cost, latent_goal_cost, plan


_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"


def _context_from_sample(sample: dict[str, torch.Tensor], history: int) -> dict[str, torch.Tensor]:
    return {
        "pixels_left": sample["pixels_left"][:history].unsqueeze(0),
        "pixels_right": sample["pixels_right"][:history].unsqueeze(0),
        "action": sample["action"][:history].unsqueeze(0),
        "proprio": sample["proprio"][:history].unsqueeze(0),
    }


def _goal_info(
    dataset: BallWMDataset,
    episode_index: int,
    goal_frame: int,
    history: int,
) -> dict[str, torch.Tensor]:
    sample = dataset._frame_arrays(episode_index, goal_frame)
    return {
        "pixels_left": sample["pixels_left"][:1].unsqueeze(0),
        "pixels_right": sample["pixels_right"][:1].unsqueeze(0),
        "action": sample["action"][:1].unsqueeze(0),
        "proprio": sample["proprio"][:1].unsqueeze(0),
    }


def _random_baseline_cost(
    model: torch.nn.Module,
    context: dict[str, torch.Tensor],
    goal_emb: torch.Tensor,
    horizon: int,
    samples: int,
    safety_weight: float,
    history_size: int,
    device: torch.device,
    action_protocol: ActionProtocol,
) -> float:
    actions = _sample_random_action_sequences(action_protocol, samples, horizon, device)
    predicted = model.rollout(context, actions, history_size=history_size)
    cost = latent_goal_cost(predicted, goal_emb) + safety_weight * bev_safety_cost(model, predicted)
    return float(cost[0].mean())


def _sample_random_action_sequences(
    action_protocol: ActionProtocol,
    samples: int,
    horizon: int,
    device: torch.device,
) -> torch.Tensor:
    """Sample the diagnostic random baseline from the same bounds as CEM."""

    if samples < 1 or horizon < 1:
        raise ValueError("samples and horizon must be positive")
    bounds = action_protocol.block_bounds_torch().to(device)
    unit = torch.rand(
        1,
        samples,
        horizon,
        action_protocol.action_block_dim,
        device=device,
    )
    return bounds[:, 0] + unit * (bounds[:, 1] - bounds[:, 0])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data_root", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("/tmp/pointgoal_eval.json"))
    parser.add_argument("--num_eval", type=int, default=10)
    parser.add_argument("--goal_offset", type=int, default=5)
    parser.add_argument("--horizon", type=int, default=5)
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--elites", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--safety_weight", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    action_protocol = load_action_protocol(_CONTRACT_PATH)
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    require_action_protocol(
        checkpoint, action_protocol.protocol_id, artifact="checkpoint"
    )
    config = JointModelConfig(**checkpoint["config"])
    model = build_joint_model(config).to(device)
    model.load_state_dict(checkpoint["model"])
    model.eval()

    dataset = BallWMDataset(
        args.data_root,
        history_frames=config.history_size,
        num_preds=config.num_preds,
        rollout_horizon=config.rollout_horizon,
        image_size=(config.image_size, config.image_size),
        action_block=action_protocol.controls_per_block,
    )
    if dataset.action_protocol_id != action_protocol.protocol_id:
        raise ValueError(
            "evaluation dataset action protocol does not match the active contract: "
            f"{dataset.action_protocol_id} != {action_protocol.protocol_id}"
        )

    planned_costs: list[float] = []
    random_costs: list[float] = []
    used = 0
    with torch.no_grad():
        for sample_index in range(len(dataset)):
            if used >= args.num_eval:
                break
            episode_index, start = dataset.samples[sample_index]
            sample = dataset[sample_index]
            context_last_frame = int(sample["frame_ids"][config.history_size - 1])
            goal_frame = context_last_frame + args.goal_offset
            num_frames = _load_episode(str(dataset.episode_paths[episode_index]))["num_frames"]
            required_frames = max(config.num_preds, config.rollout_horizon)
            if goal_frame + config.history_size + required_frames >= num_frames:
                continue
            context = _context_from_sample(sample, config.history_size)
            context = {key: value.to(device) for key, value in context.items()}
            goal_info = _goal_info(dataset, episode_index, goal_frame, config.history_size)
            goal_info = {key: value.to(device) for key, value in goal_info.items()}
            goal_emb = model.encode(goal_info)["emb"]  # (1,1,D)
            result = plan(
                model,
                context,
                action_protocol=action_protocol,
                goal_emb=goal_emb,
                horizon=args.horizon,
                samples=args.samples,
                elites=args.elites,
                iterations=args.iterations,
                safety_weight=args.safety_weight,
                history_size=config.history_size,
                device=device,
            )
            random_cost = _random_baseline_cost(
                model,
                context,
                goal_emb,
                args.horizon,
                args.samples,
                args.safety_weight,
                config.history_size,
                device,
                action_protocol,
            )
            planned_costs.append(result.cost)
            random_costs.append(random_cost)
            used += 1

    summary = {
        "num_eval": used,
        "planned_cost_mean": sum(planned_costs) / max(1, len(planned_costs)),
        "random_cost_mean": sum(random_costs) / max(1, len(random_costs)),
        "planned_better_ratio": (
            sum(p < r for p, r in zip(planned_costs, random_costs)) / max(1, len(planned_costs))
        ),
        "horizon": args.horizon,
        "goal_offset": args.goal_offset,
        "action_protocol_id": action_protocol.protocol_id,
        "controls_per_block": action_protocol.controls_per_block,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
