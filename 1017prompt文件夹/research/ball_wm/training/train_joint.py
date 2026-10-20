"""Standalone trainer for the joint stereo/proprio LeWM world model."""

from __future__ import annotations

import argparse
import hashlib
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
from typing import Mapping

import torch
import yaml
from torch.utils.data import DataLoader

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.action_protocol import require_action_protocol
    from training.ball_wm_dataset import BallWMDataset, make_train_val_test_split
    from training.candidate_losses import (
        candidate_goal_distances,
        candidate_pose_loss,
        candidate_ranking_accuracy,
        pairwise_candidate_ranking_loss,
    )
    from training.candidate_rollout_dataset import CandidateRolloutDataset
    from training.joint_jepa import JointModelConfig, build_joint_model
    from training.lewm_modules import SIGReg
else:
    from .action_protocol import require_action_protocol
    from .ball_wm_dataset import BallWMDataset, make_train_val_test_split
    from .candidate_losses import (
        candidate_goal_distances,
        candidate_pose_loss,
        candidate_ranking_accuracy,
        pairwise_candidate_ranking_loss,
    )
    from .candidate_rollout_dataset import CandidateRolloutDataset
    from .joint_jepa import JointModelConfig, build_joint_model
    from .lewm_modules import SIGReg


_CANDIDATE_CONFIG_KEYS = frozenset(
    {
        "candidate_root",
        "candidate_formal_data_root",
        "candidate_batch_ratio",
        "candidate_pose_loss_weight",
        "candidate_ranking_loss_weight",
        "candidate_ranking_margin_m",
        "candidate_pose_horizons",
        "candidate_expected_candidates",
    }
)


def training_protocol_metadata(dataset: object) -> dict[str, object]:
    """Return validated action-protocol fields for manifests and checkpoints."""

    protocol_id = getattr(dataset, "action_protocol_id", None)
    controls_per_block = getattr(dataset, "controls_per_block", None)
    if not isinstance(protocol_id, str) or not protocol_id:
        raise ValueError("training dataset must expose action_protocol_id")
    if (
        isinstance(controls_per_block, bool)
        or not isinstance(controls_per_block, int)
        or controls_per_block < 1
    ):
        raise ValueError("training dataset must expose a positive controls_per_block")
    return {
        "action_protocol_id": protocol_id,
        "controls_per_block": controls_per_block,
    }


def require_initial_checkpoint_protocol(
    checkpoint: Mapping[str, object], expected_protocol_id: str
) -> str:
    """Prevent warm-starting V2 training from an unlabelled legacy checkpoint."""

    return require_action_protocol(
        checkpoint, expected_protocol_id, artifact="initial checkpoint"
    )


def split_training_config(values: Mapping[str, object]) -> tuple[dict[str, object], dict[str, object]]:
    """Separate model constructor fields from optional candidate-training fields."""

    if not isinstance(values, Mapping):
        raise ValueError("training config must be a mapping")
    model_values = {str(key): value for key, value in values.items() if key not in _CANDIDATE_CONFIG_KEYS}
    candidate_values = {str(key): value for key, value in values.items() if key in _CANDIDATE_CONFIG_KEYS}
    return model_values, candidate_values


def candidate_training_losses(
    model: torch.nn.Module,
    batch: Mapping[str, torch.Tensor],
    *,
    pose_weight: float = 1.0,
    ranking_weight: float = 1.0,
    margin_m: float = 0.05,
    horizons: tuple[int, ...] = (1, 3, 5),
) -> dict[str, torch.Tensor]:
    """Compute pose and pairwise ranking losses for one candidate batch."""

    required = (
        "pixels_left",
        "pixels_right",
        "proprio",
        "context_action",
        "candidate_action",
        "goal_relative_xy",
        "future_relative_pose",
        "goal_distance_m",
        "collision",
    )
    if any(name not in batch for name in required):
        missing = [name for name in required if name not in batch]
        raise ValueError(f"candidate batch is missing fields: {missing}")
    if pose_weight < 0.0 or ranking_weight < 0.0:
        raise ValueError("candidate loss weights must be non-negative")
    context = {
        "pixels_left": batch["pixels_left"],
        "pixels_right": batch["pixels_right"],
        "proprio": batch["proprio"],
        "action": batch["context_action"],
    }
    predicted_future = model.rollout(context, batch["candidate_action"], history_size=5)
    predicted_pose = model.decode_relative_pose(predicted_future)
    pose_loss = candidate_pose_loss(predicted_pose, batch["future_relative_pose"], horizons=horizons)
    predicted_distance = candidate_goal_distances(predicted_pose, batch["goal_relative_xy"])
    ranking_loss = pairwise_candidate_ranking_loss(
        predicted_distance,
        batch["goal_distance_m"],
        batch["collision"],
        margin_m=margin_m,
    )
    ranking_accuracy = candidate_ranking_accuracy(
        predicted_distance.detach(),
        batch["goal_distance_m"],
        batch["collision"],
        margin_m=margin_m,
    )
    total = float(pose_weight) * pose_loss + float(ranking_weight) * ranking_loss
    return {
        "candidate_loss": total,
        "candidate_pose_loss": pose_loss,
        "candidate_ranking_loss": ranking_loss,
        "candidate_ranking_accuracy": ranking_accuracy,
    }


def _warmup_cosine_lr(step: int, warmup_steps: int, total_steps: int) -> float:
    if step < warmup_steps:
        return float(step + 1) / float(max(1, warmup_steps))
    progress = (step - warmup_steps) / max(1, total_steps - warmup_steps)
    return 0.5 * (1.0 + math.cos(math.pi * min(1.0, progress)))


def _load_config(path: Path) -> dict[str, object]:
    """Load either JSON or the repository's YAML training config."""

    text = path.read_text(encoding="utf-8")
    try:
        loaded = json.loads(text)
    except json.JSONDecodeError:
        loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise ValueError(f"training config must be a mapping: {path}")
    return loaded


def _loss_total_keys() -> tuple[str, ...]:
    """Return every scalar loss recorded in train and validation statistics."""

    return (
        "loss",
        "pred_loss",
        "sigreg_loss",
        "bev_loss",
        "current_loss",
        "history_loss",
        "rollout_latent_loss",
        "rollout_bev_loss",
        "rollout_pose_loss",
    )


def _candidate_total_keys() -> tuple[str, ...]:
    return (
        "candidate_loss",
        "candidate_pose_loss",
        "candidate_ranking_loss",
        "candidate_ranking_accuracy",
    )


def train_one_epoch(
    model: torch.nn.Module,
    loader: DataLoader,
    optimizer: torch.optim.Optimizer,
    scheduler: torch.optim.lr_scheduler.LambdaLR,
    scaler: torch.cuda.amp.GradScaler,
    sigreg: SIGReg,
    sigreg_weight: float,
    device: torch.device,
    epoch: int,
    log_every: int = 10,
    candidate_loader: DataLoader | None = None,
    candidate_batch_ratio: float = 0.0,
    candidate_pose_loss_weight: float = 1.0,
    candidate_ranking_loss_weight: float = 1.0,
    candidate_ranking_margin_m: float = 0.05,
    candidate_pose_horizons: tuple[int, ...] = (1, 3, 5),
) -> dict[str, float]:
    if not 0.0 <= float(candidate_batch_ratio) <= 1.0:
        raise ValueError("candidate_batch_ratio must be in [0, 1]")
    if candidate_loader is None and float(candidate_batch_ratio) > 0.0:
        raise ValueError("candidate_batch_ratio requires candidate_loader")
    model.train()
    totals = {name: 0.0 for name in _loss_total_keys()}
    candidate_totals = {name: 0.0 for name in _candidate_total_keys()}
    candidate_steps = 0
    candidate_iterator = None if candidate_loader is None else iter(candidate_loader)
    candidate_accumulator = 0.0
    start = time.time()
    for step, batch in enumerate(loader):
        batch = {key: value.to(device, non_blocking=True) for key, value in batch.items()}
        optimizer.zero_grad(set_to_none=True)
        with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=scaler.is_enabled()):
            output = model(batch)
            losses = model.losses(output, sigreg, sigreg_weight)
        scaler.scale(losses["loss"]).backward()
        scaler.step(optimizer)
        scaler.update()
        scheduler.step()
        for key in totals:
            totals[key] += float(losses[key].detach())
        if (step + 1) % log_every == 0:
            print(
                f"epoch {epoch} step {step + 1}/{len(loader)} "
                f"loss={float(losses['loss']):.4f} bev={float(losses['bev_loss']):.4f} "
                f"pose={float(losses['rollout_pose_loss']):.4f} "
                f"lr={scheduler.get_last_lr()[0]:.2e} ({time.time() - start:.0f}s)",
                flush=True,
            )
        if candidate_loader is not None and candidate_batch_ratio > 0.0:
            candidate_accumulator += float(candidate_batch_ratio)
            if candidate_accumulator + 1e-12 >= 1.0:
                candidate_accumulator -= 1.0
                try:
                    candidate_batch = next(candidate_iterator)
                except StopIteration:
                    candidate_iterator = iter(candidate_loader)
                    candidate_batch = next(candidate_iterator)
                candidate_batch = {
                    key: value.to(device, non_blocking=True) for key, value in candidate_batch.items()
                }
                optimizer.zero_grad(set_to_none=True)
                with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=scaler.is_enabled()):
                    candidate_losses = candidate_training_losses(
                        model,
                        candidate_batch,
                        pose_weight=candidate_pose_loss_weight,
                        ranking_weight=candidate_ranking_loss_weight,
                        margin_m=candidate_ranking_margin_m,
                        horizons=candidate_pose_horizons,
                    )
                scaler.scale(candidate_losses["candidate_loss"]).backward()
                scaler.step(optimizer)
                scaler.update()
                scheduler.step()
                candidate_steps += 1
                for key in candidate_totals:
                    candidate_totals[key] += float(candidate_losses[key].detach())
    result = {key: value / max(1, len(loader)) for key, value in totals.items()}
    result.update(
        {
            key: value / max(1, candidate_steps) if candidate_steps else 0.0
            for key, value in candidate_totals.items()
        }
    )
    result["candidate_batch_count"] = float(candidate_steps)
    return result


@torch.no_grad()
def evaluate(
    model: torch.nn.Module,
    loader: DataLoader,
    sigreg: SIGReg,
    sigreg_weight: float,
    device: torch.device,
) -> dict[str, float]:
    model.eval()
    totals = {name: 0.0 for name in _loss_total_keys()}
    for batch in loader:
        batch = {key: value.to(device, non_blocking=True) for key, value in batch.items()}
        output = model(batch)
        losses = model.losses(output, sigreg, sigreg_weight)
        for key in totals:
            totals[key] += float(losses[key])
    return {key: value / max(1, len(loader)) for key, value in totals.items()}


@torch.no_grad()
def evaluate_candidate_training(
    model: torch.nn.Module,
    loader: DataLoader,
    device: torch.device,
    *,
    candidate_pose_loss_weight: float = 1.0,
    candidate_ranking_loss_weight: float = 1.0,
    candidate_ranking_margin_m: float = 0.05,
    candidate_pose_horizons: tuple[int, ...] = (1, 3, 5),
) -> dict[str, float]:
    """Evaluate candidate pose/ranking losses on a held-out context split."""

    model.eval()
    totals = {name: 0.0 for name in _candidate_total_keys()}
    for raw_batch in loader:
        batch = {key: value.to(device, non_blocking=True) for key, value in raw_batch.items()}
        losses = candidate_training_losses(
            model,
            batch,
            pose_weight=candidate_pose_loss_weight,
            ranking_weight=candidate_ranking_loss_weight,
            margin_m=candidate_ranking_margin_m,
            horizons=candidate_pose_horizons,
        )
        for key in totals:
            totals[key] += float(losses[key])
    return {key: value / max(1, len(loader)) for key, value in totals.items()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_root", type=Path, action="append", required=True)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--out_dir", type=Path, default=Path("/tmp/joint_wm_train"))
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch_size", type=int, default=8)
    parser.add_argument("--lr", type=float, default=5e-5)
    parser.add_argument("--weight_decay", type=float, default=1e-3)
    parser.add_argument("--sigreg_weight", type=float, default=0.09)
    parser.add_argument("--image_size", type=int, default=224)
    parser.add_argument("--num_workers", type=int, default=0)
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--train_ratio", type=float, default=0.7)
    parser.add_argument("--val_ratio", type=float, default=0.3)
    parser.add_argument("--unseen_scene_test", action="store_true")
    parser.add_argument("--scene_split_manifest", type=Path, default=None)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--init_checkpoint", type=Path, default=None)
    parser.add_argument("--candidate_root", type=Path, default=None)
    parser.add_argument("--candidate_formal_data_root", type=Path, default=None)
    parser.add_argument("--candidate_batch_ratio", type=float, default=None)
    parser.add_argument("--candidate_pose_loss_weight", type=float, default=None)
    parser.add_argument("--candidate_ranking_loss_weight", type=float, default=None)
    parser.add_argument("--candidate_ranking_margin_m", type=float, default=None)
    parser.add_argument("--candidate_expected_candidates", type=int, default=None)
    args = parser.parse_args()

    torch.manual_seed(args.seed)
    config_values: dict[str, object] = {}
    if args.config:
        loaded = _load_config(args.config)
        config_values.update({k: v for k, v in loaded.items() if k not in {"data_root", "epochs"}})
    config_values, candidate_values = split_training_config(config_values)
    config_values["image_size"] = args.image_size
    model_config = JointModelConfig(**config_values)

    overrides = {
        "candidate_root": args.candidate_root,
        "candidate_formal_data_root": args.candidate_formal_data_root,
        "candidate_batch_ratio": args.candidate_batch_ratio,
        "candidate_pose_loss_weight": args.candidate_pose_loss_weight,
        "candidate_ranking_loss_weight": args.candidate_ranking_loss_weight,
        "candidate_ranking_margin_m": args.candidate_ranking_margin_m,
        "candidate_expected_candidates": args.candidate_expected_candidates,
    }
    candidate_values.update({key: value for key, value in overrides.items() if value is not None})
    candidate_root_value = candidate_values.get("candidate_root")
    if candidate_root_value is not None and not str(candidate_root_value).strip():
        candidate_root_value = None
    candidate_batch_ratio = (
        float(candidate_values.get("candidate_batch_ratio", 0.25))
        if candidate_root_value is not None
        else 0.0
    )
    candidate_pose_loss_weight = float(candidate_values.get("candidate_pose_loss_weight", 1.0))
    candidate_ranking_loss_weight = float(candidate_values.get("candidate_ranking_loss_weight", 1.0))
    candidate_ranking_margin_m = float(candidate_values.get("candidate_ranking_margin_m", 0.05))
    candidate_pose_horizons = tuple(int(value) for value in candidate_values.get("candidate_pose_horizons", (1, 3, 5)))
    candidate_expected_candidates = candidate_values.get("candidate_expected_candidates")
    if candidate_expected_candidates is not None:
        candidate_expected_candidates = int(candidate_expected_candidates)
    if candidate_root_value is not None:
        if not model_config.pose_head_enabled or model_config.rollout_horizon < 5:
            raise ValueError("candidate training requires pose_head_enabled and rollout_horizon >= 5")
        if not 0.0 <= candidate_batch_ratio <= 1.0:
            raise ValueError("candidate_batch_ratio must be in [0, 1]")
        if candidate_formal_data_root := candidate_values.get("candidate_formal_data_root"):
            candidate_formal_root = Path(candidate_formal_data_root)
        elif len(args.data_root) == 1:
            candidate_formal_root = Path(args.data_root[0])
        else:
            raise ValueError("candidate_formal_data_root is required when using multiple --data_root values")
    else:
        candidate_formal_root = None

    dataset = BallWMDataset(
        args.data_root,
        history_frames=model_config.history_size,
        num_preds=model_config.num_preds,
        rollout_horizon=model_config.rollout_horizon,
        image_size=(model_config.image_size, model_config.image_size),
    )
    protocol_metadata = training_protocol_metadata(dataset)
    train_idx, val_idx, test_idx = make_train_val_test_split(
        dataset,
        ratios=(args.train_ratio, args.val_ratio, 1.0 - args.train_ratio - args.val_ratio),
        seed=args.seed,
        unseen_scene_test=args.unseen_scene_test,
    )
    if args.scene_split_manifest is not None:
        from training.ablation_protocol import scene_indices
        if candidate_root_value is not None:
            raise ValueError('explicit split is currently supported for formal-data training only')
        train_idx, val_idx, test_idx = scene_indices(
            dataset.sample_scenes, json.loads(args.scene_split_manifest.read_text())
        )
    train_set = torch.utils.data.Subset(dataset, train_idx)
    val_set = torch.utils.data.Subset(dataset, val_idx)
    train_loader = DataLoader(
        train_set,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        drop_last=True,
        generator=torch.Generator().manual_seed(args.seed),
    )
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers)

    candidate_train_loader = None
    candidate_val_loader = None
    candidate_train_idx: list[int] = []
    candidate_val_idx: list[int] = []
    candidate_dataset = None
    if candidate_root_value is not None:
        candidate_dataset = CandidateRolloutDataset(
            candidate_root_value,
            candidate_formal_root,
            image_size=(model_config.image_size, model_config.image_size),
            expected_candidates=candidate_expected_candidates,
        )
        if candidate_dataset.action_protocol_id != dataset.action_protocol_id:
            raise ValueError(
                "candidate and primary training datasets use different action protocols"
            )
        if candidate_dataset.controls_per_block != dataset.controls_per_block:
            raise ValueError(
                "candidate and primary training datasets use different controls_per_block"
            )
        candidate_train_idx, candidate_val_idx, _candidate_test_idx = candidate_dataset.scene_splits(
            ratios=(args.train_ratio, args.val_ratio, 1.0 - args.train_ratio - args.val_ratio),
            seed=args.seed,
        )
        if not candidate_train_idx:
            raise ValueError("candidate training split has no contexts")
        candidate_train_loader = DataLoader(
            torch.utils.data.Subset(candidate_dataset, candidate_train_idx),
            batch_size=args.batch_size,
            shuffle=True,
            num_workers=args.num_workers,
        )
        if candidate_val_idx:
            candidate_val_loader = DataLoader(
                torch.utils.data.Subset(candidate_dataset, candidate_val_idx),
                batch_size=args.batch_size,
                shuffle=False,
                num_workers=args.num_workers,
            )

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    model = build_joint_model(model_config).to(device)
    if args.init_checkpoint is not None:
        initial_checkpoint = torch.load(args.init_checkpoint, map_location=device, weights_only=False)
        if not isinstance(initial_checkpoint, dict) or "model" not in initial_checkpoint:
            raise ValueError("init checkpoint must contain a model state dict")
        require_initial_checkpoint_protocol(
            initial_checkpoint, dataset.action_protocol_id
        )
        model.load_state_dict(initial_checkpoint["model"])
    sigreg = SIGReg(knots=17, num_proj=1024).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    candidate_steps_per_epoch = (
        int(math.floor(float(candidate_batch_ratio) * len(train_loader) + 1e-9))
        if candidate_train_loader is not None
        else 0
    )
    total_steps = max(1, args.epochs * (len(train_loader) + candidate_steps_per_epoch))
    warmup_steps = min(50, max(1, total_steps // 10))
    scheduler = torch.optim.lr_scheduler.LambdaLR(
        optimizer,
        lambda step: _warmup_cosine_lr(step, warmup_steps, total_steps),
    )
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out_dir / 'run_manifest.json'
    if manifest_path.exists() or any(args.out_dir.glob('joint_wm_epoch*.ckpt')):
        raise ValueError('output directory already contains training results')
    train_scenes = {dataset.sample_scenes[i] for i in train_idx}
    val_scenes = {dataset.sample_scenes[i] for i in val_idx}
    test_scenes = {dataset.sample_scenes[i] for i in test_idx}
    digest = hashlib.sha256()
    for name, tensor in sorted(model.state_dict().items()):
        digest.update(name.encode())
        digest.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    run_manifest = {
        **protocol_metadata,
        'initial_weights_sha256': digest.hexdigest(),
        'initialization': 'random' if args.init_checkpoint is None else str(args.init_checkpoint.resolve()),
        'arguments': {key: str(value) if isinstance(value, Path) else
                      [str(v) for v in value] if isinstance(value, list) else value
                      for key, value in vars(args).items()},
        'scene_split': {'train': sorted(train_scenes), 'val': sorted(val_scenes), 'test': sorted(test_scenes)},
        'sample_counts': {'train': len(train_idx), 'val': len(val_idx), 'test': len(test_idx)},
        'config': asdict(model_config),
    }
    manifest_path.write_text(json.dumps(run_manifest, indent=2))
    print(
        f"samples train={len(train_idx)} val={len(val_idx)} test={len(test_idx)} device={device}",
        flush=True,
    )
    print(f"train scenes={sorted(train_scenes)}", flush=True)
    print(f"val scenes={sorted(val_scenes)}", flush=True)
    print(f"test scenes={sorted(test_scenes)}", flush=True)
    if candidate_dataset is not None:
        print(
            f"candidate contexts train={len(candidate_train_idx)} val={len(candidate_val_idx)} "
            f"ratio={candidate_batch_ratio:.3f}",
            flush=True,
        )
    for epoch in range(args.epochs):
        train_stats = train_one_epoch(
            model, train_loader, optimizer, scheduler, scaler, sigreg,
            args.sigreg_weight, device, epoch,
            candidate_loader=candidate_train_loader,
            candidate_batch_ratio=candidate_batch_ratio,
            candidate_pose_loss_weight=candidate_pose_loss_weight,
            candidate_ranking_loss_weight=candidate_ranking_loss_weight,
            candidate_ranking_margin_m=candidate_ranking_margin_m,
            candidate_pose_horizons=candidate_pose_horizons,
        )
        val_stats = evaluate(model, val_loader, sigreg, args.sigreg_weight, device)
        if candidate_val_loader is not None:
            val_stats.update(
                evaluate_candidate_training(
                    model,
                    candidate_val_loader,
                    device,
                    candidate_pose_loss_weight=candidate_pose_loss_weight,
                    candidate_ranking_loss_weight=candidate_ranking_loss_weight,
                    candidate_ranking_margin_m=candidate_ranking_margin_m,
                    candidate_pose_horizons=candidate_pose_horizons,
                )
            )
        print(f"epoch {epoch} train={ {k: round(v, 4) for k, v in train_stats.items()} }", flush=True)
        print(f"epoch {epoch} val  ={ {k: round(v, 4) for k, v in val_stats.items()} }", flush=True)
        checkpoint = {
            **protocol_metadata,
            "run_manifest": run_manifest,
            "epoch": epoch,
            "model": model.state_dict(),
            "config": asdict(model_config),
            "optimizer": optimizer.state_dict(),
            "train_stats": train_stats,
            "val_stats": val_stats,
            "candidate_config": {
                key: (str(value) if isinstance(value, Path) else value)
                for key, value in candidate_values.items()
            },
        }
        torch.save(checkpoint, args.out_dir / f"joint_wm_epoch{epoch:02d}.ckpt")
        print(f"saved {args.out_dir / f'joint_wm_epoch{epoch:02d}.ckpt'}", flush=True)


if __name__ == "__main__":
    main()
