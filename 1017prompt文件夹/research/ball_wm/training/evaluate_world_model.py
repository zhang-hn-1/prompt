"""Isaac-free held-out metrics for action-conditioned future world-model predictions."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader, Subset

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.action_protocol import require_action_protocol
    from training.ball_wm_dataset import BallWMDataset, make_train_val_test_split
    from training.joint_jepa import JointModelConfig, build_joint_model
    from training.proprioception import checkpoint_velocity_frame
else:
    from .action_protocol import require_action_protocol
    from .ball_wm_dataset import BallWMDataset, make_train_val_test_split
    from .joint_jepa import JointModelConfig, build_joint_model
    from .proprioception import checkpoint_velocity_frame


_INPUT_KEYS = ("pixels_left", "pixels_right", "action", "proprio")


def _requested_horizons(horizons: Iterable[int]) -> tuple[int, ...]:
    values = tuple(int(horizon) for horizon in horizons)
    if not values or any(horizon < 1 for horizon in values) or len(set(values)) != len(values):
        raise ValueError("horizons must be non-empty, positive, and unique")
    return values


def select_future_prediction_test_indices(
    dataset: BallWMDataset,
    *,
    seed: int,
    test_scene_fraction: float = 0.15,
) -> tuple[list[int], list[str]]:
    """Select a deterministic scene-disjoint test set and report its scene names."""

    fraction = float(test_scene_fraction)
    if not math.isfinite(fraction) or not 0.0 < fraction < 1.0:
        raise ValueError("test_scene_fraction must be finite and strictly between 0 and 1")
    ratios = (1.0 - fraction, 0.0, fraction)
    _, _, test_indices = make_train_val_test_split(
        dataset,
        ratios=ratios,
        seed=seed,
        unseen_scene_test=True,
    )
    if not test_indices:
        raise ValueError("scene-disjoint test split has no samples")
    held_out_scenes = sorted({dataset.sample_scenes[index] for index in test_indices})
    return test_indices, held_out_scenes


def _finite_mean(values: Iterable[object]) -> float:
    array = np.asarray(list(values), dtype=np.float64).reshape(-1)
    finite = array[np.isfinite(array)]
    return float(finite.mean()) if finite.size else 0.0


def _pose_rows(values: Iterable[object], name: str) -> np.ndarray:
    rows = [np.asarray(value, dtype=np.float64) for value in values]
    if not rows:
        return np.empty((0, 4), dtype=np.float64)
    result = np.stack(rows)
    if result.ndim != 2 or result.shape[1] != 4 or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain finite pose rows shaped (4,)")
    return result


def _occupied_scores(confusion: np.ndarray) -> tuple[float, float]:
    matrix = np.asarray(confusion, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all() or (matrix < 0).any():
        raise ValueError("BEV confusion must be a finite non-negative 3x3 matrix")
    true_positive = float(matrix[1, 1])
    false_positive = float(matrix[:, 1].sum() - true_positive)
    false_negative = float(matrix[1, :].sum() - true_positive)
    iou_denominator = true_positive + false_positive + false_negative
    f1_denominator = 2.0 * true_positive + false_positive + false_negative
    return (
        0.0 if iou_denominator == 0.0 else true_positive / iou_denominator,
        0.0 if f1_denominator == 0.0 else 2.0 * true_positive / f1_denominator,
    )


def _bev_confusion(labels: torch.Tensor | np.ndarray, predicted: torch.Tensor | np.ndarray) -> np.ndarray:
    """Build a three-class BEV confusion matrix, optionally from a masked subset."""

    labels_tensor = torch.as_tensor(labels, dtype=torch.long).reshape(-1)
    predicted_tensor = torch.as_tensor(predicted, dtype=torch.long).reshape(-1)
    if labels_tensor.shape != predicted_tensor.shape:
        raise ValueError("BEV labels and predictions must have matching shapes")
    if (
        (labels_tensor < 0).any()
        or (labels_tensor >= 3).any()
        or (predicted_tensor < 0).any()
        or (predicted_tensor >= 3).any()
    ):
        raise ValueError("BEV labels/predictions must use classes 0, 1, 2")
    if labels_tensor.numel() == 0:
        return np.zeros((3, 3), dtype=np.int64)
    return (
        torch.bincount(labels_tensor * 3 + predicted_tensor, minlength=9)
        .reshape(3, 3)
        .cpu()
        .numpy()
        .astype(np.int64)
    )


def _occupied_calibration(confusion: np.ndarray) -> dict[str, float | list[list[int]]]:
    """Report occupied prevalence and detection calibration from a 3x3 confusion matrix."""

    matrix = np.asarray(confusion, dtype=np.int64)
    if matrix.shape != (3, 3) or (matrix < 0).any():
        raise ValueError("BEV confusion must be a non-negative 3x3 matrix")
    total = int(matrix.sum())
    true_positive = int(matrix[1, 1])
    false_positive = int(matrix[:, 1].sum() - true_positive)
    false_negative = int(matrix[1, :].sum() - true_positive)
    return {
        "target_occupied_fraction": 0.0 if total == 0 else float((true_positive + false_negative) / total),
        "predicted_occupied_fraction": 0.0 if total == 0 else float((true_positive + false_positive) / total),
        "occupied_precision": 0.0
        if true_positive + false_positive == 0
        else float(true_positive / (true_positive + false_positive)),
        "occupied_recall": 0.0
        if true_positive + false_negative == 0
        else float(true_positive / (true_positive + false_negative)),
        "bev_confusion": matrix.tolist(),
    }


def summarize_bev_confusions(
    bev_confusions: Mapping[int, np.ndarray],
    horizons: Iterable[int] = (1, 3, 5),
) -> dict[str, dict[str, object]]:
    """Summarize BEV confusion matrices for a teacher-forced decoder diagnostic."""

    requested = _requested_horizons(horizons)
    summaries: dict[str, dict[str, object]] = {}
    for horizon in requested:
        confusion = bev_confusions.get(horizon, np.zeros((3, 3), dtype=np.int64))
        occupied_iou, occupied_f1 = _occupied_scores(confusion)
        summaries[str(horizon)] = {
            "occupied_iou": occupied_iou,
            "occupied_f1": occupied_f1,
            **_occupied_calibration(confusion),
        }
    return summaries


def summarize_future_predictions(
    *,
    feature_cosines: Mapping[int, Iterable[object]],
    feature_errors: Mapping[int, Iterable[object]],
    pose_predictions: Mapping[int, Iterable[object]],
    pose_targets: Mapping[int, Iterable[object]],
    bev_confusions: Mapping[int, np.ndarray],
    horizons: Iterable[int] = (1, 3, 5),
) -> dict[str, object]:
    """Aggregate finite future prediction metrics at every requested one-indexed horizon."""

    requested = _requested_horizons(horizons)
    per_horizon: dict[str, dict[str, float]] = {}
    endpoint_errors: list[float] = []
    for horizon in requested:
        predicted_pose = _pose_rows(pose_predictions.get(horizon, ()), f"pose_predictions[{horizon}]")
        target_pose = _pose_rows(pose_targets.get(horizon, ()), f"pose_targets[{horizon}]")
        if predicted_pose.shape != target_pose.shape:
            raise ValueError(f"pose prediction/target counts differ at horizon {horizon}")
        if len(predicted_pose):
            xy_error = np.abs(predicted_pose[:, :2] - target_pose[:, :2])
            xy_mae = float(xy_error.mean())
            xy_endpoint_error = float(np.linalg.norm(predicted_pose[:, :2] - target_pose[:, :2], axis=1).mean())
            predicted_yaw = np.arctan2(predicted_pose[:, 2], predicted_pose[:, 3])
            target_yaw = np.arctan2(target_pose[:, 2], target_pose[:, 3])
            wrapped_yaw_error = (predicted_yaw - target_yaw + np.pi) % (2.0 * np.pi) - np.pi
            yaw_mae = float(np.degrees(np.abs(wrapped_yaw_error)).mean())
        else:
            xy_mae = 0.0
            xy_endpoint_error = 0.0
            yaw_mae = 0.0
        endpoint_errors.append(xy_endpoint_error)
        confusion = bev_confusions.get(horizon, np.zeros((3, 3), dtype=np.int64))
        occupied_iou, occupied_f1 = _occupied_scores(confusion)
        occupied_calibration = _occupied_calibration(confusion)
        per_horizon[str(horizon)] = {
            "feature_cosine": _finite_mean(feature_cosines.get(horizon, ())),
            "feature_mse": _finite_mean(feature_errors.get(horizon, ())),
            "occupied_iou": occupied_iou,
            "occupied_f1": occupied_f1,
            **occupied_calibration,
            "xy_mae_m": xy_mae,
            "yaw_mae_deg": yaw_mae,
            "ade_m": float(np.mean(endpoint_errors)),
            "fde_m": xy_endpoint_error,
        }
    return {"per_horizon": per_horizon}


@torch.no_grad()
def evaluate_future_predictions(
    model: torch.nn.Module,
    loader: DataLoader,
    horizons: Iterable[int] = (1, 3, 5),
    device: torch.device | str = "cpu",
) -> dict[str, object]:
    """Evaluate one recorded action sequence per held-out batch without simulator imports."""

    requested = _requested_horizons(horizons)
    device = torch.device(device)
    history_size = int(getattr(model, "history_size", 0))
    rollout_horizon = int(getattr(model, "rollout_horizon", 0))
    if history_size < 1 or rollout_horizon < max(requested):
        raise ValueError("model history/rollout configuration cannot cover requested horizons")
    if getattr(model, "pose_head", None) is None:
        raise ValueError("future prediction evaluation requires an enabled pose head")

    feature_cosines: dict[int, list[float]] = {horizon: [] for horizon in requested}
    feature_errors: dict[int, list[float]] = {horizon: [] for horizon in requested}
    pose_predictions: dict[int, list[np.ndarray]] = {horizon: [] for horizon in requested}
    pose_targets: dict[int, list[np.ndarray]] = {horizon: [] for horizon in requested}
    bev_confusions: dict[int, np.ndarray] = {
        horizon: np.zeros((3, 3), dtype=np.int64) for horizon in requested
    }
    teacher_bev_confusions: dict[int, np.ndarray] = {
        horizon: np.zeros((3, 3), dtype=np.int64) for horizon in requested
    }
    visible_bev_confusions: dict[int, np.ndarray] = {
        horizon: np.zeros((3, 3), dtype=np.int64) for horizon in requested
    }
    teacher_visible_bev_confusions: dict[int, np.ndarray] = {
        horizon: np.zeros((3, 3), dtype=np.int64) for horizon in requested
    }
    sample_count = 0
    model.eval()

    for raw_batch in loader:
        if any(
            name not in raw_batch
            for name in (*_INPUT_KEYS, "geometry_bev", "current_visible", "future_relative_pose")
        ):
            raise ValueError("evaluation batch is missing future world-model labels")
        batch = {name: value.to(device, non_blocking=True) for name, value in raw_batch.items()}
        available_frames = int(batch["action"].size(1))
        if available_frames < history_size + max(requested):
            raise ValueError("evaluation batch does not contain all requested future frames")
        context = {name: batch[name][:, :history_size] for name in _INPUT_KEYS}
        recorded_actions = batch["action"][:, history_size - 1 : history_size - 1 + max(requested)]
        action_sequences = recorded_actions.unsqueeze(1)
        teacher_info = model.encode({name: batch[name] for name in _INPUT_KEYS})
        teacher_future = teacher_info["emb"][:, history_size : history_size + max(requested)]
        predicted_future = model.rollout(context, action_sequences, history_size=history_size)[:, 0]
        predicted_pose = model.decode_relative_pose(predicted_future)
        predicted_bev = model.bev_head(predicted_future).argmax(dim=2)
        teacher_bev = model.bev_head(teacher_future).argmax(dim=2)
        target_bev = batch["geometry_bev"][:, history_size : history_size + max(requested)]
        target_pose = batch["future_relative_pose"][:, : max(requested)]
        if predicted_pose.shape != target_pose.shape:
            raise ValueError("future_relative_pose must match the evaluated rollout shape")

        sample_count += int(predicted_future.size(0))
        for horizon in requested:
            index = horizon - 1
            predicted_feature = F.normalize(predicted_future[:, index], dim=-1)
            target_feature = F.normalize(teacher_future[:, index], dim=-1)
            feature_cosines[horizon].extend(F.cosine_similarity(predicted_feature, target_feature, dim=-1).cpu().tolist())
            feature_errors[horizon].extend((predicted_feature - target_feature).pow(2).mean(dim=-1).cpu().tolist())
            pose_predictions[horizon].extend(predicted_pose[:, index].cpu().numpy())
            pose_targets[horizon].extend(target_pose[:, index].cpu().numpy())

            labels = target_bev[:, index].to(dtype=torch.long)
            classes = predicted_bev[:, index].to(dtype=torch.long)
            matrix = _bev_confusion(labels, classes)
            bev_confusions[horizon] += matrix
            teacher_classes = teacher_bev[:, index].reshape(-1).to(dtype=torch.long)
            teacher_bev_confusions[horizon] += _bev_confusion(labels, teacher_classes.reshape_as(labels))
            visible = batch["current_visible"][:, history_size + index] > 0
            visible_labels = labels[visible]
            visible_classes = classes[visible]
            visible_teacher_classes = teacher_classes.reshape_as(labels)[visible]
            visible_bev_confusions[horizon] += _bev_confusion(visible_labels, visible_classes)
            teacher_visible_bev_confusions[horizon] += _bev_confusion(
                visible_labels,
                visible_teacher_classes,
            )

    if sample_count == 0:
        raise ValueError("held-out evaluation loader has no samples")
    metrics = {
        "sample_count": sample_count,
        **summarize_future_predictions(
            feature_cosines=feature_cosines,
            feature_errors=feature_errors,
            pose_predictions=pose_predictions,
            pose_targets=pose_targets,
            bev_confusions=bev_confusions,
            horizons=requested,
        ),
    }
    metrics["teacher_bev_per_horizon"] = summarize_bev_confusions(teacher_bev_confusions, requested)
    metrics["visible_bev_per_horizon"] = summarize_bev_confusions(visible_bev_confusions, requested)
    metrics["teacher_visible_bev_per_horizon"] = summarize_bev_confusions(
        teacher_visible_bev_confusions,
        requested,
    )
    return metrics


def run_future_prediction_evaluation(
    *,
    checkpoint_path: Path,
    data_root: Path,
    output_path: Path,
    batch_size: int = 8,
    num_workers: int = 0,
    seed: int = 3072,
    device_name: str = "cuda",
    horizons: Iterable[int] = (1, 3, 5),
    test_scene_fraction: float = 0.15,
) -> dict[str, object]:
    """Load a pose checkpoint, evaluate unseen scenes, and write strict JSON."""

    requested = _requested_horizons(horizons)
    if batch_size < 1 or num_workers < 0:
        raise ValueError("batch_size must be positive and num_workers non-negative")
    device = torch.device(device_name if not str(device_name).startswith("cuda") or torch.cuda.is_available() else "cpu")
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    if not isinstance(checkpoint, Mapping) or "config" not in checkpoint or "model" not in checkpoint:
        raise ValueError("checkpoint must contain config and model")
    checkpoint_velocity_frame(checkpoint)
    config = JointModelConfig(**checkpoint["config"])
    if not config.pose_head_enabled:
        raise ValueError("future prediction evaluation requires a pose-enabled checkpoint")
    if config.rollout_horizon < max(requested):
        raise ValueError("checkpoint rollout_horizon cannot cover requested evaluation horizons")
    dataset = BallWMDataset(
        data_root,
        history_frames=config.history_size,
        num_preds=config.num_preds,
        rollout_horizon=config.rollout_horizon,
        image_size=(config.image_size, config.image_size),
        action_source=config.action_source,
        proprio_velocity_frame=config.proprio_velocity_frame,
        action_block=5,
        with_bev=True,
        with_visibility=True,
    )
    require_action_protocol(
        checkpoint, dataset.action_protocol_id, artifact="checkpoint"
    )
    test_indices, held_out_scenes = select_future_prediction_test_indices(
        dataset,
        seed=seed,
        test_scene_fraction=test_scene_fraction,
    )
    loader = DataLoader(
        Subset(dataset, test_indices),
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
    )
    model = build_joint_model(config).to(device)
    model.load_state_dict(checkpoint["model"], strict=True)
    metrics = evaluate_future_predictions(model, loader, requested, device)
    result: dict[str, object] = {
        "checkpoint": str(checkpoint_path),
        "data_root": str(data_root),
        "config": asdict(config),
        "held_out_scenes": held_out_scenes,
        "test_scene_fraction": float(test_scene_fraction),
        "horizons": list(requested),
        "action_protocol_id": dataset.action_protocol_id,
        "controls_per_block": dataset.controls_per_block,
        **metrics,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, allow_nan=False))
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Held-out action-conditioned future world-model evaluation")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data_root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--batch_size", type=int, default=8)
    parser.add_argument("--num_workers", type=int, default=0)
    parser.add_argument("--seed", type=int, default=3072)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--horizons", type=int, nargs="+", default=(1, 3, 5))
    parser.add_argument("--test_scene_fraction", type=float, default=0.15)
    args = parser.parse_args(argv)
    result = run_future_prediction_evaluation(
        checkpoint_path=args.checkpoint,
        data_root=args.data_root,
        output_path=args.out,
        batch_size=args.batch_size,
        num_workers=args.num_workers,
        seed=args.seed,
        device_name=args.device,
        horizons=args.horizons,
        test_scene_fraction=args.test_scene_fraction,
    )
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
