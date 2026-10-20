"""Visualize rollout BEV probes and compute spatial metrics on held-out data."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

if __package__ in (None, ""):
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from training.action_protocol import require_action_protocol
    from training.ball_wm_dataset import BallWMDataset, IMAGENET_MEAN, IMAGENET_STD
    from training.joint_jepa import JointModelConfig, build_joint_model
else:
    from .action_protocol import require_action_protocol
    from .ball_wm_dataset import BallWMDataset, IMAGENET_MEAN, IMAGENET_STD
    from .joint_jepa import JointModelConfig, build_joint_model


def multiclass_metrics(
    predicted: np.ndarray,
    target: np.ndarray,
    num_classes: int,
) -> dict[str, object]:
    """Return accuracy, mean IoU, and per-class IoU for a label map."""

    predicted = np.asarray(predicted, dtype=np.int64)
    target = np.asarray(target, dtype=np.int64)
    if predicted.shape != target.shape:
        raise ValueError(f"predicted and target shapes differ: {predicted.shape} vs {target.shape}")
    if num_classes < 1:
        raise ValueError("num_classes must be positive")
    confusion = np.zeros((num_classes, num_classes), dtype=np.int64)
    valid = (
        (target >= 0)
        & (target < num_classes)
        & (predicted >= 0)
        & (predicted < num_classes)
    )
    np.add.at(confusion, (target[valid], predicted[valid]), 1)
    intersection = np.diag(confusion)
    union = confusion.sum(axis=1) + confusion.sum(axis=0) - intersection
    per_class: list[float | None] = [
        None if union[index] == 0 else float(intersection[index] / union[index])
        for index in range(num_classes)
    ]
    valid_ious = [value for value in per_class if value is not None]
    predicted_occupied_fraction = (
        float(np.mean(predicted[valid] == 1))
        if num_classes > 1 and np.any(valid)
        else 0.0
    )
    return {
        "accuracy": float(np.mean(predicted[valid] == target[valid])) if np.any(valid) else 0.0,
        "mean_iou": float(np.mean(valid_ious)) if valid_ious else None,
        "per_class_iou": per_class,
        "predicted_occupied_fraction": predicted_occupied_fraction,
        "confusion": confusion.tolist(),
    }


def binary_metrics(predicted: np.ndarray, target: np.ndarray) -> dict[str, float | None]:
    """Return pixel accuracy and positive-class IoU for binary masks."""

    predicted = np.asarray(predicted, dtype=bool)
    target = np.asarray(target, dtype=bool)
    if predicted.shape != target.shape:
        raise ValueError(f"predicted and target shapes differ: {predicted.shape} vs {target.shape}")
    intersection = int(np.count_nonzero(predicted & target))
    union = int(np.count_nonzero(predicted | target))
    return {
        "accuracy": float(np.mean(predicted == target)),
        "iou": None if union == 0 else float(intersection / union),
    }


def _rgb_image(array: np.ndarray | torch.Tensor) -> Image.Image:
    if torch.is_tensor(array):
        tensor = array.detach().cpu().float()
        tensor = tensor * IMAGENET_STD[:, None, None] + IMAGENET_MEAN[:, None, None]
        array = (tensor.clamp(0.0, 1.0).permute(1, 2, 0).numpy() * 255.0).round().astype(np.uint8)
    array = np.asarray(array)
    if array.ndim != 3 or array.shape[-1] != 3:
        raise ValueError(f"RGB image must have shape (H, W, 3), got {array.shape}")
    if array.dtype != np.uint8:
        array = np.clip(array, 0, 255).astype(np.uint8)
    return Image.fromarray(array, mode="RGB")


def _bev_image(array: np.ndarray, palette: Sequence[tuple[int, int, int]]) -> Image.Image:
    labels = np.asarray(array, dtype=np.int64)
    colors = np.asarray(palette, dtype=np.uint8)
    if labels.ndim != 2 or labels.size == 0 or labels.min() < 0 or labels.max() >= len(colors):
        raise ValueError(f"BEV labels are outside palette: shape={labels.shape}")
    return Image.fromarray(colors[labels], mode="RGB")


def _mask_image(array: np.ndarray) -> Image.Image:
    mask = np.asarray(array, dtype=bool)
    pixels = np.zeros((*mask.shape, 3), dtype=np.uint8)
    pixels[mask] = (40, 180, 90)
    pixels[~mask] = (245, 245, 245)
    return Image.fromarray(pixels, mode="RGB")


def render_comparison(
    left_rgb: np.ndarray | torch.Tensor,
    right_rgb: np.ndarray | torch.Tensor,
    bev_target: np.ndarray,
    bev_predicted: np.ndarray,
    current_target: np.ndarray,
    current_predicted: np.ndarray,
    history_target: np.ndarray,
    history_predicted: np.ndarray,
    metrics: dict[str, object],
) -> Image.Image:
    """Build a fixed-size contact sheet for one rollout target frame."""

    occupied_fraction = metrics["bev"].get("predicted_occupied_fraction")
    if occupied_fraction is None:
        occupied_fraction = float(np.mean(np.asarray(bev_predicted) == 1))
    canvas = Image.new("RGB", (960, 700), "white")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.load_default()
    panels = [
        ("left RGB input", _rgb_image(left_rgb)),
        ("right RGB input", _rgb_image(right_rgb)),
        (
            "BEV target",
            _bev_image(bev_target, ((245, 245, 245), (190, 45, 45), (120, 120, 120))),
        ),
        (
            "BEV rollout prediction",
            _bev_image(bev_predicted, ((245, 245, 245), (190, 45, 45), (120, 120, 120))),
        ),
        ("current target", _mask_image(current_target)),
        ("current prediction", _mask_image(current_predicted)),
        ("history target", _mask_image(history_target)),
        ("history prediction", _mask_image(history_predicted)),
    ]
    panel_width, panel_height = 240, 260
    for index, (title, image) in enumerate(panels):
        row, column = divmod(index, 4)
        x = column * panel_width
        y = row * panel_height
        draw.text((x + 6, y + 6), title, fill="black", font=font)
        image.thumbnail((panel_width - 12, panel_height - 34), Image.Resampling.NEAREST)
        canvas.paste(image, (x + (panel_width - image.width) // 2, y + 28))
    draw.text(
        (8, 525),
        "BEV accuracy={:.3f}  mIoU={}  occupied frac={:.3f}  current IoU={}  history IoU={}".format(
            float(metrics["bev"]["accuracy"]),
            _format_metric(metrics["bev"]["mean_iou"]),
            float(occupied_fraction),
            _format_metric(metrics["current"]["iou"]),
            _format_metric(metrics["history"]["iou"]),
        ),
        fill="black",
        font=font,
    )
    draw.text((8, 550), "BEV classes: free=0, occupied=1, unknown=2; green mask pixels are positive.", fill="black", font=font)
    return canvas


def _format_metric(value: object) -> str:
    return "n/a" if value is None else f"{float(value):.3f}"


def _context(sample: dict[str, torch.Tensor], history: int) -> dict[str, torch.Tensor]:
    return {
        "pixels_left": sample["pixels_left"][:history].unsqueeze(0),
        "pixels_right": sample["pixels_right"][:history].unsqueeze(0),
        "action": sample["action"][:history].unsqueeze(0),
        "proprio": sample["proprio"][:history].unsqueeze(0),
    }


def _future_actions(sample: dict[str, torch.Tensor], history: int, horizon: int) -> torch.Tensor:
    """Select action blocks that drive frames after the context window."""

    if horizon < 1:
        raise ValueError("horizon must be positive")
    start = max(0, history - 1)
    return sample["action"][start : start + horizon]


@torch.no_grad()
def evaluate_visuals(
    checkpoint_path: Path,
    data_root: Path,
    output_dir: Path,
    num_samples: int,
    start_index: int,
    device_name: str,
) -> dict[str, object]:
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    config = JointModelConfig(**checkpoint["config"])
    dataset = BallWMDataset(
        data_root,
        history_frames=config.history_size,
        num_preds=config.num_preds,
        rollout_horizon=config.rollout_horizon,
        image_size=(config.image_size, config.image_size),
    )
    require_action_protocol(
        checkpoint, dataset.action_protocol_id, artifact="checkpoint"
    )
    device = torch.device(device_name if torch.cuda.is_available() else "cpu")
    model = build_joint_model(config).to(device)
    model.load_state_dict(checkpoint["model"])
    model.eval()
    output_dir.mkdir(parents=True, exist_ok=True)

    records: list[dict[str, object]] = []
    end_index = min(len(dataset), start_index + num_samples)
    for sample_index in range(start_index, end_index):
        sample = dataset[sample_index]
        history = config.history_size
        rollout_steps = max(1, config.rollout_horizon)
        future_action = _future_actions(sample, history, rollout_steps).unsqueeze(0).unsqueeze(0)
        context = {key: value.to(device) for key, value in _context(sample, history).items()}
        predicted = model.rollout(context, future_action.to(device), history_size=history)
        bev_logits = model.bev_head(predicted)[0, 0, 0]
        current_logits = model.current_head(predicted)[0, 0, 0, 0]
        history_logits = model.history_head(predicted)[0, 0, 0, 0]
        bev_predicted = bev_logits.argmax(dim=0).cpu().numpy()
        current_predicted = (current_logits.sigmoid() >= 0.5).cpu().numpy()
        history_predicted = (history_logits.sigmoid() >= 0.5).cpu().numpy()
        bev_target = sample["geometry_bev"][history].numpy()
        current_target = sample["current_visible"][history].numpy()
        history_target = sample["history_observed"][history].numpy()
        metrics = {
            "bev": multiclass_metrics(bev_predicted, bev_target, config.bev_classes),
            "current": binary_metrics(current_predicted, current_target),
            "history": binary_metrics(history_predicted, history_target),
        }
        image = render_comparison(
            sample["pixels_left"][history],
            sample["pixels_right"][history],
            bev_target,
            bev_predicted,
            current_target,
            current_predicted,
            history_target,
            history_predicted,
            metrics,
        )
        frame_id = int(sample["frame_ids"][history])
        image_path = output_dir / f"sample_{sample_index:05d}_frame_{frame_id:06d}.png"
        image.save(image_path)
        records.append(
            {
                "sample_index": sample_index,
                "scene": dataset.sample_scenes[sample_index],
                "frame_id": frame_id,
                "image": str(image_path),
                "metrics": metrics,
            }
        )
    summary = {
        "checkpoint": str(checkpoint_path),
        "data_root": str(data_root),
        "action_protocol_id": dataset.action_protocol_id,
        "controls_per_block": dataset.controls_per_block,
        "device": str(device),
        "history_size": config.history_size,
        "num_samples": len(records),
        "records": records,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data_root", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--num_samples", type=int, default=10)
    parser.add_argument("--start_index", type=int, default=0)
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args(argv)
    if args.num_samples < 1 or args.start_index < 0:
        parser.error("num_samples must be positive and start_index must be non-negative")
    summary = evaluate_visuals(
        args.checkpoint,
        args.data_root,
        args.output_dir,
        args.num_samples,
        args.start_index,
        args.device,
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
