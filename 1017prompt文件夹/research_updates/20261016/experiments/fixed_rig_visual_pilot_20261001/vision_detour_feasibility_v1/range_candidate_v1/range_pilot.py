"""One RGB-only per-eye obstacle-range candidate; depth is offline supervision."""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


HERE = Path(__file__).resolve().parent
PILOT_DIR = HERE.parents[1]
sys.path.insert(0, str(PILOT_DIR))
from pilot import FREE, OBSTACLE, UNKNOWN, classify_depth_samples, rgb_input, sample_grid, yaw_from_quaternion  # noqa: E402

CAPTURE = HERE.parent
SEQUENCES = {
    "E0_train": PILOT_DIR / "integrated_fill_e0b/depth_truth",
    "E2_train": PILOT_DIR / "integrated_fill_e2/depth_truth",
    "E1_historical_test": PILOT_DIR / "integrated_fill_e1/depth_truth",
    "hard_0_E1_0003_historical_test": CAPTURE / "bilateral_depth_h3/depth_truth",
}
SEED = 20261002
EPOCHS = 10
NEAR_TRUE_M = 2.0
NEAR_PREDICTED_M = 2.5
OBSTACLE_PROBABILITY = 0.5


def make_input(rgb: np.ndarray, eye_ids: np.ndarray, device: torch.device) -> torch.Tensor:
    if np.asarray(rgb).ndim != 4 or np.asarray(rgb).shape[-1] != 3:
        raise ValueError("RGB input must be N×H×W×3")
    return rgb_input(np.asarray(rgb), eye_ids, device)


class RangeNet(nn.Module):
    """Small shared per-eye encoder, class logits and obstacle camera-Z output."""

    def __init__(self) -> None:
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(6, 24, 5, stride=2, padding=2), nn.ReLU(),
            nn.Conv2d(24, 32, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=2, dilation=2), nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=4, dilation=4), nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=8, dilation=8), nn.ReLU(),
        )
        self.head = nn.Sequential(nn.Conv2d(32, 24, 3, padding=1), nn.ReLU(),
                                  nn.Conv2d(24, 4, 1))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        hidden = self.encoder(x)
        return self.head(F.interpolate(hidden, size=x.shape[-2:], mode="bilinear", align_corners=False))


def joint_near_obstacle(obstacle_probability: np.ndarray, predicted_depth_m: np.ndarray) -> np.ndarray:
    return ((np.asarray(obstacle_probability) >= OBSTACLE_PROBABILITY)
            & (np.asarray(predicted_depth_m) <= NEAR_PREDICTED_M))


def load_sequence(folder: Path) -> dict:
    paths = sorted(folder.glob("frame_*.npz"))
    if not paths:
        raise FileNotFoundError(folder)
    uv = sample_grid()
    rgb, labels, depth, central, times, eyes = [], [], [], [], [], []
    for path in paths:
        with np.load(path) as frame:
            root = frame["root_position_world_m"]
            yaw = yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
            for eye_id, eye in enumerate(("left", "right")):
                observed_depth = frame[f"{eye}_depth_m"][::4, ::4].astype(np.float32)
                lab, cen = classify_depth_samples(
                    observed_depth, uv, frame[f"{eye}_camera_usd_world"], root, yaw)
                rgb.append(frame[f"{eye}_rgb"][::4, ::4].copy())
                labels.append(lab.reshape(120, 160))
                depth.append(observed_depth)
                central.append(cen.reshape(120, 160))
                times.append(float(frame["simulation_time_s"]))
                eyes.append(eye_id)
    return {"rgb": np.stack(rgb), "labels": np.stack(labels), "depth": np.stack(depth),
            "central": np.stack(central), "times": np.asarray(times),
            "eye_ids": np.asarray(eyes), "complete_frames": len(paths)}


def set_seed() -> None:
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    torch.use_deterministic_algorithms(True)


def training_loss(output: torch.Tensor, labels: torch.Tensor, actual_depth: torch.Tensor,
                  criterion: nn.Module, *, depth_supervision: str = "obstacle") -> torch.Tensor:
    classification = criterion(output[:, :3], labels)
    if depth_supervision == "obstacle":
        selected = labels == OBSTACLE
    elif depth_supervision == "all_known":
        selected = ((labels == FREE) | (labels == OBSTACLE)) & torch.isfinite(actual_depth)
    else:
        raise ValueError("depth_supervision must be obstacle or all_known")
    if not selected.any():
        return classification
    log_depth = torch.log(actual_depth[selected].clamp(0.45, 10))
    range_loss = F.smooth_l1_loss(output[:, 3][selected], log_depth)
    return classification + 0.5 * range_loss


def train_one_model(train_data: dict, device: torch.device, *,
                    depth_supervision: str = "obstacle") -> tuple[RangeNet, list[float]]:
    set_seed()
    model = RangeNet().to(device)
    labels = train_data["labels"]
    counts = np.bincount(labels.reshape(-1), minlength=3).astype(np.float64)
    weights = 1 / np.sqrt(np.maximum(counts, 1))
    weights = np.minimum(weights / weights.mean(), 3)
    criterion = nn.CrossEntropyLoss(weight=torch.as_tensor(weights, dtype=torch.float32, device=device))
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    losses = []
    for epoch in range(EPOCHS):
        model.train()
        order = np.random.permutation(len(labels))
        epoch_loss = 0.0
        for start in range(0, len(order), 4):
            ids = order[start:start + 4]
            x = make_input(train_data["rgb"][ids], train_data["eye_ids"][ids], device)
            y = torch.as_tensor(labels[ids].copy(), dtype=torch.long, device=device)
            actual_depth = torch.as_tensor(train_data["depth"][ids].copy(), dtype=torch.float32, device=device)
            optimizer.zero_grad(set_to_none=True)
            output = model(x)
            loss = training_loss(output, y, actual_depth, criterion,
                                 depth_supervision=depth_supervision)
            loss.backward()
            optimizer.step()
            epoch_loss += float(loss.detach()) * len(ids)
        losses.append(epoch_loss / len(labels))
        print(f"epoch {epoch + 1}/{EPOCHS}: loss={losses[-1]:.4f}", flush=True)
    return model.eval(), losses


def predict(model: RangeNet, dataset: dict, device: torch.device) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    probabilities, ranges = [], []
    with torch.no_grad():
        for start in range(0, len(dataset["rgb"]), 4):
            end = start + 4
            x = make_input(dataset["rgb"][start:end], dataset["eye_ids"][start:end], device)
            output = model(x)
            probabilities.append(output[:, :3].softmax(1).cpu().numpy())
            ranges.append(output[:, 3].clamp(math.log(0.45), math.log(10)).exp().cpu().numpy())
    class_probability = np.concatenate(probabilities, axis=0)
    depth = np.concatenate(ranges, axis=0)
    return class_probability.argmax(1).astype(np.uint8), class_probability[:, OBSTACLE], depth


def _stage_metrics(mask: np.ndarray, detected: np.ndarray) -> dict:
    n = int(mask.sum())
    return {"true_near_obstacle_pixels": n,
            "joint_near_obstacle_recall": float(detected[mask].mean()) if n else None}


def evaluate(model: RangeNet, data: dict, device: torch.device) -> dict:
    predicted_class, p_obstacle, predicted_depth = predict(model, data, device)
    truth = data["labels"]
    measured_depth = data["depth"]
    central = data["central"]
    detected = joint_near_obstacle(p_obstacle, predicted_depth)
    near_truth = (truth == OBSTACLE) & central & np.isfinite(measured_depth) & (measured_depth <= NEAR_TRUE_M)
    class_detected = p_obstacle >= OBSTACLE_PROBABILITY
    range_detected = predicted_depth <= NEAR_PREDICTED_M
    ground = (truth == FREE) & central
    obstacle = truth == OBSTACLE
    errors = np.abs(predicted_depth[obstacle] - measured_depth[obstacle])
    time_mask = data["times"][:, None, None]
    stages = {
        "0_to_2_s": _stage_metrics(near_truth & (time_mask < 2), detected),
        "2_to_4_s": _stage_metrics(near_truth & (time_mask >= 2) & (time_mask < 4), detected),
        "4_to_6_s": _stage_metrics(near_truth & (time_mask >= 4) & (time_mask < 6), detected),
    }
    return {
        "complete_frames": data["complete_frames"], "eye_frames": len(data["rgb"]),
        "near_obstacle": _stage_metrics(near_truth, detected),
        "near_obstacle_class_only_recall": float(class_detected[near_truth].mean()) if near_truth.any() else None,
        "near_obstacle_range_only_recall": float(range_detected[near_truth].mean()) if near_truth.any() else None,
        "near_obstacle_predicted_range_median_m": float(np.median(predicted_depth[near_truth])) if near_truth.any() else None,
        "near_obstacle_by_stage": stages,
        "known_ground_central_pixels": int(ground.sum()),
        "known_ground_false_near_obstacle_rate": float(detected[ground].mean()) if ground.any() else None,
        "range_truth_obstacle_pixels": int(obstacle.sum()),
        "range_median_absolute_error_m": float(np.median(errors)) if len(errors) else None,
        "range_p90_absolute_error_m": float(np.percentile(errors, 90)) if len(errors) else None,
        "unknown_pixels": int((truth == UNKNOWN).sum()),
        "unknown_class_recall": float((predicted_class[truth == UNKNOWN] == UNKNOWN).mean()) if (truth == UNKNOWN).any() else None,
    }


def e1_gate(metrics: dict) -> dict:
    reasons = []
    near = metrics["near_obstacle"]
    if near["true_near_obstacle_pixels"] < 100 or (near["joint_near_obstacle_recall"] or 0) < 0.85:
        reasons.append("complete E1 near-obstacle recall below 0.85")
    for name, stage in metrics["near_obstacle_by_stage"].items():
        if stage["true_near_obstacle_pixels"] >= 100 and (stage["joint_near_obstacle_recall"] or 0) < 0.80:
            reasons.append(f"{name} near-obstacle recall below 0.80")
    if metrics["known_ground_false_near_obstacle_rate"] is None or metrics["known_ground_false_near_obstacle_rate"] > 0.05:
        reasons.append("ground false near-obstacle rate above 0.05")
    if metrics["range_median_absolute_error_m"] is None or metrics["range_median_absolute_error_m"] > 0.50:
        reasons.append("median obstacle range error above 0.50 m")
    if metrics["range_p90_absolute_error_m"] is None or metrics["range_p90_absolute_error_m"] > 1.50:
        reasons.append("p90 obstacle range error above 1.50 m")
    return {"passed": not reasons, "rejection_reasons": reasons}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evaluate-saved", action="store_true",
                        help="recompute diagnostic metrics from the frozen checkpoint")
    args = parser.parse_args()
    torch.set_num_threads(2)
    device = torch.device("cpu")
    data = {name: load_sequence(folder) for name, folder in SEQUENCES.items()}
    if args.evaluate_saved:
        model = RangeNet().eval()
        model.load_state_dict(torch.load(HERE / "range_model.pt", map_location="cpu", weights_only=True))
        losses = json.loads((HERE / "results.json").read_text())["training_losses"]
    else:
        training = {key: np.concatenate([data["E0_train"][key], data["E2_train"][key]])
                    for key in ("rgb", "labels", "depth", "central", "times", "eye_ids")}
        model, losses = train_one_model(training, device)
    e1 = evaluate(model, data["E1_historical_test"], device)
    hard = evaluate(model, data["hard_0_E1_0003_historical_test"], device)
    result = {
        "protocol": str(HERE / "protocol.md"), "seed": SEED, "epochs": EPOCHS,
        "training_complete_sequences": {name: data[name]["complete_frames"] for name in ("E0_train", "E2_train")},
        "no_renderer_fields_at_inference": True,
        "training_losses": losses,
        "historical_e1_test": e1,
        "historical_hard0_test": hard,
        "training_e0_diagnostic": evaluate(model, data["E0_train"], device),
        "training_e2_diagnostic": evaluate(model, data["E2_train"], device),
        "e1_control_gate": e1_gate(e1),
    }
    if not args.evaluate_saved:
        torch.save(model.cpu().state_dict(), HERE / "range_model.pt")
    (HERE / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
