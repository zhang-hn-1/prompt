"""Fixed-camera visual feasibility pilot. Renderer fields are offline labels only."""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import torch
from torch import nn


FREE, OBSTACLE, UNKNOWN = 0, 1, 2
FOCAL_PX = 16.45 / 13.5 * 480.0
CX, CY = 319.5, 239.5
DATA_ROOT = Path(__file__).resolve().parents[1] / "stereo_vertical_baseline_20260929_v1"
SEED = 20261001


def split_scenes(holdout: str) -> tuple[list[str], list[str]]:
    if holdout not in ("e1", "e2"):
        raise ValueError("Only the pre-registered E1/E2 folds are permitted")
    return sorted({"e0", "e1", "e2"} - {holdout}), [holdout]


def scene_paths(scene: str) -> list[Path]:
    if scene == "e0":
        folder = DATA_ROOT / "depth_audit_control_golden_e0_0001" / "depth_truth"
    elif scene in ("e1", "e2"):
        folder = DATA_ROOT / f"fixed_sequence_golden_{scene}_0001" / "depth_truth"
    else:
        raise ValueError(scene)
    paths = sorted(folder.glob("frame_*.npz"))
    if not paths:
        raise FileNotFoundError(folder)
    return paths


def yaw_from_quaternion(quaternion_wxyz: np.ndarray) -> float:
    w, x, y, z = np.asarray(quaternion_wxyz, dtype=np.float64)
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def classify_depth_samples(
    depth_m: np.ndarray,
    uv: np.ndarray,
    camera_usd_world: np.ndarray,
    root_position_world_m: np.ndarray,
    root_yaw: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Offline ground/obstacle labels and independent central-visibility mask."""
    depth = np.asarray(depth_m, dtype=np.float64).reshape(-1)
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    if len(depth) != len(uv):
        raise ValueError("depth and pixel coordinates must align")
    pose = np.asarray(camera_usd_world, dtype=np.float64)
    optical_rotation = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
    rays = np.column_stack(((uv[:, 0] - CX) / FOCAL_PX,
                            (uv[:, 1] - CY) / FOCAL_PX,
                            np.ones(len(uv))))
    valid_depth = np.isfinite(depth) & (depth >= 0.45) & (depth <= 10.0)
    safe_depth = np.where(valid_depth, depth, 0.0)
    hit = pose[:3, 3] + (rays * safe_depth[:, None]) @ optical_rotation.T
    root = np.asarray(root_position_world_m, dtype=np.float64)
    outside_robot = np.linalg.norm(hit - root, axis=1) >= 0.55
    dx, dy = hit[:, 0] - root[0], hit[:, 1] - root[1]
    c, s = np.cos(root_yaw), np.sin(root_yaw)
    forward = c * dx + s * dy
    lateral = -s * dx + c * dy
    valid = valid_depth & outside_robot
    ground = valid & (np.abs(hit[:, 2]) <= 0.12)
    obstacle = (valid & ~ground & (hit[:, 2] >= 0.25) & (hit[:, 2] <= 1.5)
                & (forward >= 0.35) & (forward <= 4.5))
    labels = np.full(len(depth), UNKNOWN, dtype=np.uint8)
    labels[ground] = FREE
    labels[obstacle] = OBSTACLE
    # The corridor footprint includes floor hits. A wall-specific height
    # restriction is already supplied by labels == OBSTACLE during scoring.
    central = (valid & (forward >= 0.6) & (forward <= 4.0)
               & (np.abs(lateral) <= 0.4))
    return labels, central


def sample_grid() -> np.ndarray:
    rows, cols = np.mgrid[0:480:4, 0:640:4]
    return np.column_stack((cols.reshape(-1), rows.reshape(-1)))


def load_scene(scene: str) -> dict:
    uv = sample_grid()
    images, labels, central, times, ids = [], [], [], [], []
    for path in scene_paths(scene):
        with np.load(path) as frame:
            root = frame["root_position_world_m"]
            yaw = yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
            time_s = float(frame["simulation_time_s"])
            for eye_id, eye in enumerate(("left", "right")):
                rgb = frame[f"{eye}_rgb"][::4, ::4]
                depth = frame[f"{eye}_depth_m"][::4, ::4]
                lab, cen = classify_depth_samples(
                    depth, uv, frame[f"{eye}_camera_usd_world"], root, yaw,
                )
                images.append(rgb)
                labels.append(lab.reshape(120, 160))
                central.append(cen.reshape(120, 160))
                times.append(time_s)
                ids.append(f"{path.stem}_{eye}")
    return {"scene": scene, "rgb": np.stack(images), "labels": np.stack(labels),
            "central": np.stack(central), "times": np.asarray(times), "ids": ids}


def rgb_input(rgb: np.ndarray, eye_ids: np.ndarray, device: torch.device) -> torch.Tensor:
    """Inference boundary: only current RGB, pixel location and eye identity."""
    if rgb.ndim != 4 or rgb.shape[-1] != 3:
        raise ValueError("expected a batch of H×W×3 RGB frames")
    n, h, w, _ = rgb.shape
    if len(eye_ids) != n:
        raise ValueError("eye IDs must align with RGB frames")
    image = torch.as_tensor(rgb.copy(), dtype=torch.float32, device=device).permute(0, 3, 1, 2) / 255.0
    ys = torch.linspace(-1, 1, h, device=device).view(1, 1, h, 1).expand(n, 1, h, w)
    xs = torch.linspace(-1, 1, w, device=device).view(1, 1, 1, w).expand(n, 1, h, w)
    eye = torch.as_tensor(eye_ids, dtype=torch.float32, device=device).view(n, 1, 1, 1)
    eye = (2 * eye - 1).expand(n, 1, h, w)
    return torch.cat((image, xs, ys, eye), dim=1)


class TinyPixelNet(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(6, 24, 3, padding=1), nn.ReLU(),
            nn.Conv2d(24, 24, 3, padding=2, dilation=2), nn.ReLU(),
            nn.Conv2d(24, 24, 3, padding=4, dilation=4), nn.ReLU(),
            nn.Conv2d(24, 3, 1),
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.net(image)


def set_seed() -> None:
    random.seed(SEED)
    np.random.seed(SEED)
    torch.manual_seed(SEED)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(SEED)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def evaluate(model: TinyPixelNet, dataset: dict, device: torch.device) -> dict:
    model.eval()
    predictions = []
    with torch.no_grad():
        for start in range(0, len(dataset["rgb"]), 8):
            stop = start + 8
            eye_ids = np.arange(start, min(stop, len(dataset["rgb"]))) % 2
            images = rgb_input(dataset["rgb"][start:stop], eye_ids, device)
            predictions.append(model(images).argmax(1).cpu().numpy().astype(np.uint8))
    pred = np.concatenate(predictions)
    truth = dataset["labels"]
    matrix = np.zeros((3, 3), dtype=np.int64)
    for actual in range(3):
        for predicted in range(3):
            matrix[actual, predicted] = np.sum((truth == actual) & (pred == predicted))
    unknown_recall = matrix[UNKNOWN, UNKNOWN] / max(matrix[UNKNOWN].sum(), 1)
    free_false_obstacle = matrix[FREE, OBSTACLE] / max(matrix[FREE].sum(), 1)
    central_wall = dataset["central"] & (truth == OBSTACLE) & (dataset["times"][:, None, None] <= 2.0)
    central_wall_recall = float(np.sum(central_wall & (pred == OBSTACLE)) / max(central_wall.sum(), 1))
    central_ground = dataset["central"] & (truth == FREE)
    per_frame = []
    for i, time_s in enumerate(dataset["times"]):
        mask = central_wall[i]
        per_frame.append({"time_s": float(time_s), "eye": "left" if i % 2 == 0 else "right",
                          "central_wall_pixels": int(mask.sum()),
                          "central_wall_recall": float(np.mean(pred[i][mask] == OBSTACLE)) if mask.any() else None})
    stable_wall_time = None
    frame_pairs = [per_frame[i:i + 2] for i in range(0, len(per_frame), 2)]
    for i in range(max(0, len(frame_pairs) - 2)):
        group = [item for pair in frame_pairs[i:i + 3] for item in pair]
        if all(item["central_wall_pixels"] > 0 and item["central_wall_recall"] >= 0.8 for item in group):
            stable_wall_time = frame_pairs[i][0]["time_s"]
            break
    return {
        "scene": dataset["scene"], "samples": int(len(dataset["rgb"])),
        "confusion_actual_rows_predicted_columns": matrix.tolist(),
        "unknown_recall": float(unknown_recall),
        "free_false_obstacle_rate": float(free_false_obstacle),
        "central_wall_0_to_2s_pixel_count": int(central_wall.sum()),
        "central_wall_0_to_2s_obstacle_recall": central_wall_recall if central_wall.any() else None,
        "earliest_three_frame_pair_stable_wall_time_s": stable_wall_time,
        "central_ground_pixel_count": int(central_ground.sum()),
        "central_ground_false_obstacle_rate": float(np.mean(pred[central_ground] == OBSTACLE)) if central_ground.any() else None,
        "central_ground_unknown_rate": float(np.mean(pred[central_ground] == UNKNOWN)) if central_ground.any() else None,
        "central_wall_by_eye_frame": per_frame,
    }


def train_fold(holdout: str, device: torch.device, output: Path, datasets: dict) -> dict:
    training_scenes, testing_scenes = split_scenes(holdout)
    set_seed()
    rgb = np.concatenate([datasets[s]["rgb"] for s in training_scenes])
    truth = np.concatenate([datasets[s]["labels"] for s in training_scenes])
    # Each sequence stores its left/right eye in alternating order.
    eye_ids = np.concatenate([np.arange(len(datasets[s]["rgb"])) % 2 for s in training_scenes])
    frequencies = np.bincount(truth.reshape(-1), minlength=3).astype(np.float64)
    weights = 1.0 / np.sqrt(np.maximum(frequencies, 1))
    weights = np.minimum(weights / weights.mean(), 3.0)
    model = TinyPixelNet().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.CrossEntropyLoss(weight=torch.tensor(weights, dtype=torch.float32, device=device))
    losses = []
    for epoch in range(8):
        model.train()
        indices = np.random.permutation(len(rgb))
        epoch_loss = 0.0
        for start in range(0, len(indices), 8):
            batch = indices[start:start + 8]
            x = rgb_input(rgb[batch], eye_ids[batch], device)
            y = torch.as_tensor(truth[batch].copy(), dtype=torch.long, device=device)
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(x), y)
            loss.backward()
            optimizer.step()
            epoch_loss += float(loss.detach()) * len(batch)
        losses.append(epoch_loss / len(rgb))
        print(f"fold={holdout} epoch={epoch + 1}/8 loss={losses[-1]:.4f}", flush=True)
    result = evaluate(model, datasets[testing_scenes[0]], device)
    result.update({"training_scenes": training_scenes, "held_out_scene": holdout,
                   "training_frames_by_eye": int(len(rgb)),
                   "training_class_counts": frequencies.astype(int).tolist(),
                   "loss_weights": weights.tolist(), "epoch_losses": losses})
    torch.save(model.cpu().state_dict(), output / f"model_holdout_{holdout}.pt")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "run_20261001")
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    device = torch.device("cuda" if args.device == "auto" and torch.cuda.is_available() else
                          "cpu" if args.device == "auto" else args.device)
    datasets = {scene: load_scene(scene) for scene in ("e0", "e1", "e2")}
    summary = {"protocol": "protocol.md", "device": str(device), "seed": SEED,
               "source_paths": {scene: [str(p) for p in scene_paths(scene)] for scene in datasets},
               "folds": {}}
    for holdout in ("e1", "e2"):
        summary["folds"][holdout] = train_fold(holdout, device, args.output, datasets)
        (args.output / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
    e1, e2 = summary["folds"]["e1"], summary["folds"]["e2"]
    summary["gate_passed"] = bool(
        all(fold["unknown_recall"] >= 0.90 and fold["free_false_obstacle_rate"] <= 0.05
            for fold in (e1, e2))
        and e1["central_wall_0_to_2s_obstacle_recall"] is not None
        and e1["central_wall_0_to_2s_obstacle_recall"] >= 0.80
    )
    (args.output / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"gate_passed": summary["gate_passed"],
                      "e1": {key: e1[key] for key in ("unknown_recall", "free_false_obstacle_rate", "central_wall_0_to_2s_obstacle_recall")},
                      "e2": {key: e2[key] for key in ("unknown_recall", "free_false_obstacle_rate", "central_ground_false_obstacle_rate")}}, indent=2))


if __name__ == "__main__":
    main()
