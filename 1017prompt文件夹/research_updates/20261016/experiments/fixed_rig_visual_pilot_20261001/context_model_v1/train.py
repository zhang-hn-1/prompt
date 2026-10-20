"""One preregistered context-segmentation candidate on fixed-rig lit RGB."""

from __future__ import annotations

import json
import random
import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F


HERE = Path(__file__).resolve().parent
PILOT = HERE.parent
sys.path.insert(0, str(PILOT))
import pilot  # noqa: E402


SCENES = {
    "e0": PILOT / "integrated_fill_e0b" / "depth_truth",
    "e1": PILOT / "integrated_fill_e1" / "depth_truth",
    "e2": PILOT / "integrated_fill_e2" / "depth_truth",
    "h1": PILOT / "illuminated_replay_v1" / "hard_0_e1_0001" / "depth_truth",
    "h2": PILOT / "illuminated_replay_v1" / "hard_0_e1_0002" / "depth_truth",
}


def paths(scene: str) -> list[Path]:
    result = sorted(SCENES[scene].glob("frame_*.npz"))
    if not result:
        raise FileNotFoundError(SCENES[scene])
    return result


def block(a: int, b: int) -> nn.Sequential:
    return nn.Sequential(
        nn.Conv2d(a, b, 3, padding=1), nn.GroupNorm(8, b), nn.ReLU(),
        nn.Conv2d(b, b, 3, padding=1), nn.GroupNorm(8, b), nn.ReLU(),
    )


class ContextNet(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.e0 = block(6, 32)
        self.e1 = block(32, 64)
        self.e2 = block(64, 128)
        self.d1 = block(192, 64)
        self.d0 = block(96, 32)
        self.head = nn.Conv2d(32, 3, 1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        s0 = self.e0(x)
        s1 = self.e1(F.avg_pool2d(s0, 2))
        z = self.e2(F.avg_pool2d(s1, 2))
        z = self.d1(torch.cat((F.interpolate(z, size=s1.shape[-2:], mode="bilinear", align_corners=False), s1), 1))
        z = self.d0(torch.cat((F.interpolate(z, size=s0.shape[-2:], mode="bilinear", align_corners=False), s0), 1))
        return self.head(z)


def augment_rgb(x: torch.Tensor) -> torch.Tensor:
    n = len(x)
    gamma = torch.empty((n, 1, 1, 1), device=x.device).uniform_(0.7, 1.4)
    gain = torch.empty((n, 1, 1, 1), device=x.device).uniform_(0.8, 1.2)
    bias = torch.empty((n, 1, 1, 1), device=x.device).uniform_(-0.1, 0.1)
    image = torch.clamp(x[:, :3].pow(gamma) * gain + bias, 0, 1)
    return torch.cat((image, x[:, 3:]), dim=1)


def train_fold(holdout: str, datasets: dict, device: torch.device) -> dict:
    pilot.set_seed()
    training = sorted(set(SCENES) - {holdout})
    rgb = np.concatenate([datasets[s]["rgb"] for s in training])
    labels = np.concatenate([datasets[s]["labels"] for s in training])
    eye_ids = np.concatenate([np.arange(len(datasets[s]["rgb"])) % 2 for s in training])
    frequencies = np.bincount(labels.reshape(-1), minlength=3).astype(np.float64)
    weights = 1 / np.sqrt(np.maximum(frequencies, 1))
    weights = np.minimum(weights / weights.mean(), 3.0)
    model = ContextNet().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.CrossEntropyLoss(weight=torch.as_tensor(weights, dtype=torch.float32, device=device))
    losses = []
    for epoch in range(12):
        model.train()
        permutation = np.random.permutation(len(rgb))
        total = 0.0
        for start in range(0, len(permutation), 8):
            idx = permutation[start:start + 8]
            x = augment_rgb(pilot.rgb_input(rgb[idx], eye_ids[idx], device))
            y = torch.as_tensor(labels[idx].copy(), dtype=torch.long, device=device)
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(x), y)
            loss.backward()
            optimizer.step()
            total += float(loss.detach()) * len(idx)
        losses.append(total / len(rgb))
        print(f"{holdout} epoch {epoch + 1}/12 loss={losses[-1]:.4f}", flush=True)
    result = pilot.evaluate(model, datasets[holdout], device)
    result.update({"training_scenes": training, "held_out_scene": holdout,
                   "training_frames_by_eye": len(rgb),
                   "training_class_counts": frequencies.astype(int).tolist(),
                   "loss_weights": weights.tolist(), "epoch_losses": losses})
    torch.save(model.cpu().state_dict(), HERE / f"model_holdout_{holdout}.pt")
    return result


def main() -> None:
    random.seed(pilot.SEED)
    np.random.seed(pilot.SEED)
    torch.manual_seed(pilot.SEED)
    torch.set_num_threads(4)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    pilot.scene_paths = paths
    datasets = {s: pilot.load_scene(s) for s in SCENES}
    summary = {"protocol": str(HERE / "protocol.md"), "device": str(device),
               "seed": pilot.SEED,
               "source_paths": {s: [str(p) for p in paths(s)] for s in SCENES},
               "folds": {}}
    for holdout in ("e1", "e2"):
        summary["folds"][holdout] = train_fold(holdout, datasets, device)
        (HERE / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
    a, b = summary["folds"]["e1"], summary["folds"]["e2"]
    summary["gate_passed"] = bool(
        all(x["unknown_recall"] >= .90 and x["free_false_obstacle_rate"] <= .05 for x in (a, b))
        and a["central_wall_0_to_2s_obstacle_recall"] >= .80
    )
    (HERE / "results.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"gate_passed": summary["gate_passed"],
                      "e1_wall": a["central_wall_0_to_2s_obstacle_recall"],
                      "e1_unknown": a["unknown_recall"],
                      "e1_free_false_obstacle": a["free_false_obstacle_rate"],
                      "e2_unknown": b["unknown_recall"],
                      "e2_free_false_obstacle": b["free_false_obstacle_rate"]}, indent=2))


if __name__ == "__main__":
    main()
