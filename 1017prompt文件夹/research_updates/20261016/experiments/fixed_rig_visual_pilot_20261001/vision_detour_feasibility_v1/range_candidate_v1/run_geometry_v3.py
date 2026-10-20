"""One bounded all-known-surface depth fit with fixed geometric height check."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

from range_pilot import (FREE, OBSTACLE, SEQUENCES, evaluate, joint_near_obstacle,
                         load_sequence, predict, train_one_model)
from run_coverage_v2 import FIT
from spatial_projection import CX, CY, FOCAL_PX, camera_pose_from_root


HERE = Path(__file__).resolve().parent
OUTPUT = HERE / "geometry_v3"
TEST = {
    "hard_0_E1_0003": SEQUENCES["hard_0_E1_0003_historical_test"],
    "hard_0_E1_0004": HERE.parent / "bilateral_depth_h4/depth_truth",
}


def predicted_world_height(frame_folder: Path, predicted_depth: np.ndarray) -> np.ndarray:
    rows, cols = np.mgrid[0:480:4, 0:640:4]
    rays = np.stack(((cols - CX) / FOCAL_PX, (rows - CY) / FOCAL_PX,
                     np.ones_like(rows)), axis=-1)
    result = []
    for path in sorted(frame_folder.glob("frame_*.npz")):
        with np.load(path) as frame:
            for eye in ("left", "right"):
                origin, rotation = camera_pose_from_root(
                    frame["root_position_world_m"][:2],
                    frame["root_quaternion_world_wxyz"], eye)
                index = len(result)
                result.append(origin[2] + predicted_depth[index]
                              * (rays @ rotation.T)[..., 2])
    if len(result) != len(predicted_depth):
        raise ValueError("predicted depth must align with complete stereo sequence")
    return np.stack(result)


def evaluate_height_gated(model, data: dict, folder: Path) -> dict:
    _, obstacle_probability, predicted_depth = predict(model, data, torch.device("cpu"))
    height = predicted_world_height(folder, predicted_depth)
    detected = (joint_near_obstacle(obstacle_probability, predicted_depth)
                & (height >= 0.25) & (height <= 1.50))
    truth = data["labels"]
    near = ((truth == OBSTACLE) & data["central"] & np.isfinite(data["depth"])
            & (data["depth"] <= 2.0))
    ground = (truth == FREE) & data["central"]
    def recall(mask: np.ndarray) -> dict:
        n = int(mask.sum())
        return {"true_near_obstacle_pixels": n,
                "joint_near_obstacle_recall": float(detected[mask].mean()) if n else None}
    time = data["times"][:, None, None]
    by_stage = {
        "0_to_2_s": recall(near & (time < 2)),
        "2_to_4_s": recall(near & (time >= 2) & (time < 4)),
        "4_to_6_s": recall(near & (time >= 4) & (time < 6)),
    }
    return {
        "complete_frames": data["complete_frames"],
        "near_obstacle": recall(near),
        "near_obstacle_by_stage": by_stage,
        "known_ground_central_pixels": int(ground.sum()),
        "known_ground_false_near_obstacle_rate": float(detected[ground].mean()) if ground.any() else None,
        "by_eye": {
            eye_name: {
                "true_near_obstacle_pixels": int((near & (data["eye_ids"] == eye_id)[:, None, None]).sum()),
                "joint_near_obstacle_recall": float(detected[near & (data["eye_ids"] == eye_id)[:, None, None]].mean())
                if (near & (data["eye_ids"] == eye_id)[:, None, None]).any() else None,
                "known_ground_pixels": int((ground & (data["eye_ids"] == eye_id)[:, None, None]).sum()),
                "known_ground_false_near_obstacle_rate": float(detected[ground & (data["eye_ids"] == eye_id)[:, None, None]].mean())
                if (ground & (data["eye_ids"] == eye_id)[:, None, None]).any() else None,
            } for eye_id, eye_name in ((0, "left"), (1, "right"))
        },
    }


def gate(metrics: dict, raw: dict) -> dict:
    reasons = []
    total = metrics["near_obstacle"]
    if total["true_near_obstacle_pixels"] < 100 or (total["joint_near_obstacle_recall"] or 0) < 0.85:
        reasons.append("complete near-obstacle recall below 0.85")
    for name, stage in metrics["near_obstacle_by_stage"].items():
        if stage["true_near_obstacle_pixels"] >= 100 and (stage["joint_near_obstacle_recall"] or 0) < 0.80:
            reasons.append(f"{name} near-obstacle recall below 0.80")
    if metrics["known_ground_false_near_obstacle_rate"] is None or metrics["known_ground_false_near_obstacle_rate"] > 0.05:
        reasons.append("ground false near-obstacle rate above 0.05")
    if raw["range_median_absolute_error_m"] is None or raw["range_median_absolute_error_m"] > 0.50:
        reasons.append("median obstacle range error above 0.50 m")
    if raw["range_p90_absolute_error_m"] is None or raw["range_p90_absolute_error_m"] > 1.50:
        reasons.append("p90 obstacle range error above 1.50 m")
    return {"passed": not reasons, "rejection_reasons": reasons}


def main() -> None:
    torch.set_num_threads(2)
    OUTPUT.mkdir(exist_ok=True)
    data = {name: load_sequence(folder) for name, folder in FIT.items()}
    training = {key: np.concatenate([item[key] for item in data.values()])
                for key in ("rgb", "labels", "depth", "central", "times", "eye_ids")}
    model, losses = train_one_model(training, torch.device("cpu"), depth_supervision="all_known")
    tests = {}
    for name, folder in TEST.items():
        sequence = load_sequence(folder)
        raw = evaluate(model, sequence, torch.device("cpu"))
        height_gated = evaluate_height_gated(model, sequence, folder)
        tests[name] = {"raw_rgb": raw, "fixed_height_gate": height_gated,
                       "control_gate": gate(height_gated, raw)}
    result = {
        "protocol": str(HERE / "geometry_v3_protocol.md"),
        "same_fit_sequences_as_v2": {name: item["complete_frames"] for name, item in data.items()},
        "depth_supervision": "all known ground and obstacle pixels",
        "range_supervision_pixels": int(((training["labels"] == FREE)
                                          | (training["labels"] == OBSTACLE)).sum()),
        "unchanged_architecture_class_loss_seed_epochs_and_thresholds": True,
        "training_losses": losses,
        "whole_sequence_historical_tests": tests,
        "accepted_for_control": all(item["control_gate"]["passed"] for item in tests.values()),
    }
    torch.save(model.cpu().state_dict(), OUTPUT / "range_model.pt")
    (OUTPUT / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
