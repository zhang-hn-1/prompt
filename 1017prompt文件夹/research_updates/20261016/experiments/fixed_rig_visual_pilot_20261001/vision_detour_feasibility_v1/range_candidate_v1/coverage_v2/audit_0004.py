"""Frozen V2 inference on a newly captured whole episode, after B0 parity check."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import torch

from range_pilot import (FREE, OBSTACLE, RangeNet, e1_gate, evaluate,
                         joint_near_obstacle, load_sequence, predict)


HERE = Path(__file__).resolve().parent
CAPTURE = HERE.parents[1]
NEW = CAPTURE / "bilateral_depth_h4"
OLD = CAPTURE / "bilateral_shadow_h4"
EPISODE = "episode_hard_0_e1_0004"


def retrospective_height_check(model: RangeNet, data: dict) -> dict:
    """Diagnostic only: does a fixed world-height gate rescue the frozen model?"""
    sys.path.insert(0, str(CAPTURE))
    from spatial_projection import CX, CY, FOCAL_PX, camera_pose_from_root

    _, obstacle_probability, predicted_depth = predict(model, data, torch.device("cpu"))
    rows, cols = np.mgrid[0:480:4, 0:640:4]
    rays = np.stack(((cols - CX) / FOCAL_PX, (rows - CY) / FOCAL_PX,
                     np.ones_like(rows)), axis=-1)
    height = []
    for path in sorted((NEW / "depth_truth").glob("frame_*.npz")):
        with np.load(path) as frame:
            for eye in ("left", "right"):
                origin, optical_to_world = camera_pose_from_root(
                    frame["root_position_world_m"][:2],
                    frame["root_quaternion_world_wxyz"], eye)
                image_index = len(height)
                height.append(origin[2] + predicted_depth[image_index]
                              * (rays @ optical_to_world.T)[..., 2])
    height = np.stack(height)
    raw = joint_near_obstacle(obstacle_probability, predicted_depth)
    gated = raw & (height >= 0.25) & (height <= 1.50)
    truth_near = ((data["labels"] == OBSTACLE) & data["central"]
                  & np.isfinite(data["depth"]) & (data["depth"] <= 2.0))
    ground = (data["labels"] == FREE) & data["central"]
    result = {}
    for name, frame_mask in (("both", np.ones(len(data["eye_ids"]), dtype=bool)),
                             ("left", data["eye_ids"] == 0),
                             ("right", data["eye_ids"] == 1)):
        near = truth_near & frame_mask[:, None, None]
        floor = ground & frame_mask[:, None, None]
        result[name] = {
            "true_near_pixels": int(near.sum()),
            "known_ground_pixels": int(floor.sum()),
            "raw_near_recall": float(raw[near].mean()) if near.any() else None,
            "height_gated_near_recall": float(gated[near].mean()) if near.any() else None,
            "raw_ground_false_alarm": float(raw[floor].mean()) if floor.any() else None,
            "height_gated_ground_false_alarm": float(gated[floor].mean()) if floor.any() else None,
        }
    return {"status": "retrospective diagnostic, not a new accepted control gate",
            "height_band_m": [0.25, 1.50], "root_height_assumption_m": 0.40,
            "by_eye": result}


def main() -> None:
    parity = {}
    for filename, field in (("actions.npz", "action_blocks"),
                            ("states.npz", "position_world_xy")):
        with np.load(NEW / EPISODE / filename) as captured, np.load(OLD / EPISODE / filename) as original:
            parity[field] = {"equal": bool(np.array_equal(captured[field], original[field])),
                             "shape": list(captured[field].shape)}
    new_metrics = json.loads((NEW / EPISODE / "metrics.json").read_text())
    old_metrics = json.loads((OLD / EPISODE / "metrics.json").read_text())
    parity["same_termination"] = new_metrics["termination_reason"] == old_metrics["termination_reason"]
    parity["same_elapsed_s"] = new_metrics["elapsed_s"] == old_metrics["elapsed_s"]
    if not all(item["equal"] for item in (parity["action_blocks"], parity["position_world_xy"])) \
            or not parity["same_termination"] or not parity["same_elapsed_s"]:
        raise AssertionError("depth side channel changed baseline episode behavior")

    torch.set_num_threads(2)
    model = RangeNet().eval()
    model.load_state_dict(torch.load(HERE / "range_model.pt", map_location="cpu", weights_only=True))
    data = load_sequence(NEW / "depth_truth")
    metrics = evaluate(model, data, torch.device("cpu"))
    gate = e1_gate(metrics)
    gate["rejection_reasons"] = [item.replace("E1", "hard_0 E1 0004") for item in gate["rejection_reasons"]]
    result = {
        "protocol": str(HERE / "episode_0004_protocol.md"),
        "episode": "hard_0_e1_0004",
        "frozen_checkpoint": str(HERE / "range_model.pt"),
        "depth_role": "offline labels only",
        "capture_matches_existing_frozen_controller": parity,
        "navigation_termination": new_metrics["termination_reason"],
        "navigation_elapsed_s": new_metrics["elapsed_s"],
        "measurement": metrics,
        "control_gate": gate,
        "retrospective_height_check": retrospective_height_check(model, data),
    }
    (HERE / "episode_0004_results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
