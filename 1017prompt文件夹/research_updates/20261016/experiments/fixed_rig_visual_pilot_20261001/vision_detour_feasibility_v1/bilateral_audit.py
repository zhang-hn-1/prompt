"""Reproduce the paired-eye online parity and offline depth-truth audit."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from scipy.spatial import cKDTree

from policy import MODEL, pilot


HERE = Path(__file__).resolve().parent
PILOT = HERE.parent
RUNS = (
    ("E0", "bilateral_shadow_golden_e0_0001", "e0_continuous", "golden_e0_0001"),
    ("E1", "bilateral_shadow_golden_e1_0001", "e1_continuous", "golden_e1_0001"),
    ("E2", "bilateral_shadow_e2", "e2_continuous", "golden_e2_0001"),
    ("hard_0 E1 0003", "bilateral_shadow_h3", "hard_0_e1_0003", "hard_0_e1_0003"),
    ("hard_0 E1 0004", "bilateral_shadow_h4", "hard_0_e1_0004", "hard_0_e1_0004"),
)
FRAMES = {
    "E0": PILOT / "integrated_fill_e0b/depth_truth/frame_000.npz",
    "E1": PILOT / "integrated_fill_e1/depth_truth/frame_000.npz",
    "E2": PILOT / "integrated_fill_e2/depth_truth/frame_000.npz",
    "hard_0 E1 0003": HERE / "bilateral_depth_h3/depth_truth/frame_000.npz",
}


def online_run(label: str, new: str, old: str, episode_id: str) -> dict:
    shadow = HERE / new / f"episode_{episode_id}"
    baseline = HERE / old / f"episode_{episode_id}"
    record_path = HERE / new / f"vision_records_{episode_id}.json"
    records = json.loads(record_path.read_text())["records"]
    metrics = json.loads((shadow / "metrics.json").read_text())
    old_metrics = json.loads((baseline / "metrics.json").read_text())
    environment = json.loads((shadow / "environment.json").read_text())
    identity = json.loads((HERE / new / "experiment_identity.json").read_text())
    with np.load(shadow / "actions.npz") as a, np.load(baseline / "actions.npz") as b:
        actions_equal = bool(np.array_equal(a["action_blocks"], b["action_blocks"]))
    with np.load(shadow / "states.npz") as a, np.load(baseline / "states.npz") as b:
        xy_equal = bool(np.array_equal(a["position_world_xy"], b["position_world_xy"]))
    assert actions_equal and xy_equal, label
    assert metrics["termination_reason"] == old_metrics["termination_reason"], label
    assert len(records) == metrics["action_blocks"], label
    assert all(record["left_count_consistent"] for record in records), label
    assert environment["episode_budget_s"] == 35.0, label
    assert environment["collision_mode"] == "physical_contact_v2", label
    assert identity["privileged"] is False and identity["camera_rig"] == "frozen", label

    def two_frame_trigger(combine):
        for index in range(1, len(records)):
            current = combine(records[index]["obstacle_pixels"], records[index]["right_obstacle_pixels"])
            prior = combine(records[index - 1]["obstacle_pixels"], records[index - 1]["right_obstacle_pixels"])
            if current >= 1000 and prior >= 1000:
                return index
        return None

    return {
        "episode_id": episode_id,
        "outcome": metrics["termination_reason"],
        "elapsed_s": metrics["elapsed_s"],
        "action_blocks": len(records),
        "actions_equal_to_frozen_controller": actions_equal,
        "xy_equal_to_frozen_controller": xy_equal,
        "first_left_obstacle_pixels": records[0]["obstacle_pixels"],
        "first_right_obstacle_pixels": records[0]["right_obstacle_pixels"],
        "right_above_old_left_threshold_blocks": sum(r["right_obstacle_pixels"] >= 1000 for r in records),
        "counterfactual_left_only_trigger_block": two_frame_trigger(lambda left, right: left),
        "counterfactual_either_eye_trigger_block": two_frame_trigger(max),
        "counterfactual_both_eyes_trigger_block": two_frame_trigger(min),
        "all_left_reconstructions_match": True,
    }


def projected_front_obstacle_count(frame: np.lib.npyio.NpzFile, eye: str) -> tuple[int, float | None]:
    uv = pilot.sample_grid()
    pose = frame[f"{eye}_camera_usd_world"]
    depth = frame[f"{eye}_depth_m"][::4, ::4].reshape(-1)
    rotation = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
    rays = np.column_stack(((uv[:, 0] - pilot.CX) / pilot.FOCAL_PX,
                            (uv[:, 1] - pilot.CY) / pilot.FOCAL_PX,
                            np.ones(len(uv))))
    valid_depth = np.isfinite(depth) & (depth >= 0.45) & (depth <= 10.0)
    hit = pose[:3, 3] + (rays * np.where(valid_depth, depth, 0)[:, None]) @ rotation.T
    root = frame["root_position_world_m"]
    yaw = pilot.yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
    dx, dy = hit[:, 0] - root[0], hit[:, 1] - root[1]
    forward = np.cos(yaw) * dx + np.sin(yaw) * dy
    lateral = -np.sin(yaw) * dx + np.cos(yaw) * dy
    front = (valid_depth & (np.linalg.norm(hit - root, axis=1) >= 0.55)
             & (forward >= 0.35) & (forward <= 4.5) & (np.abs(lateral) < 0.4)
             & (hit[:, 2] >= 0.25) & (hit[:, 2] <= 1.5))
    return int(front.sum()), float(forward[front].min()) if front.any() else None


def frame_truth(path: Path, model: pilot.TinyPixelNet) -> dict:
    result = {}
    uv = pilot.sample_grid()
    with np.load(path) as frame:
        root = frame["root_position_world_m"]
        yaw = pilot.yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
        for eye_id, eye in enumerate(("left", "right")):
            image = frame[f"{eye}_rgb"][::4, ::4][None]
            with torch.no_grad():
                pred = model(pilot.rgb_input(image, np.array([eye_id]), torch.device("cpu"))).argmax(1).numpy()[0]
            labels, _ = pilot.classify_depth_samples(
                frame[f"{eye}_depth_m"][::4, ::4], uv, frame[f"{eye}_camera_usd_world"], root, yaw,
            )
            label_roi = labels.reshape(120, 160)[40:100, 40:140]
            predicted = pred[40:100, 40:140] == pilot.OBSTACLE
            count, distance = projected_front_obstacle_count(frame, eye)
            result[eye] = {
                "predicted_obstacle_roi": int(predicted.sum()),
                "predicted_obstacle_over_truth_free": int((predicted & (label_roi == pilot.FREE)).sum()),
                "predicted_obstacle_over_truth_obstacle": int((predicted & (label_roi == pilot.OBSTACLE)).sum()),
                "predicted_obstacle_over_truth_unknown": int((predicted & (label_roi == pilot.UNKNOWN)).sum()),
                "actual_front_obstacle_points_0p35_to_4p5m": count,
                "nearest_actual_front_obstacle_m": distance,
            }
    return result


def e1_right_curve_visible_ground(model: pilot.TinyPixelNet) -> dict:
    """First-frame ground visibility, not a free-space or safety certificate."""
    predicted_ground, depth_ground = [], []
    uv = pilot.sample_grid()
    with np.load(FRAMES["E1"]) as frame:
        root = frame["root_position_world_m"]
        yaw = pilot.yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
        for eye_id, eye in enumerate(("left", "right")):
            image = frame[f"{eye}_rgb"][::4, ::4][None]
            with torch.no_grad():
                pred = model(pilot.rgb_input(image, np.array([eye_id]), torch.device("cpu"))).argmax(1).numpy()[0].reshape(-1)
            pose = frame[f"{eye}_camera_usd_world"]
            rotation = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
            rays = np.column_stack(((uv[:, 0] - pilot.CX) / pilot.FOCAL_PX,
                                    (uv[:, 1] - pilot.CY) / pilot.FOCAL_PX,
                                    np.ones(len(uv)))) @ rotation.T
            with np.errstate(divide="ignore", invalid="ignore"):
                distance = -pose[2, 3] / rays[:, 2]
            valid = np.isfinite(distance) & (distance > 0.3) & (distance < 8.0)
            hit = pose[:3, 3] + rays * np.where(valid, distance, 0)[:, None]
            dx, dy = hit[:, 0] - root[0], hit[:, 1] - root[1]
            points = np.column_stack((np.cos(yaw) * dx + np.sin(yaw) * dy,
                                      -np.sin(yaw) * dx + np.cos(yaw) * dy))
            labels, _ = pilot.classify_depth_samples(
                frame[f"{eye}_depth_m"][::4, ::4], uv, pose, root, yaw,
            )
            predicted_ground.append(points[valid & (pred == pilot.FREE)])
            depth_ground.append(points[valid & (labels == pilot.FREE)])

    forward = np.arange(0.5, 4.51, 0.25)
    fraction = np.clip(forward / 3.0, 0, 1)
    right_offset = 2.3 * fraction**2 * (3 - 2 * fraction)
    candidate = np.column_stack((forward, -right_offset))
    predicted_points = np.concatenate(predicted_ground)
    depth_points = np.concatenate(depth_ground)
    return {
        "route": "first 0.5-4.5m of frozen 2.3m-right-offset smoothstep",
        "sample_spacing_m": 0.25,
        "visible_if_ground_point_within_m": 0.35,
        "samples": len(candidate),
        "rgb_predicted_free_visible_samples": int((cKDTree(predicted_points).query(candidate)[0] < 0.35).sum()),
        "rendered_depth_ground_visible_samples": int((cKDTree(depth_points).query(candidate)[0] < 0.35).sum()),
        "interpretation": "visible floor samples only; absence can be occlusion or off-camera, not obstacle",
    }


def main() -> None:
    torch.set_num_threads(2)
    model = pilot.TinyPixelNet().eval()
    model.load_state_dict(torch.load(MODEL, map_location="cpu", weights_only=True))
    output = {
        "scope": "online read-only bilateral shadow; offline rendered depth only for label audit",
        "model_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        "online_runs": {label: online_run(label, new, old, episode_id)
                        for label, new, old, episode_id in RUNS},
        "offline_frame_zero_truth": {label: frame_truth(path, model) for label, path in FRAMES.items()},
        "e1_candidate_curve_first_frame_ground_visibility": e1_right_curve_visible_ground(model),
    }
    (HERE / "bilateral_audit.json").write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
