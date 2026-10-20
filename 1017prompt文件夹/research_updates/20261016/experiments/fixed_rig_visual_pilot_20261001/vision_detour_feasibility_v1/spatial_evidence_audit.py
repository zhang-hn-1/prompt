"""Depth-free binocular ground projection, checked with offline depth and map truth."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch
from scipy.spatial import cKDTree

from policy import MODEL, pilot
from spatial_projection import ROOT_HEIGHT_M, camera_pose_from_root, project_ground_pixels


HERE = Path(__file__).resolve().parent
PILOT = HERE.parent
FRAME_SETS = {
    "E0_direct": PILOT / "integrated_fill_e0b/depth_truth",
    "E1_direct": PILOT / "integrated_fill_e1/depth_truth",
    "E2_direct": PILOT / "integrated_fill_e2/depth_truth",
    "hard_0_E1_0003_direct": HERE / "bilateral_depth_h3/depth_truth",
    "E1_successful_detour": HERE / "bilateral_depth_detour_e1/depth_truth",
}
FREE_CONFIDENCE = 0.8


def model_free_masks(model: pilot.TinyPixelNet, frame) -> np.ndarray:
    images = np.stack((frame["left_rgb"][::4, ::4], frame["right_rgb"][::4, ::4]))
    with torch.no_grad():
        free_prob = model(pilot.rgb_input(images, np.array([0, 1]), torch.device("cpu"))).softmax(1).numpy()[:, 0]
    return free_prob >= FREE_CONFIDENCE


def static_rig_error(frame, eye: str) -> tuple[float, float, float]:
    root = frame["root_position_world_m"]
    quaternion = frame["root_quaternion_world_wxyz"]
    measured = frame[f"{eye}_camera_usd_world"]
    origin_exact, optical = camera_pose_from_root(root[:2], quaternion, eye, root_height_m=float(root[2]))
    origin_assumed, _ = camera_pose_from_root(root[:2], quaternion, eye)
    measured_optical = np.column_stack((measured[:3, 0], -measured[:3, 1], -measured[:3, 2]))
    return (float(np.linalg.norm(origin_exact - measured[:3, 3])),
            float(np.max(np.abs(optical - measured_optical))),
            float(np.linalg.norm(origin_assumed - measured[:3, 3])))


def scene_pixel_audit(model: pilot.TinyPixelNet, folder: Path) -> dict:
    paths = sorted(folder.glob("frame_*.npz"))
    if not paths:
        raise FileNotFoundError(folder)
    truth_counts = np.zeros(3, dtype=np.int64)
    calibration_errors = np.zeros(3)
    uv = pilot.sample_grid()
    for path in paths:
        with np.load(path) as frame:
            masks = model_free_masks(model, frame)
            root = frame["root_position_world_m"]
            quaternion = frame["root_quaternion_world_wxyz"]
            yaw = pilot.yaw_from_quaternion(quaternion)
            for eye_id, eye in enumerate(("left", "right")):
                labels, _ = pilot.classify_depth_samples(
                    frame[f"{eye}_depth_m"][::4, ::4], uv,
                    frame[f"{eye}_camera_usd_world"], root, yaw,
                )
                _, projected_valid = project_ground_pixels(masks[eye_id], root[:2], quaternion, eye)
                selected = labels[masks[eye_id].reshape(-1)][projected_valid]
                truth_counts += np.bincount(selected, minlength=3)[:3]
                calibration_errors = np.maximum(calibration_errors, static_rig_error(frame, eye))
    return {
        "complete_sequence_frames": len(paths),
        "projected_model_free_pixel_truth_free_obstacle_unknown": truth_counts.tolist(),
        "known_ground_precision": float(truth_counts[0] / max(truth_counts[0] + truth_counts[1], 1)),
        "max_camera_origin_error_with_measured_root_z_m": calibration_errors[0],
        "max_optical_rotation_element_error": calibration_errors[1],
        "max_camera_origin_error_with_online_fixed_root_z_m": calibration_errors[2],
    }


def e1_direct_time_bins(model: pilot.TinyPixelNet) -> dict:
    """Expose late close-wall free-space hallucinations separately from start."""
    bins = {"0_to_2_s": np.zeros(3, dtype=np.int64),
            "2_to_4_s": np.zeros(3, dtype=np.int64),
            "4_to_6_s": np.zeros(3, dtype=np.int64)}
    uv = pilot.sample_grid()
    for path in sorted(FRAME_SETS["E1_direct"].glob("frame_*.npz")):
        with np.load(path) as frame:
            time_s = float(frame["simulation_time_s"])
            key = "0_to_2_s" if time_s < 2 else "2_to_4_s" if time_s < 4 else "4_to_6_s"
            root = frame["root_position_world_m"]
            quaternion = frame["root_quaternion_world_wxyz"]
            yaw = pilot.yaw_from_quaternion(quaternion)
            masks = model_free_masks(model, frame)
            for eye_id, eye in enumerate(("left", "right")):
                _, valid = project_ground_pixels(masks[eye_id], root[:2], quaternion, eye)
                labels, _ = pilot.classify_depth_samples(
                    frame[f"{eye}_depth_m"][::4, ::4], uv,
                    frame[f"{eye}_camera_usd_world"], root, yaw,
                )
                bins[key] += np.bincount(labels[masks[eye_id].reshape(-1)][valid], minlength=3)[:3]
    return {key: {"free_obstacle_unknown": values.tolist(),
                  "known_ground_precision": float(values[0] / max(values[0] + values[1], 1))}
            for key, values in bins.items()}


def frozen_e1_curve() -> tuple[np.ndarray, np.ndarray]:
    records = json.loads((HERE / "bilateral_depth_detour_e1/vision_records_golden_e1_0001.json").read_text())["records"]
    with np.load(HERE / "bilateral_depth_detour_e1/episode_golden_e1_0001/states.npz") as state:
        trigger = next(i for i, record in enumerate(records) if record["detour_triggered"])
        yaw = float(state["yaw_world_rad"][trigger])
    record = records[trigger]
    start = np.asarray(record["position_world_xy"], dtype=np.float64)
    goal_base = np.asarray(record["goal_base_xy"], dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    goal = start + np.array([c * goal_base[0] - s * goal_base[1],
                             s * goal_base[0] + c * goal_base[1]])
    delta = goal - start
    distance = np.linalg.norm(delta)
    forward = delta / distance
    right = np.array([forward[1], -forward[0]])
    x = np.append(np.arange(0, distance, 0.05), distance)
    entering = np.clip(x / 3.0, 0, 1)
    leaving = np.clip((x - (distance - 2.6)) / 2.6, 0, 1)
    offset = np.where(x <= 3.0, 2.3 * entering**2 * (3 - 2 * entering),
                      np.where(x < distance - 2.6, 2.3,
                               2.3 * (1 - leaving**2 * (3 - 2 * leaving))))
    path = start + x[:, None] * forward + offset[:, None] * right
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    return path, arc


def e1_temporal_coverage(model: pilot.TinyPixelNet) -> list[dict]:
    path, arc = frozen_e1_curve()
    model_points: list[np.ndarray] = []
    true_ground_points: list[np.ndarray] = []
    result = []
    sample_times = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14}
    uv = pilot.sample_grid()
    for frame_path in sorted(FRAME_SETS["E1_successful_detour"].glob("frame_*.npz")):
        with np.load(frame_path) as frame:
            root = frame["root_position_world_m"]
            quaternion = frame["root_quaternion_world_wxyz"]
            yaw = pilot.yaw_from_quaternion(quaternion)
            masks = model_free_masks(model, frame)
            for eye_id, eye in enumerate(("left", "right")):
                predicted, _ = project_ground_pixels(masks[eye_id], root[:2], quaternion, eye)
                if len(predicted):
                    model_points.append(predicted)
                labels, _ = pilot.classify_depth_samples(
                    frame[f"{eye}_depth_m"][::4, ::4], uv,
                    frame[f"{eye}_camera_usd_world"], root, yaw,
                )
                true_ground, _ = project_ground_pixels(
                    (labels == pilot.FREE).reshape(120, 160), root[:2], quaternion, eye,
                )
                if len(true_ground):
                    true_ground_points.append(true_ground)
            time_s = float(frame["simulation_time_s"])
            if round(time_s) not in sample_times or abs(time_s - round(time_s)) > 0.03:
                continue
            nearest = int(np.argmin(np.linalg.norm(path - root[:2], axis=1)))
            ahead_arc = np.arange(arc[nearest] + 0.25, min(arc[nearest] + 2.01, arc[-1]), 0.25)
            if not len(ahead_arc):
                continue
            ahead = path[np.searchsorted(arc, ahead_arc)]
            predicted_visible = int((cKDTree(np.concatenate(model_points)).query(ahead)[0] < 0.35).sum())
            truth_visible = int((cKDTree(np.concatenate(true_ground_points)).query(ahead)[0] < 0.35).sum())
            result.append({"time_s": round(time_s, 1), "ahead_curve_samples": len(ahead),
                           "rgb_free_visible_samples": predicted_visible,
                           "depth_ground_visible_samples": truth_visible})
    return result


def true_ground_overlap_with_bodyband() -> dict:
    """Show why even perfect floor pixels do not certify ball-body clearance."""
    map_path = (Path("/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark")
                / "research/ball_wm/data/benchmark_v1/occupancy/e1_detour_bodyband.npz")
    with np.load(map_path) as occupancy:
        grid = occupancy["grid"]
        origin = occupancy["global_origin_xy"]
        resolution = float(occupancy["resolution_m"])
    uv = pilot.sample_grid()
    total, occupied = 0, 0
    for frame_path in sorted(FRAME_SETS["E1_successful_detour"].glob("frame_*.npz")):
        with np.load(frame_path) as frame:
            root = frame["root_position_world_m"]
            yaw = pilot.yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
            for eye in ("left", "right"):
                pose = frame[f"{eye}_camera_usd_world"]
                depth = frame[f"{eye}_depth_m"][::4, ::4].reshape(-1)
                labels, _ = pilot.classify_depth_samples(depth, uv, pose, root, yaw)
                optical = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
                rays = np.column_stack(((uv[:, 0] - pilot.CX) / pilot.FOCAL_PX,
                                        (uv[:, 1] - pilot.CY) / pilot.FOCAL_PX,
                                        np.ones(len(uv))))
                hit = pose[:3, 3] + (rays * np.where(np.isfinite(depth), depth, 0)[:, None]) @ optical.T
                cell = np.rint((hit[labels == pilot.FREE, :2] - origin) / resolution).astype(int)
                inside = ((cell[:, 0] >= 0) & (cell[:, 0] < grid.shape[0])
                          & (cell[:, 1] >= 0) & (cell[:, 1] < grid.shape[1]))
                cell = cell[inside]
                total += len(cell)
                occupied += int((grid[cell[:, 0], cell[:, 1]] == 1).sum())
    return {"true_floor_hits_inside_map": total,
            "true_floor_hits_at_bodyband_occupied_xy": occupied,
            "overlap_fraction": occupied / total,
            "caution": "ground surface visible at XY does not imply ball-body clearance at that XY"}


def plot_coverage(records: list[dict]) -> None:
    fig, ax = plt.subplots(figsize=(7.4, 4.2), dpi=160)
    times = [x["time_s"] for x in records]
    rgb = [x["rgb_free_visible_samples"] / x["ahead_curve_samples"] for x in records]
    truth = [x["depth_ground_visible_samples"] / x["ahead_curve_samples"] for x in records]
    ax.plot(times, rgb, "o-", color="#2563eb", label="Accumulated RGB free evidence")
    ax.plot(times, truth, "s--", color="#f97316", label="Accumulated true ground visibility")
    ax.set(xlabel="Elapsed time (s)", ylabel="Fraction of next 2 m curve observed as ground",
           ylim=(-0.03, 1.05), title="Golden E1: past and current images only")
    ax.grid(alpha=0.2)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(HERE / "e1_temporal_ground_coverage.png")


def main() -> None:
    torch.set_num_threads(2)
    model = pilot.TinyPixelNet().eval()
    model.load_state_dict(torch.load(MODEL, map_location="cpu", weights_only=True))
    output = {
        "online_geometry_inputs": ["left_rgb", "right_rgb", "position_world_xy", "current_root_quaternion"],
        "offline_truth_only": ["rendered_depth", "camera_usd_world", "bodyband_occupancy"],
        "fixed_root_height_m": ROOT_HEIGHT_M,
        "free_confidence_threshold": FREE_CONFIDENCE,
        "scene_pixel_audit": {name: scene_pixel_audit(model, folder) for name, folder in FRAME_SETS.items()},
        "e1_direct_free_prediction_by_time": e1_direct_time_bins(model),
        "e1_successful_detour_temporal_next_2m_coverage": e1_temporal_coverage(model),
        "e1_successful_detour_true_floor_bodyband_overlap": true_ground_overlap_with_bodyband(),
    }
    (HERE / "spatial_evidence_audit.json").write_text(json.dumps(output, indent=2) + "\n")
    plot_coverage(output["e1_successful_detour_temporal_next_2m_coverage"])
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    sys.exit(main())
