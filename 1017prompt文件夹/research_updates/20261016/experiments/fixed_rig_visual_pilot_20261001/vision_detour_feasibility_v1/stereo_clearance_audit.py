"""Audit fixed-rig stereo overlap; rendered depth is used only as offline truth."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pilot import CX, CY, FOCAL_PX, FREE, OBSTACLE, UNKNOWN, classify_depth_samples, sample_grid, yaw_from_quaternion


HERE = Path(__file__).resolve().parent
PILOT = HERE.parent
SEQUENCES = {
    "E0_direct": PILOT / "integrated_fill_e0b/depth_truth",
    "E1_direct": PILOT / "integrated_fill_e1/depth_truth",
    "E2_direct": PILOT / "integrated_fill_e2/depth_truth",
    "hard_0_E1_0003_direct": HERE / "bilateral_depth_h3/depth_truth",
    "E1_successful_detour": HERE / "bilateral_depth_detour_e1/depth_truth",
}
CLASSES = ("ground", "body_height_obstacle", "unknown")


def optical_pose(usd_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    pose = np.asarray(usd_world, dtype=np.float64)
    rotation = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
    return pose[:3, 3], rotation


def reproject_between_eyes(
    source_depth: np.ndarray,
    source_pose: np.ndarray,
    other_pose: np.ndarray,
    uv: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Map source pixels into the other image using *offline* source depth."""
    depth = np.asarray(source_depth, dtype=np.float64).reshape(-1)
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    if depth.shape != (len(uv),):
        raise ValueError("depth and UV must align")
    lo, lr = optical_pose(source_pose)
    ro, rr = optical_pose(other_pose)
    rays = np.column_stack(((uv[:, 0] - CX) / FOCAL_PX,
                            (uv[:, 1] - CY) / FOCAL_PX,
                            np.ones(len(uv))))
    valid = np.isfinite(depth) & (depth >= 0.45) & (depth <= 10.0)
    world = lo + (rays * np.where(valid, depth, 0)[:, None]) @ lr.T
    right_local = (world - ro) @ rr
    with np.errstate(divide="ignore", invalid="ignore"):
        right_uv = np.column_stack((CX + FOCAL_PX * right_local[:, 0] / right_local[:, 2],
                                    CY + FOCAL_PX * right_local[:, 1] / right_local[:, 2]))
    return right_uv, right_local[:, 2]


def frame_overlap(frame: np.lib.npyio.NpzFile, *, source_eye: str = "left",
                  absolute_tolerance_m: float = 0.10,
                  relative_tolerance: float = 0.05) -> dict:
    if source_eye not in ("left", "right"):
        raise ValueError("source_eye must be left or right")
    other_eye = "right" if source_eye == "left" else "left"
    uv = sample_grid()
    root = frame["root_position_world_m"]
    yaw = yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
    source_depth = frame[f"{source_eye}_depth_m"][::4, ::4].reshape(-1)
    labels, central = classify_depth_samples(source_depth, uv,
                                              frame[f"{source_eye}_camera_usd_world"], root, yaw)
    right_uv, expected_z = reproject_between_eyes(source_depth,
                                                    frame[f"{source_eye}_camera_usd_world"],
                                                    frame[f"{other_eye}_camera_usd_world"], uv)
    inside = (np.isfinite(expected_z) & (expected_z > 0.45)
              & (right_uv[:, 0] >= 0) & (right_uv[:, 0] < 640)
              & (right_uv[:, 1] >= 0) & (right_uv[:, 1] < 480))
    coords = np.rint(np.nan_to_num(right_uv, nan=-1, posinf=-1, neginf=-1)).astype(np.int64)
    coords[:, 0] = np.clip(coords[:, 0], 0, 639)
    coords[:, 1] = np.clip(coords[:, 1], 0, 479)
    observed = frame[f"{other_eye}_depth_m"][coords[:, 1], coords[:, 0]]
    # A matching right-eye depth means the surface is actually visible to both.
    same_surface = (inside & np.isfinite(observed) & (observed >= 0.45)
                    & (np.abs(observed - expected_z)
                       <= np.maximum(absolute_tolerance_m, relative_tolerance * expected_z)))
    # This RGB check is diagnostic only; it is not a stereo correspondence solver.
    left_rgb = frame[f"{source_eye}_rgb"][uv[:, 1], uv[:, 0]].astype(np.int16)
    right_rgb = frame[f"{other_eye}_rgb"][coords[:, 1], coords[:, 0]].astype(np.int16)
    rgb_difference = np.mean(np.abs(left_rgb - right_rgb), axis=1)
    output = {}
    for label, name in ((FREE, "ground"), (OBSTACLE, "body_height_obstacle"), (UNKNOWN, "unknown")):
        chosen = labels == label
        output[name] = {
            "source_pixels": int(chosen.sum()),
            "other_image_footprint": int((chosen & inside).sum()),
            "same_surface_in_both_depths": int((chosen & same_surface).sum()),
            "same_surface_rgb_mae_le_20": int((chosen & same_surface & (rgb_difference <= 20)).sum()),
        }
    frontal = (labels == OBSTACLE) & central
    output["central_obstacle"] = {
        "source_pixels": int(frontal.sum()),
        "other_image_footprint": int((frontal & inside).sum()),
        "same_surface_in_both_depths": int((frontal & same_surface).sum()),
        "same_surface_rgb_mae_le_20": int((frontal & same_surface & (rgb_difference <= 20)).sum()),
    }
    return output


def aggregate(folder: Path, *, source_eye: str = "left", absolute_tolerance_m: float = 0.10,
              relative_tolerance: float = 0.05) -> dict:
    paths = sorted(folder.glob("frame_*.npz"))
    if not paths:
        raise FileNotFoundError(folder)
    total = {name: {key: 0 for key in ("source_pixels", "other_image_footprint",
                                        "same_surface_in_both_depths", "same_surface_rgb_mae_le_20")}
             for name in (*CLASSES, "central_obstacle")}
    for path in paths:
        with np.load(path) as frame:
            result = frame_overlap(frame, source_eye=source_eye,
                                   absolute_tolerance_m=absolute_tolerance_m,
                                   relative_tolerance=relative_tolerance)
        for name, values in result.items():
            for key, value in values.items():
                total[name][key] += value
    for values in total.values():
        denominator = max(values["source_pixels"], 1)
        values["other_image_footprint_fraction"] = values["other_image_footprint"] / denominator
        values["same_surface_fraction"] = values["same_surface_in_both_depths"] / denominator
    return {"frames": len(paths), "pixel_totals": total}


def plot_e1_visibility() -> None:
    times = []
    visibility = {eye: [] for eye in ("left", "right")}
    footprints = {eye: [] for eye in ("left", "right")}
    for path in sorted(SEQUENCES["E1_direct"].glob("frame_*.npz")):
        with np.load(path) as frame:
            times.append(float(frame["simulation_time_s"]))
            for eye in visibility:
                row = frame_overlap(frame, source_eye=eye)["body_height_obstacle"]
                total = max(row["source_pixels"], 1)
                visibility[eye].append(row["same_surface_in_both_depths"] / total)
                footprints[eye].append(row["other_image_footprint"] / total)
    fig, ax = plt.subplots(figsize=(7.6, 4.1), dpi=160)
    for eye, color in (("left", "#2563eb"), ("right", "#ea580c")):
        ax.plot(times, footprints[eye], linestyle="--", color=color, alpha=0.65,
                label=f"{eye}: projects inside other image")
        ax.plot(times, visibility[eye], color=color, linewidth=2,
                label=f"{eye}: same obstacle surface visible")
    ax.set(xlabel="Time before E1 direct collision (s)",
           ylabel="Fraction of source-eye obstacle pixels", ylim=(-0.03, 1.03),
           title="Fixed vertical stereo: field overlap vs true co-visibility")
    ax.grid(alpha=0.2)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(HERE / "e1_stereo_obstacle_covisibility.png")
    plt.close(fig)


def main() -> None:
    report = {
        "method": "4-pixel source grid, source renderer depth projected into other camera; other renderer depth confirms co-visible surface",
        "renderer_depth_and_camera_poses_are_offline_truth_only": True,
        "same_surface_tolerance_m": "max(0.10, 0.05 * expected camera Z)",
        "sequences": {name: {eye: aggregate(folder, source_eye=eye) for eye in ("left", "right")}
                      for name, folder in SEQUENCES.items()},
        "looser_depth_tolerance_sensitivity": {
            name: {eye: aggregate(SEQUENCES[name], source_eye=eye,
                                  absolute_tolerance_m=0.30, relative_tolerance=0.10)
                   for eye in ("left", "right")}
            for name in ("E1_direct", "E1_successful_detour")
        },
    }
    (HERE / "stereo_clearance_audit.json").write_text(json.dumps(report, indent=2) + "\n")
    plot_e1_visibility()
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
