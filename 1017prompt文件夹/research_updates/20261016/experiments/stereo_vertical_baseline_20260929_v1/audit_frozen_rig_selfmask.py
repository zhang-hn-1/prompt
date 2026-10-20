"""Offline self-occlusion check for the unchanged hardware camera positions."""

import json
from pathlib import Path

import numpy as np

from vertical_depth import vertical_stereo_depth


ROOT = Path(__file__).resolve().parent


def truth_frames(scene: str):
    folder = ROOT / f"depth_audit_control_golden_{scene}_0001" / "depth_truth"
    return [np.load(path) for path in sorted(folder.glob("frame_*.npz"))]


def main() -> None:
    training = truth_frames("e0")
    if not training:
        raise RuntimeError("E0 camera calibration samples missing")
    result = {
        "rig_positions_base_link_m": [[0.1, 0.3, 0.1], [0.1, -0.3, 0.1]],
        "self_mask_source": "union of E0 rendered depth below 0.45 m; offline diagnosis only",
        "evaluation_data_status": "existing E1/E2 diagnostics already seen during development; not fresh held-out",
        "cameras": {},
    }
    for camera in ("left", "right"):
        masks = [
            np.isfinite(frame[f"{camera}_depth_m"]) & (frame[f"{camera}_depth_m"] < 0.45)
            for frame in training
        ]
        static_mask = np.logical_or.reduce(masks)
        camera_result = {
            "e0_union_mask_fraction": float(static_mask.mean()),
            "e0_all_frames_identical": all(np.array_equal(mask, static_mask) for mask in masks),
            "scenes": {},
        }
        for scene in ("e1", "e2"):
            samples = []
            for frame in truth_frames(scene):
                truth = frame[f"{camera}_depth_m"]
                actual_self = np.isfinite(truth) & (truth > 0) & (truth < 0.45)
                depth, valid = vertical_stereo_depth(
                    frame["left_rgb"], frame["right_rgb"],
                    anchor=camera, baseline_m=0.6,
                )
                actual_scene = np.isfinite(truth) & (truth >= 0.45) & (truth <= 10.0)
                valid_scene = valid & ~static_mask & actual_scene
                errors = np.abs(depth[valid_scene] - truth[valid_scene])
                samples.append({
                    "total_valid_fraction": float(valid.mean()),
                    "valid_true_scene_fraction": float(valid_scene.mean()),
                    "valid_matches_in_static_self_mask_fraction": float((valid & static_mask).sum() / max(valid.sum(), 1)),
                    "true_self_missed_by_e0_mask_fraction": float((actual_self & ~static_mask).sum() / max(actual_self.sum(), 1)),
                    "scene_depth_median_absolute_error_m": float(np.median(errors)) if len(errors) else None,
                })
            camera_result["scenes"][scene] = {
                "samples": samples,
                "median_total_valid_fraction": float(np.median([sample["total_valid_fraction"] for sample in samples])),
                "median_valid_true_scene_fraction": float(np.median([sample["valid_true_scene_fraction"] for sample in samples])),
                "median_scene_depth_error_m": float(np.median([sample["scene_depth_median_absolute_error_m"] for sample in samples])),
                "max_self_missed_fraction": float(max(sample["true_self_missed_by_e0_mask_fraction"] for sample in samples)),
            }
        result["cameras"][camera] = camera_result
    output = ROOT / "frozen_rig_selfmask_audit.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(output)


if __name__ == "__main__":
    main()
