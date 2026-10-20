"""Read-only vertical-stereo correspondence probe on the lit E1/E2 sequences."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import pilot  # noqa: E402


def inspect_frame(path: Path) -> dict:
    with np.load(path) as frame:
        left = np.rot90(cv2.cvtColor(frame["left_rgb"], cv2.COLOR_RGB2GRAY), 1)
        right = np.rot90(cv2.cvtColor(frame["right_rgb"], cv2.COLOR_RGB2GRAY), 1)
        baseline = float(np.linalg.norm(
            frame["left_camera_usd_world"][:3, 3] - frame["right_camera_usd_world"][:3, 3]))
        left_matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=160, blockSize=5,
            P1=8 * 5 * 5, P2=32 * 5 * 5,
            disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=50,
            speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        disp = left_matcher.compute(left, right).astype(np.float32) / 16.0
        uv = pilot.sample_grid()
        d = disp[639 - uv[:, 0], uv[:, 1]]
        true_depth = frame["left_depth_m"][::4, ::4].reshape(-1)
        root = frame["root_position_world_m"]
        yaw = pilot.yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
        labels, central = pilot.classify_depth_samples(
            true_depth, uv, frame["left_camera_usd_world"], root, yaw)
        finite = np.isfinite(true_depth) & (true_depth >= 0.8) & (true_depth <= 4.0)
        expected = pilot.FOCAL_PX * baseline / np.maximum(true_depth, 0.1)
        valid = np.isfinite(d) & (d > 1.0)
        correct = valid & finite & (np.abs(d - expected) <= 8.0)
        wall = central & (labels == pilot.OBSTACLE) & finite
        floor = central & (labels == pilot.FREE) & finite

        def rate(mask: np.ndarray) -> dict:
            n = int(mask.sum())
            return {"pixels": n, "valid_match_fraction": float((valid & mask).sum() / n) if n else None,
                    "accurate_match_fraction": float((correct & mask).sum() / n) if n else None,
                    "accurate_given_valid": float((correct & mask).sum() / (valid & mask).sum())
                    if (valid & mask).any() else None}

        return {"frame": path.name, "time_s": float(frame["simulation_time_s"]),
                "baseline_m": baseline, "wall": rate(wall), "floor": rate(floor)}


def main() -> None:
    cv2.setNumThreads(4)
    result = {"protocol": "exploratory unchanged SGBM, 160 disparities, no truth used for matching",
              "results": {}}
    for scene in ("e1", "e2"):
        folder = HERE.parent / f"integrated_fill_{scene}" / "depth_truth"
        paths = [folder / f"frame_{idx:03d}.npz" for idx in (0, 2, 4, 6, 8, 10, 20)]
        result["results"][scene] = [inspect_frame(p) for p in paths if p.exists()]
    (HERE / "stereo_probe.json").write_text(json.dumps(result, indent=2) + "\n")
    for scene, frames in result["results"].items():
        print(scene)
        for x in frames:
            print(x["time_s"], "wall", x["wall"], "floor", x["floor"])


if __name__ == "__main__":
    main()
