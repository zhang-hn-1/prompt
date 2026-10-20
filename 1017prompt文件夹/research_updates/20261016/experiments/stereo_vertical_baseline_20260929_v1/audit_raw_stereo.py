"""Inspect the existing raw stereo capture without simulator or map truth."""

import json
from pathlib import Path

import cv2
import numpy as np

from vertical_depth import vertical_stereo_depth


DATA = Path("/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/data/wm_aux30_calibration_20260926_v1/easy_1/easy_1_cal_forward_exchange/ep0000")
OUT = Path(__file__).resolve().parent


def run() -> None:
    states = np.load(DATA / "states.npz")
    rows = []
    for index in (0, 45, 80, 120, 220):
        left = cv2.cvtColor(cv2.imread(str(DATA / str(states["left_filename"][index]))), cv2.COLOR_BGR2RGB)
        right = cv2.cvtColor(cv2.imread(str(DATA / str(states["right_filename"][index]))), cv2.COLOR_BGR2RGB)
        depth, valid = vertical_stereo_depth(left, right)
        # Original image v increases toward robot right; central crop includes forward view.
        central = np.zeros(valid.shape, dtype=bool)
        central[80:400, 100:540] = True
        roi_valid = valid & central
        near = roi_valid & (depth < 3.0)
        rows.append({
            "frame": index,
            "time_s": float(states["left_timestamp"][index]),
            "valid_full_fraction": float(valid.mean()),
            "valid_central_fraction": float(roi_valid.sum() / central.sum()),
            "near_central_fraction": float(near.sum() / central.sum()),
            "central_median_depth_m": float(np.median(depth[roi_valid])) if roi_valid.any() else None,
        })
        if index in (0, 80, 220):
            color = np.zeros((*valid.shape, 3), dtype=np.uint8)
            color[valid] = cv2.applyColorMap(
                np.clip((depth[valid] / 10.0 * 255), 0, 255).astype(np.uint8).reshape(-1, 1),
                cv2.COLORMAP_TURBO,
            ).reshape(-1, 3)
            visual = np.hstack((cv2.cvtColor(left, cv2.COLOR_RGB2BGR), color))
            cv2.imwrite(str(OUT / f"frame_{index:03d}_depth.png"), visual)
    (OUT / "raw_stereo_audit.json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    run()
