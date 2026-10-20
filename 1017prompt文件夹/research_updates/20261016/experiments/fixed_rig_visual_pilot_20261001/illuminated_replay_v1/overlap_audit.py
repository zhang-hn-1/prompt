"""Measure true cross-eye overlap offline; no scene truth enters the controller."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import pilot  # noqa: E402

OLD = HERE.parents[1] / "stereo_vertical_baseline_20260929_v1"
NEW = HERE.parent


def frame_counts(path: Path) -> dict:
    uv = pilot.sample_grid()
    with np.load(path) as frame:
        depth = frame["left_depth_m"][::4, ::4].reshape(-1)
        left_pose = frame["left_camera_usd_world"].astype(np.float64)
        right_pose = frame["right_camera_usd_world"].astype(np.float64)
        labels, central = pilot.classify_depth_samples(
            depth, uv, left_pose, frame["root_position_world_m"],
            pilot.yaw_from_quaternion(frame["root_quaternion_world_wxyz"]),
        )
        rays = np.column_stack(((uv[:, 0] - pilot.CX) / pilot.FOCAL_PX,
                                (uv[:, 1] - pilot.CY) / pilot.FOCAL_PX,
                                np.ones(len(uv))))
        left_rotation = np.column_stack((left_pose[:3, 0], -left_pose[:3, 1], -left_pose[:3, 2]))
        right_rotation = np.column_stack((right_pose[:3, 0], -right_pose[:3, 1], -right_pose[:3, 2]))
        finite = np.isfinite(depth) & (depth > 0)
        hit = left_pose[:3, 3] + (rays * np.where(finite, depth, 0)[:, None]) @ left_rotation.T
        right_xyz = (hit - right_pose[:3, 3]) @ right_rotation
        safe_z = np.where(right_xyz[:, 2] > 0.01, right_xyz[:, 2], 1)
        right_u = np.rint(pilot.CX + pilot.FOCAL_PX * right_xyz[:, 0] / safe_z).astype(int)
        right_v = np.rint(pilot.CY + pilot.FOCAL_PX * right_xyz[:, 1] / safe_z).astype(int)
        inside = ((right_u >= 0) & (right_u < 640) & (right_v >= 0) & (right_v < 480)
                  & (right_xyz[:, 2] > 0))
        sample_u = np.clip(right_u, 0, 639)
        sample_v = np.clip(right_v, 0, 479)
        measured = frame["right_depth_m"][sample_v, sample_u]
        same = inside & np.isfinite(measured) & (np.abs(measured - right_xyz[:, 2]) <= 0.2)
        return {
            "time_s": float(frame["simulation_time_s"]),
            "wall": summarize(central & (labels == pilot.OBSTACLE), inside, same),
            "floor": summarize(central & (labels == pilot.FREE), inside, same),
        }


def summarize(mask: np.ndarray, inside: np.ndarray, same: np.ndarray) -> dict:
    return {"visible_left": int(mask.sum()),
            "projects_inside_right": int((mask & inside).sum()),
            "same_surface_visible_right": int((mask & same).sum())}


def main() -> None:
    roots = {
        "old_e1": OLD / "fixed_sequence_golden_e1_0001" / "depth_truth",
        "old_e2": OLD / "fixed_sequence_golden_e2_0001" / "depth_truth",
        "lit_e1": NEW / "integrated_fill_e1" / "depth_truth",
        "lit_e2": NEW / "integrated_fill_e2" / "depth_truth",
    }
    result = {name: [frame_counts(root / f"frame_{i:03d}.npz")
                     for i in range(0, 22, 2)] for name, root in roots.items()}
    (HERE / "overlap_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    for name, frames in result.items():
        for surface in ("wall", "floor"):
            total = sum(item[surface]["visible_left"] for item in frames)
            overlap = sum(item[surface]["same_surface_visible_right"] for item in frames)
            print(name, surface, overlap, "/", total,
                  f"({100 * overlap / total:.1f}%)" if total else "n/a")


if __name__ == "__main__":
    main()
