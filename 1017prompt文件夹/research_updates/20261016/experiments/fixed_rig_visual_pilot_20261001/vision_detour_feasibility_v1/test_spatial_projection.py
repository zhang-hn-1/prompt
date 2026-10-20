"""Validate the fixed stereo rig against recorded simulator poses and floor hits."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

from spatial_projection import camera_pose_from_root, project_ground_pixels


HERE = Path(__file__).resolve().parent
PILOT = HERE.parent


@pytest.mark.parametrize("frame_path", [
    PILOT / "integrated_fill_e1/depth_truth/frame_010.npz",
    PILOT / "integrated_fill_e2/depth_truth/frame_020.npz",
    HERE / "bilateral_depth_h3/depth_truth/frame_020.npz",
])
def test_rig_reconstruction_matches_measured_camera_poses(frame_path: Path) -> None:
    with np.load(frame_path) as frame:
        position = frame["root_position_world_m"]
        quaternion = frame["root_quaternion_world_wxyz"]
        for eye in ("left", "right"):
            origin, optical = camera_pose_from_root(position[:2], quaternion, eye)
            measured = frame[f"{eye}_camera_usd_world"]
            measured_optical = np.column_stack((measured[:3, 0], -measured[:3, 1], -measured[:3, 2]))
            assert np.linalg.norm(origin - measured[:3, 3]) < 0.002
            assert np.max(np.abs(optical - measured_optical)) < 1e-5


def test_projected_floor_agrees_with_independent_depth_hits() -> None:
    sys.path.insert(0, str(PILOT))
    from pilot import FREE, FOCAL_PX, CX, CY, classify_depth_samples, sample_grid, yaw_from_quaternion

    frame_path = PILOT / "integrated_fill_e1/depth_truth/frame_010.npz"
    with np.load(frame_path) as frame:
        uv = sample_grid()
        root = frame["root_position_world_m"]
        quaternion = frame["root_quaternion_world_wxyz"]
        yaw = yaw_from_quaternion(quaternion)
        eye = "right"
        pose = frame[f"{eye}_camera_usd_world"]
        depth = frame[f"{eye}_depth_m"][::4, ::4].reshape(-1)
        labels, _ = classify_depth_samples(depth, uv, pose, root, yaw)
        chosen = (labels == FREE).reshape(120, 160)
        projected, valid = project_ground_pixels(chosen, root[:2], quaternion, eye)

        measured_optical = np.column_stack((pose[:3, 0], -pose[:3, 1], -pose[:3, 2]))
        rays = np.column_stack(((uv[:, 0] - CX) / FOCAL_PX,
                                (uv[:, 1] - CY) / FOCAL_PX,
                                np.ones(len(uv))))
        actual_hits = pose[:3, 3] + (rays * np.where(np.isfinite(depth), depth, 0)[:, None]) @ measured_optical.T
        selected_hits = actual_hits[chosen.reshape(-1)][valid][:, :2]
        assert len(projected) > 500
        assert np.median(np.linalg.norm(projected - selected_hits, axis=1)) < 0.25
