"""Synthetic and recorded checks for binocular geometric overlap."""

import numpy as np

from stereo_clearance_audit import reproject_between_eyes


def test_vertical_baseline_disparity_and_distance() -> None:
    left = np.eye(4)
    right = np.eye(4)
    # USD +Y is image up; camera two is 0.6 m along image-down optical Y.
    right[1, 3] = -0.6
    uv = np.array([[319.5, 239.5]])
    right_uv, right_z = reproject_between_eyes(np.array([3.0]), left, right, uv)
    assert np.isclose(right_z[0], 3.0)
    assert np.isclose(right_uv[0, 0], uv[0, 0])
    assert np.isclose(right_uv[0, 1] - uv[0, 1], -(0.6 / 3.0) * (16.45 / 13.5 * 480))


def test_recorded_pair_keeps_same_image_column() -> None:
    path = "experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/bilateral_depth_detour_e1/depth_truth/frame_010.npz"
    with np.load(path) as frame:
        uv = np.array([[319.5, 239.5]])
        right_uv, right_z = reproject_between_eyes(np.array([3.0]),
            frame["left_camera_usd_world"], frame["right_camera_usd_world"], uv)
    assert abs(right_uv[0, 0] - uv[0, 0]) < 0.001
    assert np.isclose(right_z[0], 3.0, atol=1e-6)
