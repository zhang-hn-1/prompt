"""Metric stereo depth for the benchmark's vertically displaced cameras."""

import cv2
import numpy as np


FOCAL_LENGTH_PX = 16.45 / 13.5 * 480.0
BASELINE_M = 0.6


def vertical_stereo_depth(
    left_rgb: np.ndarray,
    right_rgb: np.ndarray,
    *,
    num_disparities: int = 192,
    min_depth_m: float = 0.6,
    max_depth_m: float = 10.0,
    anchor: str = "left",
    baseline_m: float = BASELINE_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Return depth and validity in the original 480x640 image coordinates.

    The benchmark camera baseline projects along image rows, so both images
    are rotated before using OpenCV's horizontal disparity matcher. A left
    pad removes StereoSGBM's otherwise large invalid strip on the left.
    """
    left = np.asarray(left_rgb)
    right = np.asarray(right_rgb)
    if left.shape != right.shape or left.ndim != 3 or left.shape[2] != 3:
        raise ValueError("left and right frames must have the same HxWx3 shape")
    if num_disparities <= 0 or num_disparities % 16:
        raise ValueError("num_disparities must be a positive multiple of 16")
    if anchor not in {"left", "right"}:
        raise ValueError("anchor must be left or right")
    if not np.isfinite(baseline_m) or baseline_m <= 0:
        raise ValueError("baseline_m must be finite and positive")
    left_gray = cv2.cvtColor(left, cv2.COLOR_RGB2GRAY)
    right_gray = cv2.cvtColor(right, cv2.COLOR_RGB2GRAY)
    if anchor == "left":
        rotation = cv2.ROTATE_90_COUNTERCLOCKWISE
        inverse_rotation = cv2.ROTATE_90_CLOCKWISE
        first = left_gray
        second = right_gray
    else:
        rotation = cv2.ROTATE_90_CLOCKWISE
        inverse_rotation = cv2.ROTATE_90_COUNTERCLOCKWISE
        first = right_gray
        second = left_gray
    first_rot = cv2.rotate(first, rotation)
    second_rot = cv2.rotate(second, rotation)
    padding = num_disparities
    left_pad = cv2.copyMakeBorder(first_rot, 0, 0, padding, 0, cv2.BORDER_REPLICATE)
    right_pad = cv2.copyMakeBorder(second_rot, 0, 0, padding, 0, cv2.BORDER_REPLICATE)
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=5,
        P1=8 * 5 * 5,
        P2=32 * 5 * 5,
        uniquenessRatio=5,
        speckleWindowSize=50,
        speckleRange=2,
        disp12MaxDiff=1,
    )
    disparity_rot = matcher.compute(left_pad, right_pad)[:, padding:].astype(np.float32) / 16.0
    disparity = cv2.rotate(disparity_rot, inverse_rotation)
    positive = disparity > 0
    depth = np.zeros(disparity.shape, dtype=np.float32)
    depth[positive] = FOCAL_LENGTH_PX * baseline_m / disparity[positive]
    valid = positive & (depth >= min_depth_m) & (depth <= max_depth_m)
    depth[~valid] = np.nan
    return depth, valid
