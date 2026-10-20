import numpy as np

from vertical_depth import vertical_stereo_depth


def test_vertical_disparity_recovers_metric_depth():
    rng = np.random.default_rng(20260929)
    left_gray = rng.integers(0, 256, size=(480, 640), dtype=np.uint8)
    disparity_px = 40
    right_gray = np.zeros_like(left_gray)
    right_gray[:-disparity_px] = left_gray[disparity_px:]
    left = np.repeat(left_gray[..., None], 3, axis=2)
    right = np.repeat(right_gray[..., None], 3, axis=2)

    depth, valid = vertical_stereo_depth(left, right, num_disparities=128)

    region = (slice(100, 380), slice(120, 520))
    expected_depth = (16.45 / 13.5 * 480.0) * 0.6 / disparity_px
    assert valid[region].mean() > 0.8
    assert abs(float(np.median(depth[region][valid[region]])) - expected_depth) < 0.1

    right_depth, right_valid = vertical_stereo_depth(left, right, num_disparities=128, anchor="right")
    assert right_valid[region].mean() > 0.8
    assert abs(float(np.median(right_depth[region][right_valid[region]])) - expected_depth) < 0.1

    narrow_depth, narrow_valid = vertical_stereo_depth(
        left, right, num_disparities=128, baseline_m=0.2
    )
    assert narrow_valid[region].mean() > 0.8
    assert abs(float(np.median(narrow_depth[region][narrow_valid[region]])) - expected_depth / 3) < 0.1


def test_rejects_mismatched_images():
    left = np.zeros((480, 640, 3), dtype=np.uint8)
    right = np.zeros((240, 640, 3), dtype=np.uint8)
    try:
        vertical_stereo_depth(left, right)
    except ValueError as error:
        assert "same" in str(error)
    else:
        raise AssertionError("mismatched camera frames must fail")
