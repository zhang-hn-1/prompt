import numpy as np

from visual_local_policy_v2 import supported_depth_mask


def test_black_textureless_pixels_cannot_create_obstacles():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[160:200, 200:240] = (30, 45, 80)
    valid = np.ones((480, 640), dtype=bool)
    left = supported_depth_mask(image, valid, camera="left")
    assert not left[180, 300]
    assert left[180, 220]
    assert not left[320, 220]


def test_each_eye_retains_its_visible_lateral_half():
    image = np.full((480, 640, 3), 100, dtype=np.uint8)
    valid = np.ones((480, 640), dtype=bool)
    left = supported_depth_mask(image, valid, camera="left")
    right = supported_depth_mask(image, valid, camera="right")
    assert left[150, 300] and not left[320, 300]
    assert right[320, 300] and not right[40, 300]
