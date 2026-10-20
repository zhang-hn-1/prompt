import numpy as np

from visual_local_policy_v3 import project_external_depth, choose_external_heading


def test_external_camera_projection_uses_short_baseline_positions():
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    depth[240, 320] = 2.0
    valid = np.isfinite(depth)
    left = project_external_depth(depth, valid, "left", np.array([1.0, 0, 0, 0]))
    right = project_external_depth(depth, valid, "right", np.array([1.0, 0, 0, 0]))
    assert left.shape == right.shape == (1, 2)
    assert np.allclose(left[0], [2.48, 0.10], atol=0.01)
    assert np.allclose(right[0], [2.48, -0.10], atol=0.01)


def test_external_corrridor_stays_straight_but_center_obstacle_turns():
    straight, blocked = choose_external_heading(
        np.array([5.0, 0.0]), np.array([[2.0, 0.6], [2.0, -0.6]])
    )
    turn, turn_blocked = choose_external_heading(
        np.array([5.0, 0.0]), np.array([[2.0, 0.0]])
    )
    assert straight == 0.0 and not blocked
    assert abs(turn) >= 0.29 and not turn_blocked
