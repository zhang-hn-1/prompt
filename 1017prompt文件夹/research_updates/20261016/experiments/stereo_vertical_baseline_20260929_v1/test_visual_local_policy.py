import numpy as np

from visual_local_policy import points_from_depth, choose_local_heading


def test_original_image_vertical_pixel_is_robot_lateral_coordinate():
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    depth[200, 320] = 2.0
    depth[280, 320] = 2.0
    points = points_from_depth(depth, np.isfinite(depth), image_row_limit=300)
    assert points.shape == (2, 2)
    assert points[0, 1] > points[1, 1]
    assert 1.9 < points[:, 0].mean() < 2.2
    right_points = points_from_depth(depth, np.isfinite(depth), camera="right", image_row_limit=300)
    assert np.allclose(points[:, 1] - right_points[:, 1], 0.6)


def test_obstacle_in_goal_corridor_changes_heading():
    heading, blocked = choose_local_heading(np.array([4.0, 0.0]), np.array([[1.5, 0.0]]))
    assert abs(heading) > 0.25
    assert not blocked


def test_empty_depth_uses_pointgoal_heading():
    heading, blocked = choose_local_heading(np.array([2.0, 1.0]), np.empty((0, 2)))
    assert abs(heading - np.arctan2(1.0, 2.0)) < 0.1
    assert not blocked
