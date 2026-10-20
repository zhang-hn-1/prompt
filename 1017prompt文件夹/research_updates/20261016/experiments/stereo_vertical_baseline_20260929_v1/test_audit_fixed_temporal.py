import numpy as np

from audit_fixed_temporal import binary_auc, camera_geometry, project_world


def test_binary_auc_ties_and_reversed_scores():
    assert binary_auc(np.array([0, 0, 1, 1]), np.array([0, 0, 1, 1])) == 1.0
    assert binary_auc(np.array([0, 1]), np.array([1, 0])) == 0.0
    assert binary_auc(np.array([0, 1]), np.array([2, 2])) == 0.5


def test_usd_camera_projection_matches_pixel_center():
    transform = np.eye(4)
    rotation, center = camera_geometry(transform)
    world_point = center - transform[:3, 2] * 2.0
    u, v, depth = project_world(world_point[None], rotation, center)
    assert np.allclose([u[0], v[0], depth[0]], [319.5, 239.5, 2.0])
