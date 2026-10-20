import numpy as np
from route_geometry_audit import project, dense_prefix, hazard_clearance, tracking_window


def test_projection_measures_segment_distance_and_progress_without_vertex_jumps():
    p = np.array([[0., 0.], [1., 0.], [1., 1.]])
    arc, error, tangent = project([[.4, .2], [1.1, .7]], p)
    np.testing.assert_allclose(arc, [.4, 1.7])
    np.testing.assert_allclose(error, [.2, .1])
    np.testing.assert_allclose(tangent, [[1, 0], [0, 1]])


def test_one_metre_prefix_preserves_corner_and_does_not_use_target_chord():
    p = np.array([[0., 0.], [.5, 0.], [.5, 1.]])
    dense = dense_prefix(p, 1., .05)
    np.testing.assert_allclose(dense[-1], [.5, .5])
    assert any(np.array_equal(v, [.5, 0.]) for v in dense)
    assert np.all((dense[:, 1] == 0) | (dense[:, 0] == .5))
    # A shortcut through the bend can collide while the polyline does not.
    assert hazard_clearance(dense, [[.25, .25]]) > .24
    assert hazard_clearance(dense_prefix([[0, 0], [.5, .5]], 1), [[.25, .25]]) < 1e-10


def test_astar_point_six_is_not_the_stopping_consumer_radius():
    clearance = hazard_clearance([[0, 0], [1, 0]], [[0, .6]])
    assert clearance >= .6 - 1e-8
    assert clearance < .5 + .14088407559444638 + .1 / np.sqrt(2)
    assert hazard_clearance([[0, 0]], np.empty((0, 2))) is None


def test_window_uses_one_frozen_path_even_if_the_next_replan_starts_at_robot():
    old = np.array([[0., 0.], [2., 0.]])
    points = np.array([[0., 0.], [.1, .3]])
    result = tracking_window(points, old)
    np.testing.assert_allclose(result['progress_gain_m'], .1)
    np.testing.assert_allclose(result['exit_error_m'], .3)
    assert result['rms_error_m'] > .2
    assert project(points[-1:], [[.1, .3], [2, 0]])[1][0] == 0
