"""Literal intersections catch sign, nonunit-ray, cap and occlusion errors."""
import numpy as np
import scene_ray_geometry as geom


def test_sphere_uses_axial_ray_parameter_and_rejects_misses():
    rays = np.array([[0., 0., 1.], [0., 0., 2.], [2., 0., 1.]])
    depth = geom.sphere_depth([0., 0., 0.], rays, [0., 0., 3.], 1.)
    np.testing.assert_allclose(depth[:2], [2., 1.])
    assert np.isinf(depth[2])


def test_sphere_uses_exit_when_origin_is_inside():
    depth = geom.sphere_depth([0., 0., 0.], [[0., 0., 2.]], [0., 0., 0.], 1.)
    np.testing.assert_allclose(depth, [.5])


def test_cylinder_checks_caps_and_finite_height_instead_of_infinite_side():
    depth = geom.cylinder_depth([0., 0., 0.], [[0., 0., 1.], [.3, 0., 1.], [1., 0., 1.]],
        [0., 0., 3.], np.eye(3), 1., 2.)
    np.testing.assert_allclose(depth[:2], [2., 2.])
    assert np.isinf(depth[2])
    side = geom.cylinder_depth([2., 0., 3.], [[-1., 0., 0.]], [0., 0., 3.], np.eye(3), 1., 2.)
    np.testing.assert_allclose(side, [1.])


def test_usd_camera_forward_is_negative_local_z_and_image_y_is_down():
    pose = np.eye(4); pose[:3, 3] = [1., 2., 3.]
    origin, rays, rows, cols = geom.camera_rays(pose, 3, 3, 2., 1., 1., stride=1)
    np.testing.assert_array_equal(origin, [1., 2., 3.])
    np.testing.assert_allclose(rays[1, 1], [0., 0., -1.])
    np.testing.assert_allclose(rays[2, 1], [0., -.5, -1.])


def test_static_foreground_occludes_far_sphere_without_renderer_depth_input():
    shapes = [dict(type='Sphere', centre=[0., 0., 6.], radius=1.),
              dict(type='Sphere', centre=[0., 0., 3.], radius=1.)]
    depth, ids = geom.cast_scene([0., 0., 0.], np.array([[0., 0., 1.], [0., 0., -1.]]), shapes, 10., ground_z=-2.)
    np.testing.assert_allclose(depth, [2., 2.])
    np.testing.assert_array_equal(ids, [1, -2])


def test_missing_intersection_remains_unknown_beyond_budget():
    depth, ids = geom.cast_scene([0., 0., 1.], np.array([[0., 0., 1.]]), [], 3.)
    assert np.isinf(depth[0]) and ids[0] == -1


def test_convex_mesh_intersection_checks_all_planes_and_keeps_axial_parameter():
    # Cube x,y=[-1,1], z=[2,4], expressed as n.dot(point)+offset<=0.
    equations = np.array([[1,0,0,-1],[-1,0,0,-1],[0,1,0,-1],[0,-1,0,-1],[0,0,1,-4],[0,0,-1,2]])
    depth = geom.convex_mesh_depth([0.,0.,0.], np.array([[0.,0.,1.],[0.,0.,2.],[1.,0.,1.]]), equations)
    np.testing.assert_allclose(depth[:2], [2.,1.])
    assert np.isinf(depth[2])


def test_convex_mesh_parallel_outside_ray_is_rejected_but_inside_origin_can_exit():
    equations = np.array([[1,0,0,-1],[-1,0,0,-1],[0,1,0,-1],[0,-1,0,-1],[0,0,1,-4],[0,0,-1,2]])
    outside = geom.convex_mesh_depth([2.,0.,3.], [[0.,0.,1.]], equations)
    assert np.isinf(outside[0])
    inside = geom.convex_mesh_depth([0.,0.,3.], [[0.,0.,1.]], equations)
    np.testing.assert_allclose(inside, [1.])
