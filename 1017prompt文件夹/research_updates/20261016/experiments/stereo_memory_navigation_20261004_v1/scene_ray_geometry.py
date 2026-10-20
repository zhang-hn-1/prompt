"""Independent analytic static-scene labels only, never an online observation."""
import numpy as np


def convex_mesh_depth(origin, rays, equations):
    rays = np.asarray(rays, float)
    flat = rays.reshape(-1, 3)
    equations = np.asarray(equations, float)
    lower = np.full(len(flat), -np.inf)
    upper = np.full(len(flat), np.inf)
    outside = np.zeros(len(flat), bool)
    for start in range(0, len(equations), 64):
        planes = equations[start:start+64]
        q = planes[:, :3] @ np.asarray(origin, float) + planes[:, 3]
        denominator = flat @ planes[:, :3].T
        parallel = np.abs(denominator) <= 1e-12
        outside |= np.any(parallel & (q > 1e-10), axis=1)
        with np.errstate(divide='ignore', invalid='ignore'):
            crossing = -q / denominator
        lower = np.maximum(lower, np.max(np.where(denominator < -1e-12, crossing, -np.inf), axis=1))
        upper = np.minimum(upper, np.min(np.where(denominator > 1e-12, crossing, np.inf), axis=1))
    depth = np.where(lower > 1e-8, lower, upper)
    valid = (~outside) & (upper >= lower-1e-10) & np.isfinite(depth) & (depth > 1e-8)
    return np.where(valid, depth, np.inf).reshape(rays.shape[:-1])


def sphere_depth(origin, rays, centre, radius):
    rays = np.asarray(rays, float); offset = np.asarray(origin)-np.asarray(centre)
    a = np.sum(rays*rays, axis=-1); b = np.sum(rays*offset, axis=-1)
    c = np.sum(offset*offset)-radius*radius
    discriminant = b*b-a*c
    root = np.sqrt(np.maximum(discriminant, 0.))
    with np.errstate(divide='ignore', invalid='ignore'):
        first = (-b-root)/a; second = (-b+root)/a
    depth = np.where(first > 1e-8, first, second)
    return np.where((discriminant >= 0) & (depth > 1e-8) & (a > 0), depth, np.inf)


def cylinder_depth(origin, rays, centre, rotation, radius, height):
    rotation = np.asarray(rotation); rays = np.asarray(rays, float) @ rotation
    origin = (np.asarray(origin)-np.asarray(centre)) @ rotation
    a = np.sum(rays[..., :2]**2, axis=-1); b = np.sum(rays[..., :2]*origin[:2], axis=-1)
    c = np.sum(origin[:2]**2)-radius**2
    disc = b*b-a*c
    with np.errstate(divide='ignore', invalid='ignore'):
        root = np.sqrt(np.maximum(disc, 0))
        sides = [(-b-root)/a, (-b+root)/a]
        caps = [(-height/2-origin[2])/rays[..., 2], (height/2-origin[2])/rays[..., 2]]
    best = np.full(rays.shape[:-1], np.inf)
    for t in sides:
        with np.errstate(invalid='ignore'):
            valid = (a > 1e-15) & (disc >= 0) & (t > 1e-8) & (np.abs(origin[2]+t*rays[..., 2]) <= height/2+1e-10)
        best = np.minimum(best, np.where(valid, t, np.inf))
    for t in caps:
        with np.errstate(invalid='ignore'):
            point = origin[:2]+t[..., None]*rays[..., :2]
            valid = np.isfinite(t) & (t > 1e-8) & (np.sum(point*point, axis=-1) <= radius**2+1e-10)
        best = np.minimum(best, np.where(valid, t, np.inf))
    return best


def camera_rays(pose_usd, height, width, focal, cx, cy, stride=2):
    rows, cols = np.meshgrid(np.arange(0, height, stride), np.arange(0, width, stride), indexing='ij')
    optical = np.stack(((cols-cx)/focal, (rows-cy)/focal, np.ones(rows.shape)), -1)
    world_from_optical = np.asarray(pose_usd)[:3, :3] @ np.diag([1., -1., -1.])
    return np.asarray(pose_usd)[:3, 3], optical @ world_from_optical.T, rows, cols


def cast_scene(origin, rays, shapes, maximum_depth, ground_z=0.):
    rays = np.asarray(rays, float); origin = np.asarray(origin, float)
    depth = np.full(rays.shape[:-1], np.inf); identity = np.full(depth.shape, -1, int)
    maximum_ray_length = maximum_depth * np.max(np.linalg.norm(rays, axis=-1))
    for i, shape in enumerate(shapes):
        enclosing_radius = shape['radius']
        if shape['type'] == 'Cylinder': enclosing_radius = np.hypot(enclosing_radius, shape['height']/2)
        if np.linalg.norm(np.asarray(shape['centre'])-origin) > maximum_ray_length+enclosing_radius:
            continue
        if shape['type'] == 'Sphere':
            t = sphere_depth(origin, rays, shape['centre'], shape['radius'])
        elif shape['type'] == 'Cylinder':
            t = cylinder_depth(origin, rays, shape['centre'], shape['rotation'], shape['radius'], shape['height'])
        else:
            raise ValueError('unsupported analytic shape')
        take = (t < depth) & (t <= maximum_depth)
        depth[take] = t[take]; identity[take] = i
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ground_z-origin[2])/rays[..., 2]
    take = (t > 1e-8) & (t < depth) & (t <= maximum_depth)
    depth[take] = t[take]; identity[take] = -2
    return depth, identity
