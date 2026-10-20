"""Offline measurement only; never used by the RGB predictor or control policy."""
import numpy as np


def warp_patch_measurements(photo, right_support, left_mask, true_disparity, coordinates, radius):
    coordinates = np.asarray(coordinates, int).reshape(-1, 2)
    cost = np.full(len(coordinates), np.nan); span = cost.copy()
    h, w = np.shape(photo)
    for i, (x, y) in enumerate(coordinates):
        if x-radius < 0 or x+radius >= w or y-radius < 0 or y+radius >= h:
            continue
        region = np.s_[y-radius:y+radius+1, x-radius:x+radius+1]
        disparity = np.asarray(true_disparity)[region]
        if (np.asarray(left_mask)[region].any() or not np.asarray(right_support)[region].all()
                or not np.isfinite(disparity).all() or (disparity <= 0).any()):
            continue
        cost[i] = float(np.asarray(photo)[region].mean())
        span[i] = float(np.ptp(disparity))
    return cost, span


def visibility_partition(z, right_z, left_mask, right_mask, focal_baseline):
    z = np.asarray(z, float)
    if not z.shape == np.shape(right_z) == np.shape(left_mask) == np.shape(right_mask):
        raise ValueError('visibility input shapes differ')
    y, x = np.indices(z.shape)
    valid = np.isfinite(z) & (z > 0)
    vr = y - np.divide(focal_baseline, z, out=np.zeros_like(z), where=valid)
    iy = np.rint(np.clip(vr, 0, z.shape[0]-1)).astype(int)
    partner = np.asarray(right_z)[iy, x]
    labels = np.zeros(z.shape, np.uint8)
    labels[valid] = 6
    labels[valid & (~np.isfinite(partner) | (partner <= 0))] = 5
    with np.errstate(invalid='ignore'):
        inconsistent = np.abs(partner-z) > .03+.01*z
    labels[valid & np.isfinite(partner) & (partner > 0) & inconsistent] = 4
    labels[valid & np.asarray(right_mask, bool)[iy, x]] = 3
    labels[valid & ((vr < 0) | (vr > z.shape[0]-1))] = 2
    labels[np.asarray(left_mask, bool)] = 1
    return labels


def patch_costs(left, right, left_mask, right_mask, coordinates, radius, max_disparity):
    left = np.asarray(left, float); right = np.asarray(right, float)
    coordinates = np.asarray(coordinates, int).reshape(-1, 2)
    offsets = np.arange(-radius, radius+1)
    oy, ox = np.meshgrid(offsets, offsets, indexing='ij')
    disparities = np.arange(max_disparity+1)
    h, w = left.shape[:2]
    costs = np.full((len(coordinates), len(disparities)), np.inf)
    texture = np.full(len(coordinates), np.nan)
    for i, (x, y) in enumerate(coordinates):
        if x-radius < 0 or x+radius >= w or y-radius < 0 or y+radius >= h:
            continue
        yy = y+oy; xx = x+ox
        if np.asarray(left_mask)[yy, xx].any():
            continue
        reference = left[yy, xx]
        texture[i] = reference.mean(axis=-1).std()
        rx = xx[None] - disparities[:, None, None]
        inside = ((rx >= 0) & (rx < w)).all(axis=(1, 2))
        safe_rx = np.clip(rx, 0, w-1)
        supported = inside & ~np.asarray(right_mask)[yy[None], safe_rx].any(axis=(1, 2))
        differences = np.abs(reference[None] - right[yy[None], safe_rx]).mean(axis=(1, 2, 3))
        costs[i, supported] = differences[supported]
    return costs, texture


def cost_metrics(costs, true_disparity, tolerance, exclusion_px):
    costs = np.asarray(costs, float); d = np.asarray(true_disparity, float)
    indices = np.arange(costs.shape[1])
    rounded = np.rint(np.nan_to_num(d, nan=-1)).astype(int)
    valid = np.isfinite(d) & (rounded >= 0) & (rounded < costs.shape[1])
    true_cost = costs[np.arange(len(d)), np.clip(rounded, 0, costs.shape[1]-1)].copy()
    valid &= np.isfinite(true_cost); true_cost[~valid] = np.nan
    rivals = (np.abs(indices[None]-d[:, None]) >= exclusion_px) & np.isfinite(costs)
    rival_count = (rivals & (costs <= true_cost[:, None]+tolerance)).sum(axis=1)
    best_rival = np.where(rivals, costs, np.inf).min(axis=1)
    margin = true_cost - best_rival
    margin[~np.isfinite(margin)] = np.nan
    has_candidate = np.isfinite(costs).any(axis=1)
    best = np.argmin(costs, axis=1).astype(float); best[~has_candidate] = np.nan
    return dict(true_cost=true_cost, true_measurable=valid,
                rival_count=rival_count, ambiguous=valid & (rival_count > 0),
                true_minus_best_rival=margin, best_disparity=best)
