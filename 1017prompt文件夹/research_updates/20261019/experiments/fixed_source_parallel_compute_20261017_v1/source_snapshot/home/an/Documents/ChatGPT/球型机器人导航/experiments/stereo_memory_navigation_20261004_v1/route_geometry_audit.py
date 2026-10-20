"""Offline geometry only. Never feeds evaluated trajectories back to control."""
import numpy as np

def project(points, path):
    p = np.asarray(points, float).reshape(-1, 2)
    path = np.asarray(path, float).reshape(-1, 2)
    if len(path) < 2 or not np.isfinite(path).all() or not np.isfinite(p).all():
        raise ValueError('finite polyline with two vertices required')
    delta = np.diff(path, axis=0)
    length = np.linalg.norm(delta, axis=1)
    if np.any(length <= 0):
        raise ValueError('duplicate vertices')
    u = np.clip(((p[:, None] - path[:-1]) * delta).sum(2) / length**2, 0, 1)
    error = np.linalg.norm(p[:, None] - path[:-1] - u[..., None]*delta, axis=2)
    idx = error.argmin(1); row = np.arange(len(p))
    arc = np.r_[0., np.cumsum(length)]
    return arc[idx]+u[row, idx]*length[idx], error[row, idx], delta[idx]/length[idx, None]

def dense_prefix(path, length_m=1., spacing_m=.025):
    path = np.asarray(path, float).reshape(-1, 2)
    if not len(path) or not np.isfinite(path).all() or length_m < 0 or spacing_m <= 0:
        raise ValueError('finite nonempty path and positive spacing required')
    out = [path[0]]; left = length_m
    for a, b in zip(path[:-1], path[1:]):
        length = float(np.linalg.norm(b-a))
        if length == 0:
            continue
        take = min(left, length)
        n = max(1, int(np.ceil(take/spacing_m)))
        out.extend(a+(b-a)*(np.arange(1, n+1)/n*take/length)[:, None])
        left -= take
        if left <= 1e-10:
            break
    return np.asarray(out)

def hazard_clearance(points, hazard_centres):
    h = np.asarray(hazard_centres, float).reshape(-1, 2)
    p = np.asarray(points, float).reshape(-1, 2)
    if not len(h):
        return None
    if not len(p) or not np.isfinite(p).all() or not np.isfinite(h).all():
        raise ValueError('finite points required')
    if len(p) == 1:
        return float(np.linalg.norm(h-p[0], axis=1).min())
    # Exact continuous-segment distance, not nearest sampled-point distance.
    return float(project(h, p)[1].min())

def tracking_window(points, path):
    arc, error, _ = project(points, path)
    return dict(progress_gain_m=float(arc[-1]-arc[0]), entry_error_m=float(error[0]),
                exit_error_m=float(error[-1]), rms_error_m=float(np.sqrt(np.mean(error**2))),
                peak_error_m=float(error.max()))
