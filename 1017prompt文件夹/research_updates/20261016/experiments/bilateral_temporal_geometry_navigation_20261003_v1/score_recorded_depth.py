"""Score already-recorded online predictions; truth never enters the policy."""
import json
import os
from pathlib import Path
import numpy as np
from geometry import FOCAL, CX, CY


def summary(errors):
    x = np.asarray(errors, dtype=float)
    finite = np.isfinite(x)
    # Report unmeasurable errors instead of interpolating infinities.
    rank = int(np.ceil(.9 * len(x))) - 1 if len(x) else 0
    p90 = np.sort(x)[rank] if len(x) else np.inf
    return dict(predictions=len(x), measurable=int(finite.sum()),
                fraction_error_le_0_5_m=float(np.mean(x <= .5)) if len(x) else None,
                p90_error_m=float(p90) if np.isfinite(p90) else None,
                p90_unmeasurable=bool(len(x) and not np.isfinite(p90)))


def main():
    here = Path(os.environ['BILATERAL_ASSESS_ROOT'])
    results = {}
    for label in ('e2', 'e1'):
        root = here / f'candidate_{label}'
        errors, scene_errors = [], []
        by_eye = {}; origins = dict(self=0, floor=0, body_height=0, other_unknown=0)
        files = sorted((root/'online_inputs').glob('frame_*.npz'))
        for path in files:
            with np.load(path) as inp, np.load(root/'offline_truth'/path.name) as truth:
                pos = inp['position_world_xy']
                for eye in ('left', 'right'):
                    uv = inp[f'{eye}_uv']; coords = np.rint(uv).astype(int)
                    z = truth[f'{eye}_depth_m'][coords[:, 1], coords[:, 0]]
                    predicted = inp[f'{eye}_depth_m']
                    valid = np.isfinite(z) & (z > 0) & (z <= 10)
                    measurable = valid & (z >= .45)
                    error = np.where(measurable, np.abs(predicted-z), np.inf)
                    errors.extend(error.tolist())
                    by_eye[eye] = by_eye.get(eye, 0) + len(z)
                    matrix = truth[f'{eye}_camera_usd_world']
                    rotation = np.column_stack((matrix[:3, 0], -matrix[:3, 1], -matrix[:3, 2]))
                    rays = np.column_stack(((uv[:, 0]-CX)/FOCAL, (uv[:, 1]-CY)/FOCAL, np.ones(len(uv))))
                    world = matrix[:3, 3] + (rays*np.where(valid, z, 0)[:, None]) @ rotation.T
                    own = valid & (np.linalg.norm(world-truth['root_position_world_m'], axis=1) <= .55)
                    scene_errors.extend(error[measurable & ~own].tolist())
                    floor = valid & ~own & (np.abs(world[:, 2]) <= .12)
                    body = valid & ~own & (world[:, 2] >= .25) & (world[:, 2] <= 1.5)
                    points = inp[f'{eye}_points_world']
                    picked = ((points[:, 2] >= .25) & (points[:, 2] <= 1.5)
                              & (np.linalg.norm(points-np.r_[pos, .4], axis=1) > .55))
                    origins['self'] += int(np.sum(picked & own))
                    origins['floor'] += int(np.sum(picked & floor))
                    origins['body_height'] += int(np.sum(picked & body))
                    origins['other_unknown'] += int(np.sum(picked & ~(own | floor | body)))
        results[label] = dict(frames=len(files), all_accepted=summary(errors),
            measurable_nonself_scene=summary(scene_errors), by_eye=by_eye,
            inferred_obstacle_point_true_origins=origins,
            scope='all recorded candidate frames on its actual closed-loop trajectory; repeated point observations, not independent objects or fresh held-out sequences')
        print(label, results[label], flush=True)
    (here/'recorded_depth_score.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
