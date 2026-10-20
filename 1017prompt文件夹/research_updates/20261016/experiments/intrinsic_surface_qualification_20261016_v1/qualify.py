"""GT-only full-sequence prerequisite; no perception model or online GT use."""
from pathlib import Path
import sys, json, time, hashlib
import numpy as np
from scipy.spatial import cKDTree

D = Path(__file__).resolve().parent
W = D.parents[1]
H = W / 'experiments/contact_boundary_source_20261016_v1'
V = W / 'experiments/dual_monocular_semantic_bev_20261015_v1'
sys.path.insert(0, str(V))
import projection as consumer
from semantic_projection import project_semantic

NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right']
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()

def cells(world, origin):
    return np.unique(np.floor((world[:, :2] - origin) / .1 + .5).astype(np.int64), axis=0)

def union(arrays):
    return np.unique(np.concatenate(arrays).reshape(-1, 2), axis=0)

def metric(pred, target):
    pd = cKDTree(target * .1).query(pred * .1)[0] if len(target) and len(pred) else np.full(len(pred), np.inf)
    td = cKDTree(pred * .1).query(target * .1)[0] if len(target) and len(pred) else np.full(len(target), np.inf)
    return dict(predicted=len(pred), target=len(target), correct=int((pd <= .15).sum()),
                covered=int((td <= .15).sum()), precision=float(np.mean(pd <= .15)) if len(pred) else None,
                recall=float(np.mean(td <= .15)) if len(target) else None,
                position_p90_m=float(np.quantile(pd, .9)) if len(pd) and np.isfinite(pd).all() else None), pd

def frame(a, cached, seal):
    focal = seal['initial_local_identity'][0]['optics'][0] / 18 * 640
    assert all(abs(v['optics'][0] / v['optics'][1] * 640 - focal) < 1e-8 for v in seal['initial_local_identity'])
    consumer.F = focal
    origin = cached['origin']
    yy, xx = np.indices((480, 640))
    rays = np.stack([(xx - 319.5) / focal, (yy - 239.5) / focal, np.ones_like(xx)], -1)
    target = []
    classes = []
    arms = []
    pixel_errors = []
    native_bodies = []
    for j, eye in enumerate(NAMES):
        z = a[eye + '_depth_label']
        T = a[eye + '_camera_usd_world_label']
        N = a[eye + '_camera_usd_world_estimated']
        B = N @ np.linalg.inv(np.asarray(seal['initial_local_identity'][j]['local_matrix']))
        native_bodies.append(B)
        mask = cached[eye + '_CAD']
        valid = np.isfinite(z) & (z >= .5) & (z <= 3) & ~mask
        q = np.flatnonzero(valid)
        optical = rays.reshape(-1, 3)[q] * z.reshape(-1)[q, None]
        true = optical @ (T[:3, :3] @ consumer.AX).T + T[:3, 3]
        estimated = optical @ (N[:3, :3] @ consumer.AX).T + N[:3, 3]
        outside = np.linalg.norm(true - a['root_position_physics_label'], axis=1) >= .55
        hazard = outside & (true[:, 2] > .05) & (true[:, 2] <= 1.5)
        floor = outside & (np.abs(true[:, 2]) <= .03)
        cl = np.zeros((480, 640), np.uint8)
        cl.reshape(-1)[q[floor]] = 2
        cl.reshape(-1)[q[hazard]] = 1
        classes.append(cl)
        target.append(cells(true[hazard], origin))
        error = np.linalg.norm(estimated[hazard, :2] - true[hazard, :2], axis=1)
        pixel_errors.append(error)
        arms.append(dict(camera=eye,all_pixels=z.size,finite_range_nonCAD_pixels=int(valid.sum()),
                         true_target_pixels=int(hazard.sum()),true_floor_pixels=int(floor.sum()),
                         unknown_pixels=int((cl == 0).sum()),oracle_ground_false_pixels=0,
                         native_pose_pixel_xy_p90_m=float(np.quantile(error, .9)) if len(error) else None))
    target = union(target)
    assert np.array_equal(target, cached['target_union']), 'Original full-domain denominator changed'
    assert max(np.max(abs(b - native_bodies[0])) for b in native_bodies) < 1e-6
    predictions = {}; conflicts = {}
    for method, suffix in [('oracle', '_label'), ('native', '_estimated')]:
        pair_sets = []; per_eye = []; pair_conflict = []
        for pair in [(0, 1), (2, 3)]:
            zs = [a[NAMES[j] + '_depth_label'] for j in pair]
            masks = {k: cached[NAMES[j] + '_CAD'] for k, j in zip(['left', 'right'], pair)}
            pose = {k + '_camera_usd_world': a[NAMES[j] + '_camera_usd_world' + suffix] for k, j in zip(['left', 'right'], pair)}
            pose['root_position_world_m'] = a['root_position_physics_label'] if method == 'oracle' else native_bodies[pair[0]][:3, 3]
            pred, conflict = project_semantic(zs, [classes[j] for j in pair], pose, masks, origin)
            pair_sets.append(pred[2]); per_eye.extend(pred[:2]); pair_conflict.extend(conflict)
        predictions[method] = union(pair_sets)
        conflicts[method] = pair_conflict
    return target, predictions, arms, conflicts, np.concatenate(pixel_errors)

def main():
    start = time.monotonic(); cfg = json.loads((D / 'protocol.json').read_text())
    for p, h in cfg['input_sha256'].items(): assert sha(p) == h, p
    root = Path(cfg['data_root']); seal = json.loads((root / 'capture_seal.json').read_text())
    out = D / 'maps'; out.mkdir(exist_ok=False)
    rows = []; distances = {m: [] for m in ['oracle', 'native']}; pixel_errors = []
    for rr in seal['rows']:
        path = root / 'frames' / rr['frame']; assert sha(path) == rr['sha256']
        with np.load(path) as a, np.load(H / 'e0_recovery_qualification' / rr['frame']) as cached:
            target, preds, arms, conflicts, pe = frame(a, cached, seal)
            row = dict(frame=rr['frame'], time_s=float(a['source_time_s']), arms=arms, conflicts=conflicts)
            save = dict(target=target, origin=cached['origin'], root=a['root_position_physics_label'])
            for m in preds:
                row[m], dist = metric(preds[m], target); distances[m].extend(dist.tolist()); save[m] = preds[m]
            for name in ['floor_intersecting', 'suspended', 'unidentified']:
                save[name] = cached[name + '_cells']
                for m in preds:
                    row.setdefault('category_coverage', {}).setdefault(m, {})[name] = metric(preds[m], cached[name + '_cells'])[0]
            np.savez_compressed(out / rr['frame'], **save); rows.append(row); pixel_errors.extend(pe.tolist())
        if len(rows) % 10 == 1: print(len(rows), '/', len(seal['rows']), row['native'], flush=True)
    aggregates = {}; gates = {}
    for m in distances:
        totals = {k: sum(r[m][k] for r in rows) for k in ['predicted', 'target', 'correct', 'covered']}
        dist = np.asarray(distances[m]); n = totals['predicted']; t = totals['target']
        ag = dict(**totals, precision=totals['correct']/n if n else None, recall=totals['covered']/t if t else None,
                  position_p90_m=float(np.quantile(dist, .9)) if len(dist) and np.isfinite(dist).all() else None)
        aggregates[m] = ag
        gates[m] = dict(precision=ag['precision'] is not None and ag['precision'] >= .95,
                        recall=ag['recall'] is not None and ag['recall'] >= .8,
                        position=ag['position_p90_m'] is not None and ag['position_p90_m'] <= .15,
                        every_frame_precision=all(r[m]['predicted'] == 0 or r[m]['precision'] >= .95 for r in rows),
                        every_frame_recall=all(r[m]['target'] == 0 or r[m]['recall'] >= .8 for r in rows),
                        every_frame_position=all(r[m]['predicted'] == 0 or (r[m]['position_p90_m'] is not None and r[m]['position_p90_m'] <= .3) for r in rows))
    result = dict(complete=True, rows=rows, aggregate=aggregates, gates=gates,
                  necessary_consumer_pass=all(all(g.values()) for g in gates.values()),
                  native_pose_target_pixel_xy_p50_p90_p95_m=np.quantile(pixel_errors,[.5,.9,.95]).tolist(),
                  frames=len(rows), eyes=len(rows)*4, source_admitted=False, model=0, fit=0, physics=0, navigation=0,
                  wall_s=time.monotonic()-start, source_sha256=sha(Path(__file__)),
                  scope='Ideal GT surface category+depth oracle and fixed v76 consumer on complete current E0, rendered vs causal native poses. Not RGB/model accuracy, new source, free space or navigation. No posthoc predicted height semantics.')
    (D / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print('FINAL', json.dumps({k: result[k] for k in ['aggregate','gates','necessary_consumer_pass','wall_s']},ensure_ascii=False), flush=True)

if __name__ == '__main__': main()
