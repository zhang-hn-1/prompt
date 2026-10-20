"""Fixed known-self RGB intervention, diagnostic only. No GT or source admission."""
from pathlib import Path
import sys, json, hashlib, time, datetime
import numpy as np

D = Path(__file__).resolve().parent
W = D.parents[1]
sys.path.insert(0, str(W / 'experiments/four_camera_weak_baseline_20261017_v1'))
import source as old
cv2 = old.cv2
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
put = lambda p, x: Path(p).write_text(json.dumps(x, indent=2, allow_nan=False) + '\n')
now = lambda: datetime.datetime.now(datetime.timezone.utc).isoformat()

def guard(cfg):
    for section in ['input_sha256', 'code_sha256']:
        for p, h in cfg[section].items():
            assert sha(p) == h, p

def predict(rgb, masks):
    # Exact frozen native matcher and padding; never instantiate/regenerate CAD.
    ml = cv2.StereoSGBM_create(minDisparity=0, **old.CONFIG)
    mr = cv2.StereoSGBM_create(minDisparity=-448, **old.CONFIG)
    pred = {}; calls = 0
    for q, p in old.PAIRS:
        a, b = [np.ascontiguousarray(np.rot90(rgb[n])) for n in [q, p]]
        lm, rm = [np.ascontiguousarray(np.rot90(masks[n])) for n in [q, p]]
        ap, bp = [cv2.copyMakeBorder(v, 0, 0, 448, 448, cv2.BORDER_CONSTANT, value=0) for v in [a, b]]
        dl = ml.compute(ap, bp).astype(np.float32)[:, 448:-448] / 16.; calls += 1
        dr = mr.compute(bp, ap).astype(np.float32)[:, 448:-448] / 16.; calls += 1
        for n, pn, dd, od, mm, om, sign in [(q, p, dl, dr, lm, rm, 1), (p, q, dr, dl, rm, lm, -1)]:
            z, v, disparity = old.depths(dd, od, mm, om, sign)
            pred[n] = dict(depth=z, valid=v, disparity=disparity, CAD=masks[n], partner=pn, sign=sign)
    assert calls == 4
    return pred

def main():
    cfg = json.loads((D / 'protocol.json').read_text()); guard(cfg)
    cv2.setNumThreads(1); assert cv2.__version__ == '4.6.0'
    allrows = []; seal = []; array_exact = point_exact = calls = 0
    for row in cfg['rows']:
        with np.load(row['input_rgb_frame']) as f:
            rgb = {n: f[n + '_rgb'] for n in old.NAMES}
        with np.load(row['prediction_path']) as f:
            masks = {n: f[n + '_CAD'] for n in old.NAMES}
            body = f['body_estimated']; timestamp = float(f['time_s'])
        inverted = {n: a.copy() for n, a in rgb.items()}
        edited = {}
        for n in old.NAMES:
            assert masks[n].dtype == bool and masks[n].shape == (480, 640)
            inverted[n][masks[n]] = 255 - inverted[n][masks[n]]
            assert np.array_equal(rgb[n][~masks[n]], inverted[n][~masks[n]])
            assert np.all(rgb[n][masks[n]] != inverted[n][masks[n]])
            edited[n] = int(masks[n].sum())
        outputs = {}; times = {}
        for arm, images in [('original', rgb), ('self_inverted', inverted)]:
            begin = time.monotonic(); pred = predict(images, masks); calls += 4
            records, arrays = old.points(pred, body, row['frame'], timestamp)
            for n in old.NAMES:
                for key in ['depth', 'valid', 'disparity', 'CAD']:
                    arrays[n + '_' + key] = pred[n][key]
            arrays.update(body_estimated=body, time_s=np.asarray(timestamp))
            if arm == 'original':
                with np.load(row['prediction_path']) as saved:
                    for n in old.NAMES:
                        for key in ['depth', 'valid', 'disparity', 'CAD', 'world', 'query_pixel', 'partner_pixel']:
                            x, y = arrays[n + '_' + key], saved[n + '_' + key]
                            assert x.dtype == y.dtype and x.shape == y.shape and x.tobytes() == y.tobytes(), (n, key)
                            if key in ['world', 'query_pixel', 'partner_pixel']: point_exact += 1
                            else: array_exact += 1
            times[arm] = time.monotonic() - begin
            path = D / (row['sequence'] + '_' + str(row['frame']).zfill(3) + '_' + arm + '.npz')
            np.savez_compressed(path, **arrays)
            seal.append(dict(path=str(path), sha256=sha(path), sequence=row['sequence'], frame=row['frame'], arm=arm))
            outputs[arm] = arrays
        allrows.append((row, outputs, edited, times))
        print('DONE', row['sequence'], row['frame'], flush=True)
    guard(cfg); assert calls == 32 and array_exact == 64 and point_exact == 48
    put(D / 'prediction_seal.json', dict(utc=now(), complete=True, outputs=seal, baseline_arrays_exact=array_exact,
        baseline_point_arrays_exact=point_exact, matcher_calls=calls, GT_read=False, new_CAD=0))
    results = []
    for row, arms, edited, times in allrows:
        a, b = arms['original'], arms['self_inverted']; eyes = []
        # Native BT/block conservative support: rotated x+/-4,y+/-3;
        # np.rot90 maps this to original row+/-4,column+/-3.
        blocked = {n: cv2.dilate(a[n + '_CAD'].astype(np.uint8), np.ones((9, 7), np.uint8)).astype(bool) for n in old.NAMES}
        for n in old.NAMES:
            partner = next(p if n == q else q for q, p in old.PAIRS if n in [q, p])
            va, vb, za, zb = [x for x in [a[n + '_valid'], b[n + '_valid'], a[n + '_depth'], b[n + '_depth']]]
            q, p = a[n + '_query_pixel'], a[n + '_partner_pixel']
            y, x, py, px = q // 640, q % 640, p // 640, p % 640
            border = (y >= 4) & (y < 476) & (py >= 4) & (py < 476) & (x >= 3) & (x < 637) & (px >= 3) & (px < 637)
            protected = border & ~blocked[n].ravel()[q] & ~blocked[partner].ravel()[p]
            both = va & vb; delta = np.zeros(va.shape, np.float64)
            delta[both] = np.abs(za[both].astype(float) - zb[both].astype(float))
            changed = ~vb.ravel()[q] | (delta.ravel()[q] > .15)
            eyes.append(dict(eye=n, full_pixels=307200, edited_known_self_pixels=edited[n], original_valid=int(va.sum()),
                inverted_valid=int(vb.sum()), validity_changes=int((va != vb).sum()), old_risk_points=len(q),
                old_risk_points_lost_or_shift_gt15cm=int(changed.sum()), protected_risk_points=int(protected.sum()),
                protected_risk_points_lost_or_shift_gt15cm=int((changed & protected).sum()),
                protected_changed_query_pixel=q[changed & protected].tolist(),
                inverted_risk_points=len(b[n + '_query_pixel'])))
        results.append(dict(sequence=row['sequence'], frame=row['frame'], eyes=eyes, matcher_LR_points_wall_s=times))
    keys = ['full_pixels', 'edited_known_self_pixels', 'original_valid', 'inverted_valid', 'validity_changes', 'old_risk_points',
            'old_risk_points_lost_or_shift_gt15cm', 'protected_risk_points', 'protected_risk_points_lost_or_shift_gt15cm', 'inverted_risk_points']
    totals = {k: sum(e[k] for r in results for e in r['eyes']) for k in keys}
    witness = totals['protected_risk_points_lost_or_shift_gt15cm'] >= 1
    guard(cfg)
    put(D / 'result.json', dict(utc=now(), complete=True, rows=results, totals=totals,
        decision='SELF_RGB_DEPENDENCY_WITNESSED_DIAGNOSTIC_ONLY' if witness else 'NO_PROTECTED_SELF_RGB_DEPENDENCY_WITNESS_ON_FIXED_FRAMES',
        witness=witness, baseline_arrays_exact=64, baseline_point_arrays_exact=48, matcher_calls=32,
        GT_read=False, physics=0, fit=0, new_CAD=0, numerical_accuracy_or_free_space_or_repair_admission=False,
        scope='Only CAD-interior RGB inverted. Original outside-CAD RGB, masks, padding, matcher/pose/thresholds unchanged. Protected query and original partner fulllocal BT/block rectangles contain no CAD pixels and stay inside image. Not wholeSGM support independence, mask calibration proof, error measurement, benefit, semantics or deployable repair.'))

if __name__ == '__main__':
    main()
