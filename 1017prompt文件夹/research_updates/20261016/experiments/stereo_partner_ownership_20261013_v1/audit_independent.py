"""Independent read-only reproduction; writes only independent_review.json here."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import numpy as np

D = Path(__file__).resolve().parent
seal = json.loads((D / 'prediction_seal.json').read_text())
result = json.loads((D / 'result.json').read_text())
protocol = json.loads((D / 'protocol.json').read_text())
truth_hashes = json.loads((D / 'truth_scoring_sha.json').read_text())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
checks = {}
for label, hashes in [('inputs', seal['input_sha256']), ('outputs', seal['output_sha256']),
                      ('sources', seal['source_sha256']), ('truth', truth_hashes)]:
    assert all(sha(p) == value for p, value in hashes.items()), label
    checks[label + '_hashes_matched'] = len(hashes)
assert set(seal['output_sha256']) == {str(p) for p in (D / 'predictions').glob('*.npz')}
gate = protocol['gate']
assert [r['name'] for r in seal['rows']] == ['H', 'E0', 'E1']
assert [r['sequence'] for r in result['results']] == ['H', 'E0', 'E1']
groups = []
for row, reported in zip(seal['rows'], result['results']):
    root = Path(row['directory'])
    name = row['name']
    assert name == reported['sequence']
    paths = sorted((root / 'source_predictions').glob('*.npz'))
    assert [p.stem for p in paths] == [f'frame_{i:03d}' for i in range(0, 901, 5)]
    n = retained = obstacle = obstacle_kept = nohit = nohit_kept = 0
    times = []
    all_errors, kept_errors = [], []
    for p in paths:
        with np.load(root / 'sensor_packets' / p.name.replace('frame_', 'source_')) as s, \
             np.load(D / 'predictions' / f'{name}_{p.name}') as v, np.load(p) as pred:
            q, partner = s['query_pixel_id'], s['partner_pixel_id']
            assert np.array_equal(v['query'], q) and np.array_equal(v['partner'], partner)
            counts = Counter(map(int, partner))
            multiplicity = np.array([counts[int(r)] for r in partner])
            keep = multiplicity == 1
            assert np.array_equal(v['claimants'], multiplicity)
            assert np.array_equal(v['keep'], keep)
            assert len(set(map(int, q))) == len(q)
            assert int(v['frame']) == int(s['frame']) == int(pred['frame']) == int(p.stem[6:])
            assert float(v['time_s']) == float(s['time_s']) == float(pred['time_s'])
            # Simulator clock accumulates the float32 step; 90 s differs by ~2 microseconds.
            assert np.isclose(float(pred['time_s']), int(pred['frame']) / 10, rtol=0, atol=5e-6)
            times.append(float(pred['time_s']))
            assert not bool(v['certified_free'])
            x, y = q % 640, q // 640
            assert ((x % 2 == 0) & (y % 2 == 0)).all()
            z = pred['depth_m'][y, x]
            assert np.isfinite(z).all() and ((z >= .5) & (z <= 3)).all()
        with np.load(root / 'frames' / p.name) as f:
            gt = f['left_depth_m'][y, x]
            finite = np.isfinite(gt) & (gt > 0)
            err = np.abs(z - gt)
            all_errors.append(err[finite])
            kept_errors.append(err[finite & keep])
            # Independent componentwise world-height calculation.
            transform = f['left_camera_usd_world']
            safe_gt = np.where(finite, gt, 0)
            height = (transform[2, 3] + transform[2, 0] * ((x - 319.5) / 584.8888888888889) * safe_gt
                      - transform[2, 1] * ((y - 239.5) / 584.8888888888889) * safe_gt
                      - transform[2, 2] * safe_gt)
            obs = finite & (height > .05) & (height <= 1.5)
            obstacle += int(obs.sum()); obstacle_kept += int((obs & keep).sum())
            nohit += int(np.isposinf(gt).sum()); nohit_kept += int((np.isposinf(gt) & keep).sum())
        n += len(q); retained += int(keep.sum())
    before, after = np.concatenate(all_errors), np.concatenate(kept_errors)
    expected = dict(candidates=n, retained=retained, rejected=n-retained,
                    finite_before=len(before), finite_after=len(after), obstacle_before=obstacle,
                    obstacle_after=obstacle_kept, infinite_truth_before=nohit, infinite_truth_after=nohit_kept,
                    candidate_retention=retained/n, obstacle_retention=obstacle_kept/obstacle,
                    conditional_scoring_coverage_before=len(before)/n,
                    conditional_scoring_coverage_after=len(after)/retained,
                    full_image_scored_fraction_after=len(after)/(len(paths)*640*480),
                    before_percentiles=np.quantile(before,[.5,.9,.95]).tolist(),
                    after_percentiles=np.quantile(after,[.5,.9,.95]).tolist(),
                    above030_fraction=float(np.mean(after > .3)))
    for key, value in expected.items():
        assert np.allclose(value, reported[key], rtol=0, atol=1e-12), (name, key)
    passed = (len(after) >= gate['finite_scored_min']
              and expected['after_percentiles'][1] <= gate['p90_m_max']
              and expected['above030_fraction'] <= gate['outlier030_fraction_max']
              and expected['obstacle_retention'] >= gate['positive_height_obstacle_retention_min']
              and expected['candidate_retention'] >= gate['candidate_retention_min'])
    assert passed == reported['source_gate']
    assert np.all(np.diff(times) > 0)
    groups.append(dict(sequence=name, epochs=len(paths), all_identities_and_counts_matched=True,
                       exact_frame_coverage='0..900 in steps of 5', first_time_s=times[0],
                       last_time_s=times[-1], monotonic_clocks=True,
                       all_reported_metrics_reproduced=True, source_gate=passed,
                       before_outlier030_fraction=float(np.mean(before > .3)),
                       after_outlier030_fraction=expected['above030_fraction']))
assert result['pass_gate'] == all(r['source_gate'] for r in groups)
assert result['physics'] == result['fit'] == 0
review = dict(status='review_complete_no_blocking_correctness_findings', checks=checks, groups=groups,
              decision='reject mechanism; E0 fails fixed p90 and outlier gates; close unused physical budget',
              critical_findings=[], important_findings=[],
              observations=[
                  'Static code review: truth frames are first opened after all 543 predictions and prediction seal are written.',
                  'Hash verification supports artifact consistency; static review is not an OS-enforced information-flow proof.',
                  'New relative to cited local baseline: packet-wide right integer pixel ownership; existing LR and paired texture gates do not ensure uniqueness.',
                  'Scope is CAD/range/outside-body/stride2 selected source packet only. Integer collisions can occur for valid surfaces.',
                  'Scoring is conditional on finite positive GT; infinite truth counts are reported separately and no full-image or navigation accuracy is established.',
                  'H/E0/E1 are previously inspected development groups, with no heldout/generalization claim.',
                  'No source-generated free-space or deletion of old risk is performed by the reviewed ownership output.',
                  'check_ownership.py prints 8 cases while its code contains four ordinary, three invalid-input, and one immutability/free-space scenario.'
              ],
              audited_artifact_sha256={str(D / n):sha(D / n) for n in ['ownership.py','protocol.json','run.py','check_ownership.py','result.json','prediction_seal.json','truth_scoring_sha.json']},
              reproduce='PYTHONDONTWRITEBYTECODE=1 /home/an/miniconda3/envs/ball_wm/bin/python experiments/stereo_partner_ownership_20261013_v1/audit_independent.py')
(D / 'independent_review.json').write_text(json.dumps(review, indent=2) + '\n')
print(json.dumps(dict(status=review['status'], checks=checks, groups=groups, decision=review['decision']), indent=2))
