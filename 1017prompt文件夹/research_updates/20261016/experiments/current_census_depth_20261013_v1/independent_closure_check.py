"""Reviewer-owned raw-output audit. Does not import or execute score/generate.

Writes only independent_closure.json. Reuses the unchanged memory consumer for
the separately constructed C54 replay; numeric scoring is independently written.
"""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import sys
import time
import numpy as np
from scipy.spatial import cKDTree

D = Path(__file__).resolve().parent
W = D.parents[1]
H = W / 'experiments/stereo_memory_navigation_20261004_v1'
V = W / 'experiments/viewpoint_confirmation_navigation_20261012_v1'
O = W / 'experiments/observed_end_subgoal_20261013_v1'
S = W / 'experiments/source_confidence_calibration_20261012_v1'
RISK = W / 'experiments/risk_height_navigation_20261012_v1'
GROUPS = {
    'H': (RISK / 'candidate_e2_run', RISK / 'candidate_e2_protocol.json'),
    'E0': (S / 'data_e0_run', S / 'data_e0_protocol.json'),
    'E1': (S / 'data_e1_run', S / 'data_e1_protocol.json'),
    'B54': (O / 'baseline_e1_run', O / 'baseline_e1_protocol.json'),
    'C54': (O / 'candidate_e1_run', O / 'candidate_e1_protocol.json'),
}


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def quant(values):
    return np.percentile(values, [50, 90, 95]).tolist() if len(values) else [None] * 3


compared = 0
max_float_difference = 0.0


def compare(got, expected, label):
    global compared, max_float_difference
    if isinstance(got, dict):
        for key, value in got.items():
            assert key in expected, (label, 'missing', key)
            compare(value, expected[key], label + '.' + key)
    elif isinstance(got, list):
        assert len(got) == len(expected), (label, len(got), len(expected))
        for i, (a, b) in enumerate(zip(got, expected)):
            compare(a, b, label + '[' + str(i) + ']')
    elif isinstance(got, (float, np.floating)):
        delta = abs(float(got) - expected)
        assert np.isclose(got, expected, rtol=1e-11, atol=1e-9), (label, got, expected)
        max_float_difference = max(max_float_difference, delta)
        compared += 1
    else:
        assert got == expected, (label, got, expected)
        compared += 1


def truth_points(frame, ids):
    ids = np.asarray(ids, np.int64)
    z = frame['left_depth_m'].reshape(-1)[ids]
    usable = (z > 0) & np.isfinite(z)
    q = ids[usable]
    rays_usd = np.column_stack(((q % 640 - 319.5) / 584.8888888888889,
                                -(q // 640 - 239.5) / 584.8888888888889,
                                -np.ones(len(q))))
    local = rays_usd * z[usable, None]
    transform = frame['left_camera_usd_world']
    world = np.empty_like(local)
    for axis in range(3):
        world[:, axis] = transform[axis, 3] + np.sum(local * transform[axis, :3], axis=1)
    return usable, world


def run():
    started = time.monotonic()
    seal = read(D / 'prediction_seal.json')
    initial_seal_hash = digest(D / 'prediction_seal.json')
    result = read(D / 'result.json')
    original_result_hash = digest(D / 'result.json')
    expected = {g['group']: g for g in result['groups']}
    protocol = read(D / 'protocol.json')
    gatespec = protocol['gates']
    recovery = read(D / 'recovery_registration.json')
    hashes = {}
    for section in ('inputs', 'outputs', 'sources'):
        for path, saved in seal[section].items():
            assert digest(path) == saved, ('seal', section, path)
        hashes[section] = len(seal[section])
        print('verified hashes', section, hashes[section], flush=True)
    for path, saved in recovery['preserved_prefix'].items():
        assert digest(path) == saved, ('preserved prefix', path)
    hashes['preserved_prefix'] = len(recovery['preserved_prefix'])
    truth_hashes = read(D / 'score_truth_sha.json')
    for path, saved in truth_hashes.items():
        assert digest(path) == saved, ('truth identity', path)
    hashes['scoring_truth'] = len(truth_hashes)
    assert set(expected) == set(GROUPS) == set(protocol['groups'])
    recomputed = []
    selected_map_rows = []
    for name, (root, config) in GROUPS.items():
        records = []
        error_lists = [[], []]
        location_lists = [[], []]
        totals = [dict(predicted=0, finite=0, nohit=0, accepted_points=0,
                       finite_accepted_points=0, ground_false_points=0, gt_obstacle_points=0)
                  for _ in range(2)]
        obstacle_total = obstacle_retained = 0
        ids = [int(p.stem.split('_')[1]) for p in sorted((root / 'source_predictions').glob('*.npz'))]
        assert ids == list(range(0, 901, 5))
        for index in ids:
            filename = f'frame_{index:03d}.npz'
            with np.load(root / 'frames' / filename) as frame:
                truth_depth = frame['left_depth_m']
                finite_truth = np.isfinite(truth_depth) & (truth_depth > 0)
                row = dict(frame=index, time_s=float(frame['time_s']), arms=[])
                query_sets = []
                original_obstacles = set()
                for arm in range(2):
                    predpath = root / 'source_predictions' / filename if arm == 0 else D / 'predictions' / name / filename
                    surfacepath = root / 'partial_observations' / filename if arm == 0 else D / 'predictions' / name / f'surface_{index:03d}.npz'
                    with np.load(predpath) as pred, np.load(surfacepath) as surface:
                        assert float(pred['time_s']) == row['time_s']
                        z = pred['depth_m']
                        assert z.shape == (480, 640)
                        measured = np.asarray(pred['confidence'], bool) & np.isfinite(z)
                        measured &= (z >= protocol['matching']['depth_range_m'][0]) & (z <= protocol['matching']['depth_range_m'][1])
                        scored = measured & finite_truth
                        depth_errors = np.abs(np.subtract(z[scored], truth_depth[scored]))
                        error_lists[arm].append(depth_errors)
                        q = surface['query_pixel_id']
                        assert len(q) == len(set(map(int, q)))
                        query_sets.append(set(map(int, q)))
                        good, true_world = truth_points(frame, q)
                        ground_count = int(np.count_nonzero(np.abs(true_world[:, 2]) <= .03))
                        obstacle = (true_world[:, 2] > .05) & (true_world[:, 2] <= 1.5)
                        if arm == 0:
                            original_obstacles = set(map(int, q[good][obstacle]))
                        point_delta = surface['world'][good] - true_world
                        location_errors = np.sqrt(np.sum(point_delta ** 2, axis=1))
                        location_lists[arm].append(location_errors)
                        values = dict(predicted=int(np.count_nonzero(measured)), finite=int(np.count_nonzero(scored)),
                                      nohit=int(np.count_nonzero(measured & np.isposinf(truth_depth))),
                                      accepted_points=len(q), finite_accepted_points=int(np.count_nonzero(good)),
                                      ground_false_points=ground_count, gt_obstacle_points=int(np.count_nonzero(obstacle)))
                        for k, v in values.items():
                            totals[arm][k] += v
                        row['arms'].append(dict(predicted=values['predicted'], finite=values['finite'],
                                                p50_p90_p95=quant(depth_errors),
                                                above030_fraction=float(np.count_nonzero(depth_errors > .3) / len(depth_errors)) if len(depth_errors) else None,
                                                points=len(q), finite_points=values['finite_accepted_points'],
                                                ground_false=ground_count, point_p50_p90_p95=quant(location_errors)))
                obstacle_total += len(original_obstacles)
                obstacle_retained += len(original_obstacles.intersection(query_sets[1]))
                records.append(row)
        arms = []
        for arm in range(2):
            e = np.concatenate(error_lists[arm])
            pe = np.concatenate(location_lists[arm])
            counts = totals[arm]
            arms.append(dict(arm=('baseline', 'census')[arm], **counts,
                             finite_scoring_coverage=counts['finite'] / max(1, counts['predicted']),
                             full_image_scored_fraction=counts['finite'] / (len(ids) * 480 * 640),
                             depth_p50_p90_p95=quant(e),
                             above030_fraction=float(np.count_nonzero(e > .3) / len(e)) if len(e) else None,
                             ground_false_fraction=counts['ground_false_points'] / max(1, counts['finite_accepted_points']),
                             point_error_p50_p90_p95=quant(pe)))
        bad_tail = [r['frame'] for r in records if r['arms'][1]['finite'] >= 1000 and r['arms'][1]['p50_p90_p95'][1] > gatespec['per_frame_p90_max_m_when_candidate_finite_ge1000']]
        bad_coverage = [r['frame'] for r in records if r['arms'][0]['predicted'] >= 1000 and r['arms'][1]['predicted'] < gatespec['per_frame_count_vs_baseline_min_when_baseline_ge1000'] * r['arms'][0]['predicted']]
        c = arms[1]
        computed_gates = dict(
            finite_count=c['finite'] >= gatespec['finite_depth_samples_min'],
            depth_p90=c['depth_p50_p90_p95'][1] is not None and c['depth_p50_p90_p95'][1] <= gatespec['finite_depth_p90_max_m'],
            outlier=c['above030_fraction'] is not None and c['above030_fraction'] <= gatespec['finite_error_above_0_3_max_fraction'],
            dense_coverage=c['predicted'] >= gatespec['dense_count_vs_baseline_min'] * arms[0]['predicted'],
            per_frame_coverage=len(bad_coverage) == 0,
            catastrophic_frame=len(bad_tail) == 0,
            obstacle_retention=obstacle_total > 0 and obstacle_retained >= gatespec['accepted_true_obstacle_query_retention_vs_baseline_min'] * obstacle_total,
        )
        group = dict(group=name, arms=arms, original_true_obstacle_queries=obstacle_total,
                     still_measured_obstacle_queries=obstacle_retained,
                     obstacle_query_retention=obstacle_retained / max(1, obstacle_total),
                     dense_count_ratio=c['predicted'] / max(1, arms[0]['predicted']),
                     catastrophic_frames=bad_tail, frame_coverage_failures=bad_coverage,
                     gates=computed_gates, source_gate=all(computed_gates.values()), frames=records)
        compare(group, expected[name], name)
        # Independently score every sealed selected map using reference coordinates.
        with np.load(read(config)['occupancy']) as occ:
            cells = np.column_stack(np.where(occ['grid'] > 0))
            reference = cells * float(occ['resolution_m']) + occ['global_origin_xy']
        tree = cKDTree(reference)
        maprows = []
        for index in (450, 495, 500, 899):
            pair = []
            for title in ('baseline', 'candidate'):
                with np.load(D / 'predictions' / name / f'map_{title}_{index:03d}.npz') as m:
                    x, y = np.where(m['labels'] == 1)
                    centres = m['origin'] + np.column_stack((x, y)) * float(m['resolution'])
                    distances, _ = tree.query(centres)
                    ages = float(m['time_s']) - m['clock'][x, y]
                    old = ages > 10
                    incorrect = distances > .15
                    pair.append(dict(arm=title, risk_cells=len(x), far015=int(np.count_nonzero(incorrect)),
                                     older10s=int(np.count_nonzero(old)), old_far015=int(np.count_nonzero(old & incorrect)),
                                     cell_distance_p50_p90_p95=quant(distances)))
                    map_time = float(m['time_s'])
            maprows.append(dict(frame=index, time_s=map_time, arms=pair))
        compare(maprows, expected[name]['maps'], name + '.maps')
        group.pop('frames')
        group['maps'] = maprows
        group['frame_metrics_checked'] = len(records)
        recomputed.append(group)
        print('raw metrics and selected maps exact', name, 'failed gates', [k for k,v in computed_gates.items() if not v], flush=True)
    assert result['source_pass'] == all(g['source_gate'] for g in recomputed)
    # Reconstruct C54 baseline and candidate from saved surfaces and native poses,
    # using new memory/grid instances, no generator or saved raw-memory snapshots.
    sys.path[:0] = [str(V), str(H), str(RISK), '/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
    from tentative_point_memory import TentativePointMemory
    from viewpoint_confirmation import ViewpointConfirmationGrid
    root = GROUPS['C54'][0]
    with np.load(root / 'maps/frame_000.npz') as first:
        origin = first['origin']
    grids = [ViewpointConfirmationGrid(origin, 10, static_scene_assumption=True) for _ in range(2)]
    memories = [TentativePointMemory(), TentativePointMemory()]
    surface_records = [[], []]
    viewpoints = {}
    for index in range(0, 901, 5):
        posepath = root / 'known_poses' / f'frame_{index:03d}.npz'
        with np.load(posepath) as pose:
            viewpoints[index] = dict(time_s=float(pose['time_s']), position_world=pose['left_camera_usd_world'][:3, 3])
        for arm in range(2):
            path = root / 'partial_observations' / f'frame_{index:03d}.npz' if arm == 0 else D / 'predictions/C54' / f'surface_{index:03d}.npz'
            with np.load(path) as surface:
                assert int(surface['frame']) == index
                assert float(surface['acquisition_time_s']) == viewpoints[index]['time_s']
                surface_records[arm].append(dict(frame=index, time_s=float(surface['acquisition_time_s']),
                                               world=surface['world'], query_pixel_id=surface['query_pixel_id'],
                                               partner_pixel_id=surface['partner_pixel_id'],
                                               input_sha256={str(path):digest(path), str(posepath):digest(posepath)}))
    native = {round(r['time_s'], 7): np.asarray(r['position_world']) for r in read(root / 'native_predictions.json') if r.get('available')}
    sealed_counts = next(r for r in seal['rows'] if r['group'] == 'C54')['map_counts']
    next_record = 0
    selected_checks = 0
    for index in range(900):
        with np.load(root / 'maps' / f'frame_{index:03d}.npz') as old_map:
            now = float(old_map['time_s'])
            while next_record < 181 and surface_records[0][next_record]['time_s'] <= now:
                for arm in range(2):
                    memories[arm].update(surface_records[arm][next_record], now)
                next_record += 1
            for arm in range(2):
                records = memories[arm].at(now)['records']
                grids[arm].consume_records(records, now, native[round(now, 7)], viewpoints)
            np.testing.assert_array_equal(grids[0].labels, old_map['labels'])
            np.testing.assert_array_equal(grids[0].hazard_acquisition_time, old_map['hazard_acquisition_time'])
            for arm, title in enumerate(('baseline', 'candidate')):
                assert int(np.count_nonzero(grids[arm].labels == 1)) == sealed_counts[index][title]
                if index in (450, 495, 500, 899):
                    with np.load(D / 'predictions/C54' / f'map_{title}_{index:03d}.npz') as saved:
                        np.testing.assert_array_equal(grids[arm].labels, saved['labels'])
                        np.testing.assert_array_equal(grids[arm].hazard_acquisition_time, saved['clock'])
                        selected_checks += 1
            if index % 200 == 0:
                print('C54 independent replay', index, '/900', flush=True)
    blocker_xy = np.asarray(read(O / 'blocker_provenance.json')['cell_xy'])
    blocker = []
    for arm, title in enumerate(('baseline', 'candidate')):
        g = grids[arm]
        key = tuple(g.cells(blocker_xy[None])[0])
        present = bool(g.labels[key] == 1)
        blocker.append(dict(arm=title, risk=present, age_s=float(now - g.hazard_acquisition_time[key]) if present else None))
    cohort = []
    for record in read(O / 'blocker_provenance.json')['source_epochs']:
        index = int(record['frame'].split('_')[1])
        q = np.asarray(record['pixel_ids'])
        filename = f'frame_{index:03d}.npz'
        with np.load(root / 'frames' / filename) as f, np.load(root / 'source_predictions' / filename) as old, np.load(D / 'predictions/C54' / filename) as new:
            z = new['depth_m'].reshape(-1)[q]
            gt = f['left_depth_m'].reshape(-1)[q]
            valid = new['confidence'].reshape(-1)[q] & np.isfinite(z)
            e = abs(z - gt)
            cohort.append(dict(frame=index, queries=len(q), census_measured=int(valid.sum()), census_unknown=int((~valid).sum()),
                               error_le015=int(np.count_nonzero(valid & (e <= .15))), error_gt030=int(np.count_nonzero(valid & (e > .3))),
                               old_error_quantiles=quant(abs(old['depth_m'].reshape(-1)[q] - gt)), new_error_quantiles=quant(e[valid]), source_pixels=q.tolist()))
    if (D / 'prior_blocker_cohort.json').exists():
        expected_blocker = read(D / 'prior_blocker_cohort.json')
        compare(cohort, expected_blocker['rows'], 'prior_blocker_cohort')
        compare(blocker, expected_blocker['final_original_blocker_cell'], 'blocker_map_state')
    # Confirm no audited scientific output was modified by this review.
    assert digest(D / 'result.json') == original_result_hash
    assert digest(D / 'prediction_seal.json') == initial_seal_hash
    ledger = read(D / 'ledger.json')
    assert ledger['physical_used'] == 0 and ledger['fit'] == 0 and ledger['recovery_used'] == 1
    assert not result['source_pass'] and ledger['status'] == 'source_rejected' and ledger['physical_unattempted_closed'] == 4
    report = dict(
        utc=datetime.now(timezone.utc).isoformat(), reviewer='independent_census_review',
        decision='CONFIRMED_SOURCE_REJECTION_AND_CLOSED_UNUSED_PHYSICAL_BUDGET',
        source_pass=False, source_admitted=False, navigation_admitted=False, physical=0, fit=0,
        hash_checks=hashes, numerical_fields_compared=compared, max_float_difference=max_float_difference,
        independently_scored_pairs=905, frame_arm_metric_sets=1810, selected_map_metric_sets=40,
        groups=recomputed,
        C54_replay=dict(steps=900, both_arms_count_exact=True, baseline_full_labels_and_clocks_exact_steps=900,
                        selected_full_labels_and_clocks_exact_arrays=selected_checks,
                        candidate_source_records_consumed=next_record,
                        final_map_time_s=now, memory_code_reused_unchanged=True,
                        generator_and_scorer_not_imported=True),
        prior_blocker=dict(cell_xy=blocker_xy.tolist(), rows=cohort, final_original_blocker_cell=blocker),
        identity=dict(prediction_seal_sha256=initial_seal_hash, result_sha256=original_result_hash,
                      reviewer_script_sha256=digest(__file__)),
        issues=[],
        limitations=[
            'All five trajectories are previously seen development; no independent holdout or actual navigation run.',
            'Depth metrics score only confident .5..3m predictions with finite positive rendered depth; nohit counts are reported separately, not assigned zero error or free space.',
            'Obstacle retention covers original accepted GT-obstacle query pixels, not all visible obstacles; retained correspondence does not certify location accuracy.',
            'Ground false and accepted-point location metrics use renderer pose/depth only offline. Native pose, ideal IMU, encoder, initial gauge and fixed height priors remain inherited privileges.',
            'Independent complete map replay is C54 only. Other group full baseline replay identity is supported by hashed generator assertions; this review independently rescored all 40 selected map arrays.',
            'Memory replay reuses the unchanged production memory/grid classes. Raw depth, world projection scoring, gate aggregation and map error counts are reviewer implementations.',
            '905 outputs were not regenerated with an independent real-image Census implementation; correspondence math had the earlier synthetic independent brute-force checks.',
            'Offline counterfactual map removal is unknown, not certified free space, and does not prove an actual controller would succeed.',
        ],
        elapsed_s=time.monotonic() - started,
    )
    (D / 'independent_closure.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(decision=report['decision'], compared=compared, max_float_difference=max_float_difference,
                          C54_blocker=blocker, elapsed_s=report['elapsed_s'])), flush=True)


if __name__ == '__main__':
    run()
