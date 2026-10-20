"""Fixed postseal complete-domain scoring; ORIGINAL unfiltered support denominator."""
from common import *
from scipy.spatial import cKDTree
NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right']
def main():
    cfg = load(D / 'protocol.json'); seal = load(D / 'replay_seal.json'); assert seal['complete']
    action_seal = load(D / 'action_seal.json')
    assert sha(D / 'action_data_prerequisite.json') == action_seal['action_data_prerequisite_sha256']
    data_prerequisite = load(D / 'action_data_prerequisite.json')
    fs = load(cfg['feature_seal']); results = {}; oldmatches = 0
    for seq in cfg['sequences']:
        with np.load(seq['occupancy']) as a:
            origin = a['global_origin_xy']; truth = cKDTree(np.argwhere(a['grid'] > 0) * float(a['resolution_m']) + origin)
        exposures = [r for r in seal['rows'] if r['sequence'] == seq['name']]
        features = [r for r in fs['rows'] if r['sequence'] == seq['name']]
        times = [dict(time_s=r['time_s']) for r in exposures] if seq['name'] == 'E0' else load(seq['policy_records'])
        prior = load(seq['prior_result']); targetrows = None if seq['name'] == 'E0' else prior['source_rows']
        latest = set(); ever = set(); index = 0; frames = []
        metrics = {k: dict(false_cell_seconds=0., active_cell_seconds=0., blind_supported_queries=0, blind_missing=0) for k in ['B', 'Q']}
        for pi, pol in enumerate(times):
            t = float(pol['time_s'])
            while index < len(exposures) and exposures[index]['time_s'] <= t + 1e-8:
                rr = exposures[index]; assert sha(rr['path']) == rr['sha256']
                with np.load(rr['path']) as a:
                    baseorigin = a['origin'].copy()
                    last = {k: a[k + '_last'].copy() for k in ['B', 'Q']}
                    persistent = {k: a[k + '_persistent'].copy() for k in ['B', 'Q']}
                if seq['name'] == 'E0':
                    with np.load(Path(seq['target_root']) / f'frame_{index:03d}.npz') as a:
                        tcells = a['target_union']; T = tcells * .1 + a['origin']; targetorigin = a['origin'].copy()
                    with np.load(features[index]['feature_path']) as a: xy = a['cells'] * .1 + a['origin']
                else:
                    tcells = np.array(targetrows[index]['target_cells_array'], int).reshape(-1, 2)
                    T = tcells * .1 + origin; targetorigin = origin
                    with np.load(features[index]['prediction_path']) as a: xy = np.concatenate([a[n + '_world'][:, :2] for n in NAMES])
                supported = cKDTree(xy).query(T)[0] <= .15 if len(xy) else np.zeros(len(T), bool)
                latest = set(map(tuple, tcells.tolist())); ever.update(map(tuple, tcells[supported].tolist())); index += 1
            assert index > 0
            blind = np.array(sorted(ever - latest), int).reshape(-1, 2) * .1 + targetorigin
            dt = (times[pi + 1]['time_s'] - t if pi + 1 < len(times) else 0.) if seq['name'] == 'E0' else .1
            row = dict(policy=pi, time_s=t, blind_supported_queries=len(blind))
            for k in ['B', 'Q']:
                active = (np.isfinite(last[k]) & (t - last[k] <= 10.)) | persistent[k]
                labels = np.full((400, 400), -1, np.int8); labels[active] = 1
                if k == 'B' and seq['old_B_maps']:
                    with np.load(Path(seq['old_B_maps']) / f'frame_{pi:03d}.npz') as a:
                        assert np.array_equal(labels, a['B_labels'] if seq['name'] == 'E0' else a['labels'])
                    oldmatches += 1
                xy = np.argwhere(active) * .1 + baseorigin
                false = int((truth.query(xy)[0] > .15).sum()) if len(xy) else 0
                missing = int((cKDTree(xy).query(blind)[0] > .15).sum()) if len(xy) else len(blind)
                v = metrics[k]; v['false_cell_seconds'] += false * dt; v['active_cell_seconds'] += len(xy) * dt
                v['blind_supported_queries'] += len(blind); v['blind_missing'] += missing
                row[k] = dict(active=len(xy), false=false, blind_missing=missing)
            frames.append(row)
        for v in metrics.values(): v['blind_missing_fraction'] = v['blind_missing'] / v['blind_supported_queries'] if v['blind_supported_queries'] else None
        b, q = metrics['B'], metrics['Q']
        reduction = 1 - q['false_cell_seconds'] / b['false_cell_seconds'] if b['false_cell_seconds'] else None
        drop = q['blind_missing_fraction'] - b['blind_missing_fraction'] if b['blind_missing_fraction'] is not None else None
        if seq['name'] in ['E0', 'E1B']:
            expected = prior['aggregate']['B']['false_cell_seconds'] if seq['name'] == 'E0' else prior['memory']['false_map_cell_seconds']
            assert abs(b['false_cell_seconds'] - expected) < 1e-6
            supportedprior = load(seq['supported_result']) if seq['name'] == 'E1B' else prior['aggregate']['B']
            den = supportedprior['blind_previously_source_supported_target_queries'] if seq['name'] == 'E1B' else supportedprior['blind_historical_target_queries']
            miss = supportedprior['missing_queries'] if seq['name'] == 'E1B' else supportedprior['blind_missing_queries']
            assert (b['blind_supported_queries'], b['blind_missing']) == (den, miss)
        gates = dict(false_reduction_ge20pct=reduction is not None and reduction >= .2,
            blind_missing_increase_le5pp=drop is not None and drop <= .05,
            UNKNOWN_new_sources_not_withheld=seal['physical_UNKNOWN_withheld'] == 0,
            interface_invariants=all(seal['fixtures'].values()))
        results[seq['name']] = dict(metrics=metrics, false_reduction=reduction, blind_missing_increase=drop,
            gates=gates, passed=all(gates.values()), rows=frames)
        print('SCORED', seq['name'], reduction, drop, gates, flush=True)
    assert oldmatches == 1010 and sum(len(r['rows']) for r in results.values()) == 1910
    policy_feasible = all(r['passed'] for r in results.values())
    passed = policy_feasible and data_prerequisite['qualified']
    put(D / 'result.json', dict(complete=True, utc=now(),
        decision='PRIVILEGED_POLICY_AND_ACTION_DATA_NECESSARY_GATES_PASSED_NOT_FIT_ADMISSION' if passed else 'STOP_FIXED_ADMISSION_COMBINATION_BEFORE_FIT',
        necessary_passed=passed, privileged_policy_feasible=policy_feasible,
        action_data_prerequisite_qualified=data_prerequisite['qualified'],
        action_data_prerequisite_sha256=sha(D / 'action_data_prerequisite.json'),
        sequences=results, B_original_policy_maps_exact=oldmatches,
        full_exposures=470, full_evaluation_times=1910, scope=cfg['scope'],
        v111_v114_physical_label_rejections_unchanged=True, fit=0, learned_inference=0,
        source_regeneration=0, physics=0, navigation=0, online_default_changed=False,
        replay_seal_sha256=sha(D / 'replay_seal.json'), protocol_sha256=sha(D / 'protocol.json')))
if __name__ == '__main__': main()
