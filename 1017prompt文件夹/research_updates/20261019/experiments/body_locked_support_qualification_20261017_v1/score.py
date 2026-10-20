"""Fixed postseal complete-domain scoring; ORIGINAL unfiltered support denominator."""
from common import *
from scipy.spatial import cKDTree
NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right']
def main():
    cfg = load(D / 'protocol.json'); seal = load(D / 'replay_seal.json'); assert seal['complete']
    for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
    action_seal = load(cfg['action_seal'])
    prediction_seal = load(D / 'prediction_seal.json'); assert prediction_seal['complete']
    assert sha(D / 'prediction_seal.json') == seal['prediction_seal_sha256']
    action_stats = {}
    for seqname in ['E0','E1B','E1A']:
        counts = dict(rows=0, WITHHOLD_NEW=0, HOLD_teacher=0, HOLD_rejected=0, physical_positive=0, positive_rejected=0, physical_UNKNOWN=0, UNKNOWN_rejected=0, physical_negative=0, negative_rejected=0)
        for a,p in zip(action_seal['rows'],prediction_seal['rows']):
            assert (a['sequence'],a['frame']) == (p['sequence'],p['frame'])
            if a['sequence'] != seqname: continue
            assert sha(a['path'])==a['sha256'] and sha(p['path'])==p['sha256']
            with np.load(a['path']) as y,np.load(p['path']) as z:
                assert np.array_equal(y['cells'],z['cells'])
                reject=np.isfinite(z['scores']) & (z['scores']>=.95); phys=y['physical_teacher_class']; hold=y['teacher_action']==0
                counts['rows']+=len(phys); counts['WITHHOLD_NEW']+=int(reject.sum()); counts['HOLD_teacher']+=int(hold.sum()); counts['HOLD_rejected']+=int((reject&hold).sum())
                for name,value in [('positive',1),('UNKNOWN',-1),('negative',0)]:
                    mask=phys==value; counts['physical_'+name]+=int(mask.sum()); counts[name+'_rejected']+=int((mask&reject).sum())
        for name in ['positive','UNKNOWN','negative']:
            den=counts['physical_'+name]; counts[name+'_rejected_fraction']=counts[name+'_rejected']/den if den else None
        counts['HOLD_retained_fraction']=1-counts['HOLD_rejected']/counts['HOLD_teacher'] if counts['HOLD_teacher'] else None
        action_stats[seqname]=counts
    assert sum(v['rows'] for v in action_stats.values())==40052
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
            HOLD_teacher_retained_ge95pct=action_stats[seq['name']]['HOLD_retained_fraction']>=.95,
            physical_positive_rejection_le5pct=action_stats[seq['name']]['positive_rejected_fraction']<=.05,
            physical_UNKNOWN_rejection_le5pct=action_stats[seq['name']]['UNKNOWN_rejected_fraction']<=.05,
            interface_invariants=all(seal['fixtures'].values()),
            selector_P90_le100ms=prediction_seal['sequence_timing'][seq['name']]['P90_s']<=.1)
        results[seq['name']] = dict(metrics=metrics, false_reduction=reduction, blind_missing_increase=drop,
            gates=gates, passed=all(gates.values()), action_metrics=action_stats[seq['name']], rows=frames)
        print('SCORED', seq['name'], reduction, drop, gates, flush=True)
    assert oldmatches == 1010 and sum(len(r['rows']) for r in results.values()) == 1910
    passed = all(r['passed'] for r in results.values())
    put(D / 'result.json', dict(complete=True, utc=now(),
        decision='BODY_LOCK_COMPATIBILITY_OFFLINE_GATES_PASSED_NOT_NAVIGATION' if passed else 'REJECT_FIXED_BODY_LOCK_COMPATIBILITY_NO_VARIANTS_OR_PHYSICAL',
        offline_development_passed=passed, sequences=results,
        B_original_policy_maps_exact=oldmatches, full_exposures=470, full_evaluation_times=1910,
        fit=0, advisory_cell_actions=40052, source_calls=0, physics=0, navigation=0,
        actual_learning_increment_proven=False, online_default_changed=False,
        E0_seen_development_not_validation=True, E1A_B_one_seen_development_scene=True,
        old_physical_known_rejections_unchanged=True, scope=cfg['scope'],
        prediction_seal_sha256=sha(D/'prediction_seal.json'),
        replay_seal_sha256=sha(D/'replay_seal.json'), protocol_sha256=sha(D/'protocol.json')))
if __name__ == '__main__': main()
