"""Privileged action teacher. UNKNOWN stays a separate physical label, default HOLD."""
from common import *
def main():
    cfg = load(D / 'protocol.json')
    for p, h in cfg['input_sha256'].items(): assert sha(p) == h, p
    fs = load(cfg['feature_seal']); ls = load(cfg['label_result'])
    out = D / 'actions'; out.mkdir(); rows = []; counts = {}; support = {}
    teacher_geometry = {}
    for seq in cfg['sequences']:
        with np.load(seq['occupancy']) as occ:
            teacher_geometry[seq['name']] = (occ['global_origin_xy'].copy(), float(occ['resolution_m']))
    for f, l in zip(fs['rows'], ls['rows']):
        assert (f['sequence'], f['frame']) == (l['sequence'], l['frame'])
        assert sha(f['feature_path']) == f['feature_sha256']
        assert sha(l['label_file']) == l['sha256']
        with np.load(f['feature_path']) as a, np.load(l['label_file']) as z:
            y = z['label']; assert np.array_equal(a['cells'], z['cells']) and np.isin(y, [-1, 0, 1]).all()
            assert np.isfinite(a['features']).all() and a['features'].shape == (len(y), 24)
            action = (y == 0).astype(np.int8) #0 HOLD_BASELINE;1 WITHHOLD_NEW
            assert not action[y == -1].any()
            p = out / (f['sequence'] + '_' + Path(f['feature_path']).name)
            np.savez_compressed(p, cells=a['cells'], physical_teacher_class=y,
                teacher_action=action, origin=a['origin'], time_s=a['time_s'])
            group = counts.setdefault(f['group'], dict(cells=0, physical_positive=0, physical_negative=0, physical_unknown=0, HOLD_BASELINE=0, WITHHOLD_NEW=0))
            group['cells'] += len(y)
            for code, name in [(1, 'physical_positive'), (0, 'physical_negative'), (-1, 'physical_unknown')]: group[name] += int((y == code).sum())
            group['HOLD_BASELINE'] += int((action == 0).sum()); group['WITHHOLD_NEW'] += int(action.sum())
            s = support.setdefault(f['group'], dict(HOLD_BASELINE=set(), WITHHOLD_NEW=set(), HOLD_frames=0, WITHHOLD_frames=0))
            origin, res = teacher_geometry[f['sequence']]
            teacher_cells = np.rint((a['cells'] * .1 + a['origin'] - origin) / res).astype(int)
            for code, name in [(0, 'HOLD_BASELINE'), (1, 'WITHHOLD_NEW')]: s[name].update(map(tuple, teacher_cells[action == code].tolist()))
            s['HOLD_frames'] += int((action == 0).any()); s['WITHHOLD_frames'] += int((action == 1).any())
            rows.append(dict(sequence=f['sequence'], frame=f['frame'], time_s=f['time_s'],
                path=str(p), sha256=sha(p), feature_sha256=f['feature_sha256'], label_sha256=l['sha256']))
    assert len(rows) == 470
    data_gates = {}
    for key, g in counts.items():
        s = support[key]
        for name in ['HOLD_BASELINE', 'WITHHOLD_NEW']: g['unique_' + name + '_teacher_cells'] = len(s[name])
        g['HOLD_exposures'] = s['HOLD_frames']; g['WITHHOLD_exposures'] = s['WITHHOLD_frames']
        g['physical_known_fraction'] = (g['physical_positive'] + g['physical_negative']) / g['cells']
        data_gates[key] = dict(HOLD_instances_ge50=g['HOLD_BASELINE'] >= 50,
            WITHHOLD_instances_ge50=g['WITHHOLD_NEW'] >= 50,
            HOLD_distinct_cells_ge20=g['unique_HOLD_BASELINE_teacher_cells'] >= 20,
            WITHHOLD_distinct_cells_ge20=g['unique_WITHHOLD_NEW_teacher_cells'] >= 20,
            HOLD_exposures_ge11=g['HOLD_exposures'] >= 11, WITHHOLD_exposures_ge11=g['WITHHOLD_exposures'] >= 11)
    put(D / 'action_data_prerequisite.json', dict(complete=True, utc=now(),
        qualified=all(all(g.values()) for g in data_gates.values()), counts=counts, gates=data_gates,
        physical_known90pct_gate_not_claimed=True, physical_UNKNOWN_retained=True,
        no_fit_admission=True, fresh_holdout_available=False, causal_features_finite24=True,
        note='Complete action policy defaults to HOLD on physical UNKNOWN; action counts are not physical risk labels. Distinct cell/exposure support is not independence. Counts refer to CURRENT capture cells, never promotion-eligible-only.'))
    put(D / 'action_seal.json', dict(complete=True, utc=now(), rows=rows, counts=counts,
        teacher_used=True, no_physical_UNKNOWN_relabel=True, physical_known_not_action_completeness=True,
        original_v111_and_v114_rejections_unchanged=True, fit=0, source_regeneration=0,
        action_data_prerequisite_sha256=sha(D / 'action_data_prerequisite.json'),
        protocol_sha256=sha(D / 'protocol.json')))
    print('PRIVILEGED_ACTIONS_SEALED', counts, flush=True)
if __name__ == '__main__': main()
