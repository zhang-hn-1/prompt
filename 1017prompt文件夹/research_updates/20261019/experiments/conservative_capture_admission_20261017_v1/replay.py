from common import *
from consumer import CaptureAdmission, EntryState, interface_checks
def main():
    cfg = load(D / 'protocol.json'); actions = load(D / 'action_seal.json')
    assert actions['complete'] and actions['teacher_used']
    fs = load(cfg['feature_seal']); out = D / 'maps'; out.mkdir(); rows = []
    fixtures = interface_checks(); bmatches = 0; action_counts = {}; invalid_changes = 0
    for seq in cfg['sequences']:
        pairs = [(f, a) for f, a in zip(fs['rows'], actions['rows']) if f['sequence'] == seq['name']]
        b = CaptureAdmission(); q = CaptureAdmission(); previous = -np.inf
        for f, a in pairs:
            assert sha(f['feature_path']) == f['feature_sha256'] and sha(a['path']) == a['sha256']
            with np.load(f['feature_path']) as feat, np.load(a['path']) as teacher:
                cells = feat['cells']; t = f['time_s']; assert t > previous; previous = t
                assert np.array_equal(cells, teacher['cells']) and teacher['time_s'] == t
                scores = teacher['teacher_action'].astype(float)
                oldlast = q.engine.last.copy(); oldepochs = q.engine.epochs.copy(); oldpersist = q.engine.persistent.copy()
                by, bc = b.update(f['frame'], t, t, cells, np.zeros(len(cells)), f['frame'], t)
                qy, qc = q.update(f['frame'], t, t, cells, scores, f['frame'], t)
                rejected = cells[scores == 1]; ix = tuple(rejected.T)
                assert np.array_equal(q.engine.last[ix], oldlast[ix]) and np.array_equal(q.engine.epochs[ix], oldepochs[ix]) and np.array_equal(q.engine.persistent[ix], oldpersist[ix])
                assert not np.any((qy == 1) & (by != 1))
                accepted = cells[scores == 0]; assert (qy[tuple(accepted.T)] == 1).all()
                invalid_changes += int(scores[teacher['physical_teacher_class'] == -1].sum())
                if seq['old_B_maps']:
                    path = Path(seq['old_B_maps']) / f'frame_{f["frame"] if seq["name"] == "E0" else 5*f["frame"]:03d}.npz'
                    with np.load(path) as z: assert np.array_equal(by, z['B_labels'] if seq['name'] == 'E0' else z['labels'])
                    bmatches += 1
                dest = out / f'{seq["name"]}_{f["frame"]:03d}.npz'
                np.savez_compressed(dest, B_labels=by, Q_labels=qy, B_last=b.engine.last, Q_last=q.engine.last,
                    B_persistent=np.isfinite(b.engine.persistent), Q_persistent=np.isfinite(q.engine.persistent),
                    B_epochs=b.engine.epochs, Q_epochs=q.engine.epochs, cells=cells,
                    physical_teacher_class=teacher['physical_teacher_class'], teacher_action=teacher['teacher_action'],
                    origin=feat['origin'], resolution=.1, time_s=t)
                rows.append(dict(sequence=f['sequence'], frame=f['frame'], time_s=t,
                    path=str(dest), sha256=sha(dest), feature_sha256=f['feature_sha256'], action_sha256=a['sha256']))
                v = action_counts.setdefault(seq['name'], dict(HOLD_BASELINE=0, WITHHOLD_NEW=0))
                v['HOLD_BASELINE'] += int((scores == 0).sum()); v['WITHHOLD_NEW'] += int((scores == 1).sum())
        print('MAPS_SEALED', seq['name'], len(pairs), flush=True)
    assert len(rows) == 470 and bmatches == 290 and invalid_changes == 0
    put(D / 'replay_seal.json', dict(complete=True, utc=now(), rows=rows, fixtures=fixtures,
        B_exposure_maps_exact=bmatches, physical_UNKNOWN_withheld=invalid_changes, action_counts=action_counts,
        teacher_policy_not_online=True, learned_inference=0, fit=0, source_calls=0, physics=0,
        action_seal_sha256=sha(D / 'action_seal.json'), protocol_sha256=sha(D / 'protocol.json')))
if __name__ == '__main__': main()
