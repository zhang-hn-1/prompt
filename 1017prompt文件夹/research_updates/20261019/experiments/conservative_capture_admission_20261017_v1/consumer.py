"""Joint new-capture/cell admission; never source authenticity or observed free.

The underlying fixed B engine sees ONLY admitted new cells, once. It retains its
original recent10s/two-admitted-epoch persistence. Rejected capture identities
are consumed by metadata guards, not queued for future history insertion.
"""
import importlib.util
from pathlib import Path
import numpy as np
P = Path(__file__).resolve().parent.parent / 'persistent_entry_data_qualification_20261017_v1/consumer_repaired.py'
spec = importlib.util.spec_from_file_location('unchanged_admission_B_engine', P)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
EntryState = m.EntryState

class CaptureAdmission:
    def __init__(self, shape=(400, 400)):
        self.engine = EntryState(shape)
        self.original_cells = None
        self.original_scores = None
        self.last_withheld = None

    def update(self, frame, source_time, now, cells, withhold_scores, decision_frame, decision_time):
        g = self.engine
        # Validate causal metadata before accessing cells/scores or changing state.
        if isinstance(frame, bool) or not isinstance(frame, (int, np.integer)) or frame < 0 or isinstance(decision_frame, bool) or not isinstance(decision_frame, (int, np.integer)) or decision_frame != frame or not np.isfinite([source_time, now, decision_time]).all() or source_time > now or decision_time != source_time or frame < g.frame or source_time < g.source_time or now < g.now or (frame == g.frame and source_time != g.source_time) or (frame > g.frame and source_time <= g.source_time):
            raise ValueError('causal immutable capture metadata required before payload')
        cells = np.asarray(cells); scores = np.asarray(withhold_scores, float)
        if cells.ndim != 2 or cells.shape[1] != 2 or not np.issubdtype(cells.dtype, np.integer) or scores.shape != (len(cells),) or np.any(np.isfinite(scores) & ((scores < 0) | (scores > 1))) or np.any(np.isinf(scores)) or np.any(cells < 0) or np.any(cells >= np.array(g.shape)) or len(np.unique(cells, axis=0)) != len(cells):
            raise ValueError('joint current unique cells and finite [0,1]/NaN scores required')
        if frame == g.frame and (not np.array_equal(cells, self.original_cells) or not np.array_equal(scores, self.original_scores, equal_nan=True)):
            raise ValueError('same original capture payload or admission decision changed')
        withhold = np.isfinite(scores) & (scores >= .95)
        accepted = cells[~withhold]
        labels, clock = g.update(frame, source_time, now, accepted, np.ones(len(accepted)), decision_frame, decision_time)
        self.original_cells = cells.copy(); self.original_scores = scores.copy(); self.last_withheld = withhold.copy()
        assert not np.any(labels == 0)
        return labels, clock

def interface_checks():
    c = np.array([[1, 1], [2, 2]], int)
    # all HOLD/NaN reproduces baseline; repeated source does not advance epochs.
    b = EntryState((4, 4)); q = CaptureAdmission((4, 4))
    for f, t, scores in [(0, 0., [0., np.nan]), (1, .5, [0., 0.]), (1, .8, [0., 0.])]:
        st = 0. if f == 0 else .5
        x, xc = b.update(f, st, t, c, [1., 1.], f, st)
        y, yc = q.update(f, st, t, c, scores, f, st)
        assert np.array_equal(x, y) and np.array_equal(xc, yc)
    assert q.engine.epochs[1, 1] == 2
    # Rejecting a NEW source at an accepted old cell cannot clear/refresh history.
    q = CaptureAdmission((4, 4)); q.update(0, 0., 0., c[:1], [0.], 0, 0.)
    y, _ = q.update(1, .5, .5, c, [1., 1.], 1, .5)
    assert y[1, 1] == 1 and y[2, 2] == -1
    assert q.engine.last[1, 1] == 0 and q.engine.epochs[1, 1] == 1
    y, _ = q.update(2, 11., 11., np.empty((0, 2), int), [], 2, 11.)
    assert y[1, 1] == y[2, 2] == -1 and not np.isfinite(q.engine.persistent).any()
    before = (q.engine.last.copy(), q.engine.epochs.copy())
    try: q.update(1, .5, 11., c, [0., 0.], 1, .5)
    except ValueError: pass
    else: raise AssertionError('rejected original capture resurrected as history')
    assert np.array_equal(before[0], q.engine.last) and np.array_equal(before[1], q.engine.epochs)
    # Full original payload decisions immutable, including frames entirely withheld.
    q = CaptureAdmission((4, 4)); q.update(0, 0., 0., c, [1., 1.], 0, 0.)
    try: q.update(0, 0., .1, c, [0., 1.], 0, 0.)
    except ValueError: pass
    else: raise AssertionError('duplicate admission changed')
    class Poison:
        def __array__(self, *args, **kwargs): raise AssertionError('future payload accessed')
    try: q.update(1, 2., 1., Poison(), Poison(), 1, 2.)
    except ValueError: pass
    else: raise AssertionError('future source admitted')
    return dict(all_HOLD_NaN_matches_B=True, joint_cell_not_independent_sources=True,
        duplicate_epoch_exact=True, rejected_full_capture_immutable=True,
        rejected_capture_no_later_resurrection=True, no_old_history_clear_or_refresh=True,
        future_before_payload=True, no_observed_free=True)
