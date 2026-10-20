"""Fit0 dataset prerequisite. Old sealed features first; offline labels second.

No source invocation, map update, model, scene runtime, or distance correction.
The label is renderer-query depth agreement, NOT RGB visibility or obstacle truth.
"""
from pathlib import Path
import datetime
import hashlib
import json
import os
import numpy as np

D = Path(__file__).resolve().parent
NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p): return json.loads(Path(p).read_text())
def put(p, value):
    Path(p).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
def now(): return datetime.datetime.now(datetime.timezone.utc).isoformat()

def main():
    cfg = load(D / 'protocol.json')
    assert not (D / 'feature_reference_seal.json').exists()
    for p, h in cfg['metadata_sha256'].items(): assert sha(p) == h, p
    old = load(cfg['feature_seal'])
    assert old['complete'] and len(old['rows']) == 470
    rows = []
    total = 0
    # Only causal feature/prediction data here. No raw frame/depth/static label read.
    for r in old['rows']:
        for p, h in [(r['feature_path'], r['feature_sha256']),
                     (r['prediction_path'], r['prediction_sha256'])]:
            assert sha(p) == h, p
        with np.load(r['feature_path']) as a, np.load(r['prediction_path']) as p:
            assert np.isfinite(a['features']).all() and a['features'].shape[1] == 24
            eye = a['source_eye']; uid = a['source_query_uid']; cell = a['source_cell_row']
            assert len(eye) == len(uid) == len(cell) == r['source_points']
            assert a['time_s'] == r['time_s'] == p['time_s']
            assert (cell >= 0).all() and (cell < len(a['features'])).all()
            assert np.isfinite(a['features'][cell]).all()
            for e, name in enumerate(NAMES):
                q = uid[eye == e]
                assert np.array_equal(q, p[name + '_query_pixel'])
                assert np.array_equal(a['source_partner_uid'][eye == e], p[name + '_partner_pixel'])
                z = p[name + '_depth'].ravel()[q]
                d = p[name + '_disparity'].ravel()[q]
                assert np.isfinite(z).all() and np.isfinite(d).all()
                assert ((z >= .5) & (z <= 3)).all()
            total += len(uid)
        rows.append(dict(r, point_input_channels='24 sealed cell features gathered by source_cell_row plus query axialZ and signed disparity; 26 finite channels', teacher_read=False))
    assert total == old['original_source_points'] == 2985832
    put(D / 'feature_reference_seal.json', dict(complete=True, utc=now(), rows=rows,
        points=total, teacher_read=False, feature_generation=0,
        original_feature_seal_sha256=sha(cfg['feature_seal']),
        scope='References and complete per-point input finite/identity validation; no duplicate RGB/feature production. Full original source UID domain.'))
    print('FEATURE_REFERENCES_SEALED', len(rows), total, flush=True)
    teachers = load(cfg['old_static_label_seal'])
    assert teachers['complete']
    lookup = {(r['sequence'], r['frame']): r for r in teachers['rows']}
    assert len(lookup) == len(rows)
    (D / 'labels').mkdir()
    summary = {}; seqs = {}; eyes = {}; result_rows = []
    for index, r in enumerate(rows):
        raw = r['input_rgb_frame']; assert sha(raw) == r['input_sha256'], raw
        tr = lookup[r['sequence'], r['frame']]
        assert sha(tr['label_file']) == tr['sha256']
        assert tr['feature_sha256'] == r['feature_sha256']
        with np.load(r['feature_path']) as a, np.load(r['prediction_path']) as p, np.load(raw) as t, np.load(tr['label_file']) as s:
            assert np.array_equal(s['cells'], a['cells'])
            eye = a['source_eye']; uid = a['source_query_uid']; cell = a['source_cell_row']
            static = s['label'][cell]
            n = len(uid); y = np.full(n, -1, np.int8)
            status = np.zeros(n, np.int8)  # 0 invalid GT; 1 finite margin; 2 positive; 3 negative
            residual = np.full(n, np.nan, np.float64)
            gt_kind = np.zeros(n, np.int8)  # 0 +finite; 1 zero; 2 negativefinite; 3 +inf; 4 -inf; NaN forbidden
            for e, name in enumerate(NAMES):
                ix = np.flatnonzero(eye == e); q = uid[ix]
                gt = t[name + '_depth_label'].ravel()[q].astype(np.float64)
                z = p[name + '_depth'].ravel()[q].astype(np.float64)
                assert not np.isnan(gt).any()
                ok = np.isfinite(gt) & (gt > 0)
                k = np.zeros(len(ix), np.int8)
                k[gt == 0] = 1; k[np.isfinite(gt) & (gt < 0)] = 2
                k[np.isposinf(gt)] = 3; k[np.isneginf(gt)] = 4; gt_kind[ix] = k
                ray = np.sqrt(1 + ((q % 640 - 319.5) / cfg['focal_px']) ** 2 + ((q // 640 - 239.5) / cfg['focal_px']) ** 2)
                er = np.full(len(ix), np.nan); er[ok] = abs(z[ok] - gt[ok]) * ray[ok]
                labels = np.full(len(ix), -1, np.int8)
                labels[ok & (er <= cfg['positive_max_m'])] = 1
                labels[ok & (er >= cfg['negative_min_m'])] = 0
                y[ix] = labels; residual[ix] = er
                st = np.zeros(len(ix), np.int8); st[ok] = 1; st[labels == 1] = 2; st[labels == 0] = 3; status[ix] = st
            assert np.all(y[status == 0] == -1)
            counts = dict(points=n, positive=int((y == 1).sum()), negative=int((y == 0).sum()),
                unknown_invalid=int((status == 0).sum()), unknown_margin=int((status == 1).sum()))
            for k, v in [('zero', 1), ('negative_finite', 2), ('positive_infinity', 3), ('negative_infinity', 4)]:
                counts['GT_' + k] = int((gt_kind == v).sum())
            for domain, mask in [('static_negative_proxy', static == 0), ('static_positive_proxy', static == 1), ('static_margin_unknown', static == -1)]:
                counts[domain + '_points'] = int(mask.sum())
                counts[domain + '_known'] = int((mask & (y != -1)).sum())
                counts[domain + '_positive'] = int((mask & (y == 1)).sum())
                counts[domain + '_negative'] = int((mask & (y == 0)).sum())
                counts[domain + '_invalid_unknown'] = int((mask & (status == 0)).sum())
                counts[domain + '_margin_unknown'] = int((mask & (status == 1)).sum())
            for target, key in [(summary, r['group']), (seqs, r['sequence'])]:
                v = target.setdefault(key, dict(frames=0, positive_frames=0, negative_frames=0))
                v['frames'] += 1; v['positive_frames'] += int((y == 1).any()); v['negative_frames'] += int((y == 0).any())
                for k, value in counts.items(): v[k] = v.get(k, 0) + value
            for e, name in enumerate(NAMES):
                v = eyes.setdefault(r['sequence'] + '/' + name, dict(points=0, positive=0, negative=0, unknown_invalid=0, unknown_margin=0))
                mask = eye == e
                for k, mm in [('points', np.ones(n, bool)), ('positive', y == 1), ('negative', y == 0), ('unknown_invalid', status == 0), ('unknown_margin', status == 1)]: v[k] += int((mask & mm).sum())
            dest = D / 'labels' / (r['sequence'] + '_' + Path(raw).name)
            np.savez_compressed(dest, label=y, label_status=status, GT_value_kind=gt_kind,
                query_radial_residual_m=residual, source_eye=eye, query_uid=uid,
                static_cell_proxy_label=static, source_cell_row=cell, time_s=r['time_s'])
            result_rows.append(dict(sequence=r['sequence'], group=r['group'], frame=r['frame'],
                feature_path=r['feature_path'], feature_sha256=r['feature_sha256'],
                input_path=raw, input_sha256=r['input_sha256'], prediction_path=r['prediction_path'],
                prediction_sha256=r['prediction_sha256'], static_label_path=tr['label_file'],
                static_label_sha256=tr['sha256'], label_path=str(dest), label_sha256=sha(dest), **counts))
        if index % 50 == 0:
            put(D / 'progress.json', dict(utc=now(), frames=index + 1, last_input=raw))
            print('LABELS', index + 1, r['sequence'], flush=True)
    gates = {}
    for key, v in summary.items():
        v['known_fraction'] = (v['positive'] + v['negative']) / v['points']
        for name in ['static_negative_proxy', 'static_positive_proxy', 'static_margin_unknown']:
            den = v[name + '_points']; v[name + '_known_fraction'] = v[name + '_known'] / den if den else None
        gates[key] = dict(all_candidate_known_ge90pct=v['known_fraction'] >= .9,
            error_proxy_known_ge90pct=v['static_negative_proxy_known_fraction'] is not None and v['static_negative_proxy_known_fraction'] >= .9,
            positive_instances_ge50=v['positive'] >= 50, negative_instances_ge50=v['negative'] >= 50,
            positive_frames_ge11=v['positive_frames'] >= 11, negative_frames_ge11=v['negative_frames'] >= 11)
    passed = all(all(v.values()) for v in gates.values())
    put(D / 'label_seal.json', dict(complete=True, utc=now(), rows=result_rows,
        feature_reference_seal_sha256=sha(D / 'feature_reference_seal.json'), protocol_sha256=sha(D / 'protocol.json')))
    result = dict(complete=True, utc=now(), decision='FINITE_QUERY_DATA_NECESSARY_GATES_PASSED_NOT_FIT_ADMISSION' if passed else 'NOT_QUALIFIED_BEFORE_FIT',
        qualified=passed, frames=len(rows), points=total, groups=summary, sequences=seqs, per_eye=eyes,
        gates=gates, rows=result_rows, label_scope=cfg['label_scope'], consumer_scope=cfg['consumer'],
        model=None, source_calls=0, fit=0, learned_inference=0, physics=0, navigation=0, map_replay=0,
        fresh_holdout=False, original_outputs_changed=False, whole_research_complete=False,
        label_seal_sha256=sha(D / 'label_seal.json'))
    put(D / 'result.json', result)
    print('RESULT', result['decision'], json.dumps(summary), flush=True)

if __name__ == '__main__': main()
