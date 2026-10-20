"""Immutable inputs, raw-before-label boundary, literal saved-V1 parity and counts."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent;W=H.parent.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p=json.loads((H/'multiview_cost_trace_protocol.json').read_text());run=json.loads((H/'multiview_cost_traces/manifest.json').read_text());r=json.loads((H/'multiview_cost_trace_audit.json').read_text())
    count=0
    for mapping in [p['source_and_input_sha256'],json.loads((H/'multiview_rgb_verification.json').read_text())['source_and_artifact_sha256']]:
        for path,s in mapping.items():assert sha(path)==s,path;count+=1
    for m in [run,r]:
        for path,s in m['source_sha256'].items():assert sha(path)==s,path
    assert p['sealed_utc']<run['started_utc']<run['ended_utc']<r['started_utc']<r['ended_utc']
    assert run['fidelity_pass'] and run['all_curves_saved_before_any_truth_labels']
    assert not r['new_candidate_admitted'] and r['old_candidate_rejection_preserved'] and r['new_dense_predictions']==r['online_navigation_runs']==0
    assert '6 failed' in (H/'multiview_cost_trace_red.log').read_text() and '6 passed' in (H/'multiview_cost_trace_green.log').read_text()
    assert '104 passed' in (H/'multiview_cost_trace_full_suite.log').read_text()
    q=np.load(H/'past_multiview_measurements.npz');total=0;retained=wrong=switch=gtvalid=same=visible=invalid=gtgap=0
    for row,logged in zip(p['rows'],run['rows']):
        f=row['frame'];d=np.load(logged['path']);assert sha(logged['path'])==logged['sha256'] and all(logged['checks'].values())
        assert logged['cost_max_abs_diff']==logged['interval_max_abs_diff']==0
        pix=q[f'frame_{f:03d}__source_pixels_yx'];saved=np.load(H/'multiview_rgb_predictions'/f'frame_{f:03d}.npz')
        spatial=np.round(np.linspace(0,len(pix)-1,4)).astype(int);yy,xx=pix.T;eligible=np.flatnonzero(saved['confidence'][yy,xx]);eligible=eligible[~np.isin(eligible,spatial)]
        selection=np.r_[spatial,eligible[np.round(np.linspace(0,len(eligible)-1,4)).astype(int)]]
        np.testing.assert_array_equal(d['query_indices'],selection);np.testing.assert_equal(d['anchors_yx'],pix[selection]);assert len(set(selection))==8
        assert d['source_cost'].shape==(8,128,1+2*total//8) # prior frames give1/3/5/7
        for i,a in enumerate(row['anchors']):
            total+=1;assert a['query_index']==selection[i];np.testing.assert_equal(a['pixel_yx'],pix[selection[i]])
            y,x=a['pixel_yx'];cost=d['cost'][i];b=int(d['best_index'][i]);finite=np.isfinite(cost);conf=bool(d['confidence'][i])
            np.testing.assert_equal(d['raw_depth_m'][i],saved['raw_depth_m'][y,x]);assert conf==saved['confidence'][y,x]
            assert cost[b]==saved['cost'][y,x] and d['view_bits'][i,b]==saved['view_bits'][y,x]
            assert d['interval_ray_width_m'][i]==saved['interval_ray_width_m'][y,x]
            for di in range(128):
                values=d['source_cost'][i,di];valid=np.isfinite(values);np.testing.assert_equal(valid,d['source_valid'][i,di])
                expected=(np.median(values[valid]) if valid.any() else np.inf)
                assert cost[di]==expected
                bits=sum(1<<j for j in range(len(values)) if valid[j]);assert d['view_bits'][i,di]==bits
            label=next(v for v in r['rows'] if v['frame']==f and v['query_index']==a['query_index'])
            assert label['finite_bins_n']==int(finite.sum()) and label['confidence']==conf
            assert label['competitive_bins_n']==int(d['competitive'][i].sum())
            if conf:
                retained+=1
                if label['raw_ray_error_m']>.1:
                    wrong+=1;switch+=label['winner_adjacent_source_switch'];gtvalid+=label['gt_nearest_cost'] is not None
                    same+=label['winner_bits']==label['gt_nearest_bits'];visible+=label['all_winner_sources_true_visible'];invalid+=label['winner_adjacent_invalid']
                    gtgap+=label['gt_nearest_gap'] is not None and label['gt_nearest_gap']>.05
    assert (total,retained,wrong,switch,gtvalid,same,visible,invalid,gtgap)==(32,17,16,8,11,6,12,7,11)
    assert (r['pooled']['n'],r['pooled']['retained_n'],r['pooled']['retained_wrong_n'])==(32,17,16)
    review=json.loads((H/'multiview_cost_trace_review.json').read_text());assert not review['critical'] and not review['important']
    names=['seal_multiview_cost_trace.py','multiview_cost_trace.py','test_multiview_cost_trace.py','run_multiview_cost_trace.py','audit_multiview_cost_trace.py','plot_multiview_cost_trace.py','write_multiview_cost_trace_report.py','verify_multiview_cost_trace.py','multiview_cost_trace_protocol.json','multiview_cost_trace_seal.log','multiview_cost_trace_red.log','multiview_cost_trace_green.log','multiview_cost_trace_full_suite.log','multiview_cost_trace_run.log','multiview_cost_trace_audit.log','multiview_cost_trace_audit.json','multiview_cost_trace_figures.log','multiview_cost_trace_review.json','multiview_cost_traces/manifest.json','figures/multiview_cost_trace_all_anchors.png','figures/multiview_cost_trace_anchor_rgb.png']
    paths=[H/n for n in names]+[Path(a['path']) for a in run['rows']]+[W/'reports/benchmark_multiview_cost_support_trace_20261004.md']
    out=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=count,stage_cpu_tests=104,anchors=32,
        source_depth_anchor_scores=16384,saved_dense_v1_output_parity_exact=True,max_cost_interval_diff=0,
        curves_saved_before_truth_labels=True,sampling_gt_accuracy_independent=True,sampling_half_saved_confidence_conditioned=True,
        population_or_fresh_claim=False,independent_review_no_critical_or_important=True,figures_inspected=True,
        old_v1_rejection_preserved=True,new_dense_predictions=0,new_simulations=0,new_control_runs=0,whole_research_complete=False,
        source_and_artifact_sha256={str(a):sha(a) for a in paths})
    (H/'multiview_cost_trace_verification.json').write_text(json.dumps(out,indent=2)+'\n');print('VERIFIED',out['verified_utc'],'oldSHA',count,'newartifacts',len(paths),'32 faithful anchors, rejection preserved')
if __name__=='__main__':main()
