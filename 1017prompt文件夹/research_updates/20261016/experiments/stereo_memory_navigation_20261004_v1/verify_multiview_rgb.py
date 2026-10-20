"""Final immutable inputs, all denominators, truthful stage claims and artifact identity."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent;W=H.parent.parent

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    oldchecks=0
    for name,key in [('multiview_rgb_protocol.json','input_sha256'),('past_multiview_verification.json','source_and_artifact_sha256')]:
        for path,s in json.loads((H/name).read_text())[key].items():assert sha(path)==s,path;oldchecks+=1
    result=json.loads((H/'multiview_rgb_audit.json').read_text());run=json.loads((H/'multiview_rgb_predictions/manifest.json').read_text())
    for m in [run,result]:
        for path,s in m['source_sha256'].items():assert sha(path)==s,path
    assert not result['candidate_accepted'] and not result['online_navigation_admitted'] and not result['old_gates_changed']
    assert result['new_simulations']==result['new_control_runs']==result['fresh_sequences']==0
    assert '6 failed' in (H/'multiview_rgb_red.log').read_text()
    assert '6 passed' in (H/'multiview_rgb_green.log').read_text()
    assert '98 passed' in (H/'multiview_rgb_full_suite.log').read_text()
    assert 'PREDICTIONS_COMPLETE' in (H/'multiview_rgb_run.log').read_text()
    data=np.load(H/'multiview_rgb_measurements.npz');old=np.load(H/'past_multiview_measurements.npz');total=0
    sums={k:0 for k in ['confident','correct','strict_any_visible_contributor','strict_all_visible_contributors','fast_confident','fast_correct']}
    errors=[]
    for i,(r,predrow) in enumerate(zip(result['rows'],run['rows'])):
        f=r['frame'];prefix=f'frame_{f:03d}__';n=r['metrics']['n'];total+=n
        np.testing.assert_equal(data[prefix+'source_pixels_yx'],old[prefix+'source_pixels_yx'])
        assert n==len(old[prefix+'source_world_m']) and f==predrow['frame']
        assert predrow['ended_utc']<result['started_utc'] and sha(predrow['prediction'])==predrow['sha256']
        assert len(r['sources'])==1+2*i
        assert r['sources'][0]==dict(frame=f,time_s=r['time_s'],eye='right',bit=1)
        for j,s in enumerate(r['sources']):
            assert s['bit']==1<<j
            if j>0:assert s['time_s']<r['time_s'] and s['frame'] in [x['frame'] for x in result['rows'][:i]]
        pred=np.load(predrow['prediction']);assert all(pred[k].shape==(480,640) for k in pred.files)
        assert not np.isfinite(pred['depth_m'][~pred['confidence']]).any()
        for key in sums:
            a=data[prefix+key];assert len(a)==n and int(a.sum())==r['metrics'][key+'_n'];sums[key]+=int(a.sum())
        c=data[prefix+'confident'];e=data[prefix+'error_m']
        np.testing.assert_array_equal(data[prefix+'correct'],c&(e<=.1));assert np.isinf(e[~c]).all()
        np.testing.assert_allclose(np.percentile(e[c],90),r['metrics']['candidate_ray_error_p90_m'])
        assert not r['metrics']['gate_pass'];errors.append(e[c])
        assert np.all(~data[prefix+'strict_all_visible_contributors']|data[prefix+'strict_any_visible_contributor'])
    assert total==73383
    assert sums==dict(confident=796,correct=43,strict_any_visible_contributor=1,strict_all_visible_contributors=1,fast_confident=2832,fast_correct=6)
    for k,v in sums.items():assert result['pooled'][k+'_n']==v
    np.testing.assert_allclose(np.percentile(np.concatenate(errors),90),result['pooled']['candidate_ray_error_p90_m'])
    clean=json.loads((H/'multiview_rgb_inputs/manifest.json').read_text())
    for row in clean['rows']:
        assert sha(row['path'])==row['sha256'];packet=np.load(row['path'])
        assert set(packet.files)=={'left_rgb','right_rgb','left_pose','right_pose','time_s','frame'}
        actual=np.load(row['rgb_source']);poses=np.load(row['known_pose_source'])
        for eye in ['left','right']:
            np.testing.assert_equal(packet[eye+'_rgb'],actual[eye+'_rgb']);np.testing.assert_equal(packet[eye+'_pose'],poses[eye+'_camera_usd_world'])
    review=json.loads((H/'multiview_rgb_review.json').read_text())
    assert not review['critical'] and not review['important']
    names=['summarize_multiview_rgb_provenance.py','multiview_rgb_provenance_summary.json','multiview_rgb_provenance_summary.log','multiview_rgb.py','test_multiview_rgb.py','prepare_multiview_rgb_inputs.py','run_multiview_rgb.py','audit_multiview_rgb.py','diagnose_multiview_rgb.py','plot_multiview_rgb.py','write_multiview_rgb_report.py','verify_multiview_rgb.py','multiview_rgb_protocol.json','multiview_rgb_red.log','multiview_rgb_green.log','multiview_rgb_full_suite.log','multiview_rgb_prepare.log','multiview_rgb_run.log','multiview_rgb_audit_before_prediction_completion.log','multiview_rgb_audit.log','multiview_rgb_audit.json','multiview_rgb_measurements.npz','multiview_rgb_true_depth_diagnostic.json','multiview_rgb_true_depth_diagnostic.npz','multiview_rgb_true_depth_diagnostic.log','multiview_rgb_figures.log','multiview_rgb_review.json','multiview_rgb_inputs/manifest.json','multiview_rgb_predictions/manifest.json','figures/multiview_rgb_all_frames.png','figures/multiview_rgb_comparison.png']
    paths=[H/n for n in names]+[W/'reports/benchmark_causal_multiview_rgb_plane_sweep_20261004.md']
    paths += [Path(r['path']) for r in clean['rows']]+[Path(r['prediction']) for r in run['rows']]
    output=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=oldchecks,whole_stage_cpu_tests=98,
        actual_rgb_candidate_predictions=4,queries=total,sums=sums,all_four_predictions_before_truth_evaluation=True,
        old_artifacts_preserved=True,sanitized_known_pose_packet_identity_exact=True,synthetic_full_predictor_future_payload_test=True,
        actual_dense_future_reprediction=False,independent_review_no_critical_or_important=True,report_body_reviewed=True,figures_inspected=True,
        candidate_accepted=False,online_rgb_memory_admitted=False,new_simulations=0,new_control_runs=0,whole_research_complete=False,
        source_and_artifact_sha256={str(p):sha(p) for p in paths})
    (H/'multiview_rgb_verification.json').write_text(json.dumps(output,indent=2)+'\n')
    print('VERIFIED',output['verified_utc'],'oldchecks',oldchecks,'newartifacts',len(paths),'fullquery gate REJECTED, no online admission')
if __name__=='__main__':main()
