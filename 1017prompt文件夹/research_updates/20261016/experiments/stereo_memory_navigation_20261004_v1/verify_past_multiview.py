"""Preserved data/source identity, complete-query coverage and causal provenance checks."""
from pathlib import Path
import datetime,hashlib,json
import numpy as np
from scene_ray_geometry import camera_rays

H=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    count=0
    for name,key in [('past_multiview_protocol.json','input_sha256'),('static_scene_verification.json','source_and_artifact_sha256')]:
        for p,s in json.loads((H/name).read_text())[key].items():assert sha(p)==s,p;count+=1
    result=json.loads((H/'past_multiview_audit.json').read_text());initial=json.loads((H/'past_multiview_audit_initial.json').read_text())
    for p,s in result['source_sha256'].items():assert sha(p)==s,p
    assert sha(H/'audit_past_multiview_initial.py')==initial['source_sha256'][str(H/'audit_past_multiview.py')]
    assert result['pooled']==initial['pooled']
    assert result['all_history_strict_past'] and result['future_append_poison_exact'] and not result['online_admitted'] and not result['old_gates_changed']
    assert result['new_simulations']==result['new_inference_calls']==0
    assert '92 passed' in (H/'past_multiview_green.log').read_text()
    data=np.load(H/'past_multiview_measurements.npz');before=np.load(H/'past_multiview_measurements_initial.npz')
    assert data.files==before.files
    for k in data.files:np.testing.assert_equal(data[k],before[k])
    source=np.load(H/'static_scene_ray_measurements.npz');mesh=np.load(H/'static_scene_mesh_measurements.npz')
    pool={};total=0
    for i,row in enumerate(result['rows']):
        frame=row['frame'];p=f'frame_{frame:03d}__';core=source[f'left_{frame:03d}_sphere_core_selection']
        truth=np.load(H/f'capture_transaction_development/hard_0_e1_0004/offline_truth/frame_{frame:03d}.npz')
        origin,rays,yy,xx=camera_rays(truth['left_camera_usd_world'],480,640,584.8888888888889,319.5,239.5,2)
        np.testing.assert_array_equal(data[p+'source_pixels_yx'],np.column_stack([yy[core],xx[core]]))
        np.testing.assert_equal(data[p+'source_world_m'],origin+mesh[f'left_{frame:03d}_raw_mesh_axis_depth_m'][core,None]*rays[core])
        n=int(core.sum());assert n==row['metrics']['n'];total+=n
        for name in ['current_left','current_right','current_stereo','current_rgb','past_any_eye','past_stereo','past_rgb_reliable','current_right_status']:
            assert len(data[p+name])==n;pool.setdefault(name,[]).append(data[p+name])
        np.testing.assert_array_equal(data[p+'current_stereo'],data[p+'current_left']&data[p+'current_right'])
        assert not np.any(data[p+'current_rgb']&~data[p+'current_stereo'])
        assert not np.any(data[p+'past_stereo']&~data[p+'past_any_eye'])
        assert not np.any(data[p+'past_rgb_reliable']&~data[p+'past_stereo'])
        assert row['loaded_history_frames']==[r['frame'] for r in result['rows'][:i]]
        assert row['loaded_history_times_s']==[r['time_s'] for r in result['rows'][:i]]
        for flag,name in [('past_any_eye','any'),('past_stereo','stereo'),('past_rgb_reliable','rgb')]:
            present=data[p+flag];ff=data[p+'past_first_'+name+'_frame'];tt=data[p+'past_first_'+name+'_time_s']
            np.testing.assert_array_equal(ff>=0,present)
            assert np.all(tt[present]<row['time_s']) and np.isnan(tt[~present]).all()
            assert set(ff[present]).issubset(set(row['loaded_history_frames']))
        assert int(np.sum((~data[p+'current_stereo'])&data[p+'past_stereo']))==row['metrics']['stereo_past_gain_n']
    pool={k:np.concatenate(v) for k,v in pool.items()}
    current=int(pool['current_stereo'].sum());union=int((pool['current_stereo']|pool['past_stereo']).sum());gain=int((~pool['current_stereo']&pool['past_stereo']).sum())
    assert (total,current,union,gain)==(73383,8379,24298,15919)
    assert int(pool['current_rgb'].sum())==int(pool['past_rgb_reliable'].sum())==0
    assert int(np.sum(~pool['current_stereo']))==65004
    assert int(np.sum(~pool['current_stereo']&pool['past_any_eye']))==40982
    review=json.loads((H/'past_multiview_review.json').read_text());assert not review['critical'] and not review['important'] and not review['report_body_reviewed']
    names=['past_multiview.py','test_past_multiview.py','audit_past_multiview.py','audit_past_multiview_initial.py','plot_past_multiview.py','verify_past_multiview.py','past_multiview_protocol.json','past_multiview_audit.json','past_multiview_measurements.npz','past_multiview_audit.log','past_multiview_audit_initial.json','past_multiview_measurements_initial.npz','past_multiview_audit_initial.log','past_multiview_red.log','past_multiview_green.log','past_multiview_figures.log','past_multiview_review.json','figures/past_multiview_all_frames.png','figures/past_multiview_support_counts.png']
    paths=[H/name for name in names]+[H.parent.parent/'reports/benchmark_causal_past_multiview_support_20261004.md']
    out=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=count,whole_stage_cpu_tests_passed=92,
        actual_query_pairs=4,source_queries=73383,current_stereo_queries=current,current_or_past_stereo_queries=union,causal_stereo_gain_queries=gain,
        current_or_past_reliable_rgb_queries=0,all_original_queries_retained=True,initial_all_arrays_and_metrics_preserved=True,
        first_sources_strictly_past=True,reviewer_independent_actual_fields_reproduced=True,reviewer_stronger_future_payload_poison_passed=True,
        independent_review_no_critical_or_important=True,minor_legend_resolved_in_report_caption=True,final_report_body_reviewed_by_independent_agent=False,
        new_simulations=0,new_model_inferences=0,rgb_memory_admitted=False,whole_research_complete=False,figures_inspected=True,
        source_and_artifact_sha256={str(p):sha(p) for p in paths})
    (H/'past_multiview_verification.json').write_text(json.dumps(out,indent=2)+'\n')
    print('VERIFIED',out['verified_utc'],'old SHA checks',count,'new artifacts',len(paths),'complete73383queries, strict causal sources, no online admission')
if __name__=='__main__':main()
