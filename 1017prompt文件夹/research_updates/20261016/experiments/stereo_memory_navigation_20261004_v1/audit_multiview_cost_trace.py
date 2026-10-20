"""Attach independent labels only after all raw traces are sealed and faithful."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from multiview_rgb import optical_rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def summarize(rows):
    finite=[r for r in rows if r['raw_depth_m'] is not None];retained=[r for r in rows if r['confidence']]
    wrong=[r for r in retained if r['raw_ray_error_m']>.1]
    return dict(n=len(rows),raw_finite_n=len(finite),retained_n=len(retained),retained_wrong_n=len(wrong),
        retained_wrong_adjacent_source_switch_n=sum(r['winner_adjacent_source_switch'] for r in wrong),
        retained_wrong_no_adjacent_source_switch_n=sum(not r['winner_adjacent_source_switch'] for r in wrong),
        retained_wrong_gt_bin_valid_n=sum(r['gt_nearest_cost'] is not None for r in wrong),
        retained_wrong_same_bitmap_gt_n=sum(r['winner_bits']==r['gt_nearest_bits'] for r in wrong),
        retained_wrong_gt_gap_above_slack_n=sum(r['gt_nearest_gap'] is not None and r['gt_nearest_gap']>.05 for r in wrong),
        retained_wrong_all_sources_true_visible_n=sum(r['all_winner_sources_true_visible'] for r in wrong),
        retained_wrong_adjacent_invalid_n=sum(r['winner_adjacent_invalid'] for r in wrong),
        retained_wrong_finite_domain_width_le_gate_n=sum(r['finite_domain_ray_width_m'] is not None and r['finite_domain_ray_width_m']<=.1 for r in wrong),
        retained_wrong_competitive_covers_finite_domain_n=sum(r['competitive_covers_finite_domain'] for r in wrong))

def main():
    p=json.loads((H/'multiview_cost_trace_protocol.json').read_text());manifest=json.loads((H/'multiview_cost_traces/manifest.json').read_text())
    start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    assert manifest['fidelity_pass'] and manifest['all_curves_saved_before_any_truth_labels'] and manifest['ended_utc']<start
    for path,s in p['source_and_input_sha256'].items():assert sha(path)==s,path
    old=np.load(H/'past_multiview_measurements.npz');measurements=np.load(H/'multiview_rgb_measurements.npz');rows=[]
    for row in p['rows']:
        f=row['frame'];pre=f'frame_{f:03d}__';tracepath=H/'multiview_cost_traces'/f'frame_{f:03d}.npz';d=np.load(tracepath)
        assert sha(tracepath)==next(r['sha256'] for r in manifest['rows'] if r['frame']==f)
        depth=d['depths_m'];lo=depth.copy();hi=depth.copy();mid=(depth[:-1]+depth[1:])/2;lo[1:]=mid;hi[:-1]=mid
        pose=np.load(H/'multiview_rgb_inputs'/f'frame_{f:03d}.npz')['left_pose']
        query=np.array([a['query_index'] for a in row['anchors']]);world=old[pre+'source_world_m'][query]
        z=((world-pose[:3,3])@np.linalg.inv(optical_rotation(pose)).T)[:,2]
        for i,a in enumerate(row['anchors']):
            y,x=a['pixel_yx'];norm=float(np.sqrt(1+((x-319.5)/584.8888888888889)**2+((y-239.5)/584.8888888888889)**2))
            curve=d['cost'][i];finite=np.isfinite(curve);best=int(d['best_index'][i]);bi=int(np.argmin(abs(depth-z[i])))
            raw=float(d['raw_depth_m'][i]) if np.isfinite(d['raw_depth_m'][i]) else None
            bits=int(d['view_bits'][i,best]) if raw is not None else 0
            neighbours=[j for j in [best-1,best+1] if 0<=j<len(depth)]
            source_visible=int(measurements[pre+'visible_source_bits'][a['query_index']])
            bestcost=float(curve[best]) if finite.any() else None
            gtcost=float(curve[bi]) if np.isfinite(curve[bi]) else None
            width=float((max(hi[finite])-min(lo[finite]))*norm) if finite.any() else None
            entry=dict(frame=f,**a,true_axis_depth_m=float(z[i]),nearest_gt_bin=bi,gt_nearest_depth_m=float(depth[bi]),
                gt_grid_quantization_ray_error_m=float(abs(depth[bi]-z[i])*norm),gt_grid_half_cell_ray_width_m=float((hi[bi]-lo[bi])*norm),
                raw_depth_m=raw,raw_ray_error_m=float(abs(raw-z[i])*norm) if raw is not None else None,
                confidence=bool(d['confidence'][i]),reference_std=float(d['reference_std'][i]),reference_valid=bool(d['reference_valid'][i]),
                best_index=best,best_cost=bestcost,winner_bits=bits,gt_nearest_bits=int(d['view_bits'][i,bi]),gt_nearest_cost=gtcost,
                gt_nearest_gap=gtcost-bestcost if gtcost is not None and bestcost is not None else None,
                winner_adjacent_source_switch=bool(raw is not None and any(d['view_bits'][i,j]!=bits for j in neighbours)),
                winner_adjacent_invalid=bool(raw is not None and any(not np.isfinite(curve[j]) for j in neighbours)),
                finite_bins_n=int(finite.sum()),competitive_bins_n=int(d['competitive'][i].sum()),
                competitive_covers_finite_domain=bool(finite.any() and np.array_equal(d['competitive'][i],finite)),
                finite_domain_ray_width_m=width,competitive_interval_ray_width_m=float(d['interval_ray_width_m'][i]) if np.isfinite(d['interval_ray_width_m'][i]) else None,
                actual_visible_source_bits=source_visible,all_winner_sources_true_visible=bool(bits and (bits&source_visible)==bits),
                sources=row['sources'])
            rows.append(entry);print('ANCHOR',f,a['query_index'],'confidence',entry['confidence'],'error',entry['raw_ray_error_m'],'adjacentSwitch',entry['winner_adjacent_source_switch'],'GTgap',entry['gt_nearest_gap'],flush=True)
    assert len(rows)==32 and all(sum(r['frame']==f for r in rows)==8 for f in [0,10,20,23])
    out=dict(started_utc=start,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,pooled=summarize(rows),
        by_stratum={s:summarize([r for r in rows if r['stratum']==s]) for s in ['spatial','retained_output']},
        by_frame={str(f):summarize([r for r in rows if r['frame']==f]) for f in [0,10,20,23]},
        old_candidate_rejection_preserved=True,new_candidate_admitted=False,online_navigation_runs=0,new_dense_predictions=0,
        source_sha256={str(x):sha(x) for x in [Path(__file__),H/'multiview_cost_trace.py',H/'multiview_cost_trace_protocol.json',H/'multiview_cost_traces/manifest.json']},
        limitations='32 fixed stratified anchors, not random/population-weighted/fresh; nearest GT bin is a label, curved5x5 visibility and exact true depth cost not certified; adjacency is association, not repaired counterfactual')
    (H/'multiview_cost_trace_audit.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n');print('TRACE_AUDIT_COMPLETE',out['pooled'],flush=True)
if __name__=='__main__':main()
