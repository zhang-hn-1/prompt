"""Trace sparse raw RGB costs before labels; require parity with frozen saved V1."""
from pathlib import Path
import json,hashlib,datetime,time
import numpy as np
from multiview_cost_trace import trace_rgb
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p=json.loads((H/'multiview_cost_trace_protocol.json').read_text());base=json.loads((H/'multiview_rgb_protocol.json').read_text())
    for path,s in p['source_and_input_sha256'].items():assert sha(path)==s,path
    depths=1/np.linspace(1/base['depth_min_m'],1/base['depth_max_m'],base['depth_bins'])
    opts=dict(depths=depths,focal=base['focal_px'],cx=base['cx'],cy=base['cy'],patch_radius=base['patch_radius'],texture_min=base['texture_std_min'],
        parallax_min=base['minimum_parallax_deg'],min_sources=base['minimum_sources'],max_cost=base['max_cost'],slack=base['competitive_slack'],max_interval=base['max_interval_ray_width_m'])
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');history=[];rows=[];out=H/'multiview_cost_traces';out.mkdir(exist_ok=True)
    start=datetime.datetime.now(datetime.timezone.utc).isoformat()
    for row in p['rows']:
        f=row['frame'];packet=np.load(H/'multiview_rgb_inputs'/f'frame_{f:03d}.npz')
        assert set(packet.files)=={'left_rgb','right_rgb','left_pose','right_pose','time_s','frame'}
        pair=[dict(rgb=packet[eye+'_rgb'],pose=packet[eye+'_pose'],mask=masks[eye],eye=eye,time_s=float(packet['time_s']),frame=f) for eye in ['left','right']]
        anchors=np.array([a['pixel_yx'] for a in row['anchors']]);yy,xx=anchors.T
        timer=time.perf_counter();trace=trace_rgb(pair[0],[pair[1]]+history,anchors,**opts);elapsed=time.perf_counter()-timer
        path=out/f'frame_{f:03d}.npz';np.savez_compressed(path,anchors_yx=anchors,query_indices=[a['query_index'] for a in row['anchors']],**trace)
        original=np.load(H/'multiview_rgb_predictions'/f'frame_{f:03d}.npz');best=trace['cost'][np.arange(8),trace['best_index']]
        oldcost=original['cost'][yy,xx];finite=np.isfinite(oldcost);costmax=float(np.max(abs(best[finite]-oldcost[finite]))) if finite.any() else 0.
        width=trace['interval_ray_width_m'];oldwidth=original['interval_ray_width_m'][yy,xx];wf=np.isfinite(oldwidth)
        widthmax=float(np.max(abs(width[wf]-oldwidth[wf]))) if wf.any() else 0.
        winnerbits=trace['view_bits'][np.arange(8),trace['best_index']]
        checks=dict(raw_depth_exact=np.array_equal(trace['raw_depth_m'],original['raw_depth_m'][yy,xx],equal_nan=True),
            winner_bits_exact=np.array_equal(winnerbits,original['view_bits'][yy,xx]),confidence_exact=np.array_equal(trace['confidence'],original['confidence'][yy,xx]),
            cost_finite_pattern_exact=np.array_equal(np.isfinite(best),finite),cost_atol_pass=costmax<=1e-5,
            interval_finite_pattern_exact=np.array_equal(np.isfinite(width),wf),interval_atol_pass=widthmax<=1e-6)
        rows.append(dict(frame=f,path=str(path),sha256=sha(path),elapsed_s=elapsed,checks=checks,cost_max_abs_diff=costmax,interval_max_abs_diff=widthmax))
        history.extend(pair);print('TRACE_FIDELITY',rows[-1],flush=True)
    result=dict(started_utc=start,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,
        fidelity_pass=all(all(r['checks'].values()) for r in rows),all_curves_saved_before_any_truth_labels=True,
        source_sha256={str(x):sha(x) for x in [Path(__file__),H/'multiview_cost_trace.py',H/'multiview_cost_trace_protocol.json']})
    (out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    assert result['fidelity_pass'],'sparse instrumentation must reproduce frozen V1 before labels'
    print('CURVES_COMPLETE',result['ended_utc'],'32 anchors, frozen V1 parity PASS',flush=True)
if __name__=='__main__':main()
