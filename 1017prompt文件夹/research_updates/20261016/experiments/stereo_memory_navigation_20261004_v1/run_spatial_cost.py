"""Four historical causal RGB predictions; offline truth is absent from this runner."""
from pathlib import Path
import datetime,hashlib,json,time
import numpy as np
from spatial_cost import predict_rgb,decode_evidence
from spatial_cost_unary import select_sources
H=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def now():return datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
    p=json.loads((H/'spatial_cost_protocol.json').read_text())
    for path,value in p['source_sha256'].items():assert sha(path)==value,path
    for path,value in p['prediction_input_sha256'].items():assert sha(path)==value,path
    m=json.loads((H/'multiview_rgb_inputs/manifest.json').read_text())
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    opts=dict(depths=1/np.linspace(1/p['depth_min_m'],1/p['depth_max_m'],p['depth_bins']),focal=p['focal_px'],cx=p['cx'],cy=p['cy'],
        patch_radius=p['patch_radius'],texture_min=p['texture_std_min'],parallax_min=p['minimum_parallax_deg'],min_sources=p['minimum_sources'],
        max_cost=p['max_cost'],slack=p['competitive_slack'],max_interval=p['max_interval_ray_width_m'])
    out=H/'spatial_cost_predictions';out.mkdir(exist_ok=False);history=[];rows=[]
    for r in m['rows']:
        assert sha(r['path'])==r['sha256'];packet=np.load(r['path'])
        assert set(packet.files)=={'left_rgb','right_rgb','left_pose','right_pose','time_s','frame'}
        pair=[dict(rgb=packet[e+'_rgb'],pose=packet[e+'_pose'],mask=masks[e],eye=e,time_s=float(packet['time_s']),frame=int(packet['frame'])) for e in ['left','right']]
        sources=[pair[1]]+history;started=now();tick=time.perf_counter()
        pred,c,bits,norm=predict_rgb(pair[0],sources,p1=p['p1'],p2=p['p2'],**opts);elapsed=time.perf_counter()-tick
        baseline=decode_evidence(c,bits,norm,opts['depths'],c,max_cost=opts['max_cost'],slack=opts['slack'],max_interval=opts['max_interval'])
        oldpath=H/'multiview_rgb_predictions'/Path(r['path']).name;old=np.load(oldpath)
        equality={k:bool(np.array_equal(old[k],baseline[k],equal_nan=True)) for k in old.files}
        assert all(equality.values()),equality
        path=out/Path(r['path']).name;np.savez_compressed(path,**pred)
        unarypath=out/(path.stem+'_unary.npz');np.savez_compressed(unarypath,cost=c,view_bits=bits,raynorm=norm)
        views=[dict(frame=v['frame'],time_s=v['time_s'],eye=v['eye'],bit=1<<i) for i,v in enumerate(select_sources(pair[0],sources))]
        finite=np.isfinite(pred['raw_depth_m']);changed=finite&(pred['winner_index']!=pred['unary_best_index'])
        row=dict(frame=r['frame'],time_s=r['time_s'],sources=views,prediction=str(path),sha256=sha(path),unary_path=str(unarypath),unary_sha256=sha(unarypath),
            started_utc=started,ended_utc=now(),elapsed_cpu_wall_s=elapsed,confident_pixels=int(pred['confidence'].sum()),finite_raw_pixels=int(finite.sum()),
            changed_finite_winners=int(changed.sum()),unchanged_baseline_array_equal=equality,old_prediction_sha256=sha(oldpath))
        rows.append(row);history.extend(pair)
        (H/'spatial_cost_status.json').write_text(json.dumps(dict(utc=now(),status='running',completed_frames=len(rows),rows=rows),indent=2)+'\n')
        print('FRAME',row['frame'],'seconds',elapsed,'confident',row['confident_pixels'],'changed',row['changed_finite_winners'],flush=True)
    assert [r['frame'] for r in rows]==p['frames']
    result=dict(role=p['role'],all_four_predictions_serialized_before_truth_evaluation=True,rows=rows,completed_utc=now(),protocol_sha256=sha(H/'spatial_cost_protocol.json'))
    (out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    (H/'spatial_cost_status.json').write_text(json.dumps(dict(utc=now(),status='predictions_complete',completed_frames=4),indent=2)+'\n')
    print('ALL_PREDICTIONS_SAVED',flush=True)
if __name__=='__main__':main()
