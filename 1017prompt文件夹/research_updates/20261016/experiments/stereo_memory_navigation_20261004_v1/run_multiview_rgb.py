"""Predict all four packets before any independent truth evaluation is run."""
from pathlib import Path
import json,hashlib,time,datetime
import numpy as np
from multiview_rgb import predict_rgb,select_sources
H=Path(__file__).resolve().parent

def main():
    p=json.loads((H/'multiview_rgb_protocol.json').read_text())
    manifest=json.loads((H/'multiview_rgb_inputs/manifest.json').read_text())
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    depths=1/np.linspace(1/p['depth_min_m'],1/p['depth_max_m'],p['depth_bins'])
    opts=dict(depths=depths,focal=p['focal_px'],cx=p['cx'],cy=p['cy'],patch_radius=p['patch_radius'],
        texture_min=p['texture_std_min'],parallax_min=p['minimum_parallax_deg'],min_sources=p['minimum_sources'],
        max_cost=p['max_cost'],slack=p['competitive_slack'],max_interval=p['max_interval_ray_width_m'])
    out=H/'multiview_rgb_predictions';out.mkdir(exist_ok=True);history=[];rows=[]
    for r in manifest['rows']:
        packet=np.load(r['path']);assert hashlib.sha256(Path(r['path']).read_bytes()).hexdigest()==r['sha256']
        assert set(packet.files)=={'left_rgb','right_rgb','left_pose','right_pose','time_s','frame'}
        pair=[dict(rgb=packet[eye+'_rgb'],pose=packet[eye+'_pose'],mask=masks[eye],eye=eye,
            time_s=float(packet['time_s']),frame=int(packet['frame'])) for eye in ['left','right']]
        sources=[pair[1]]+history
        started=datetime.datetime.now(datetime.timezone.utc).isoformat();timer=time.perf_counter()
        pred=predict_rgb(pair[0],sources,**opts);elapsed=time.perf_counter()-timer
        path=out/Path(r['path']).name;np.savez_compressed(path,**pred)
        views=[dict(frame=v['frame'],time_s=v['time_s'],eye=v['eye'],bit=1<<i) for i,v in enumerate(select_sources(pair[0],sources))]
        row=dict(frame=r['frame'],time_s=r['time_s'],sources=views,prediction=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),elapsed_cpu_wall_s=elapsed,
            confident_pixels=int(pred['confidence'].sum()),finite_raw_pixels=int(np.isfinite(pred['raw_depth_m']).sum()))
        rows.append(row);history.extend(pair);print('PREDICTION',row,flush=True)
    assert [r['frame'] for r in rows]==p['frames']
    result=dict(role=p['role'],all_four_predictions_serialized_before_truth_evaluation=True,rows=rows,
        source_sha256={str(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in [Path(__file__),H/'multiview_rgb.py',H/'multiview_rgb_protocol.json',H/'multiview_rgb_inputs/manifest.json']})
    (out/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    print('PREDICTIONS_COMPLETE',flush=True)
if __name__=='__main__':main()
