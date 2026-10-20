"""Optional frozen V1 on RGB-only anchors; save all predictions before plane labels."""
from pathlib import Path
import json,datetime,hashlib,time
import numpy as np
from board_material import select_rgb_anchors,validate_budget
from multiview_cost_trace import trace_rgb
from multiview_rgb import optical_rotation
H=Path(__file__).resolve().parent;o=H/'matte_coded_board_run'
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
p=json.loads((H/'matte_coded_board_protocol.json').read_text());a=json.loads((o/'geometry_assessment.json').read_text())
assert a['geometry_gate_pass'],'geometry rejection blocks downstream'
for f,s in p['source_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
v=json.loads((H/'multiview_rgb_protocol.json').read_text());m=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
maximum=p['matching_budget']['max_anchors_per_frame'];validate_budget(6,maximum,100)
kwargs=dict(depths=np.sort(1/np.linspace(1/v['depth_max_m'],1/v['depth_min_m'],v['depth_bins'])),focal=v['focal_px'],cx=v['cx'],cy=v['cy'],patch_radius=v['patch_radius'],texture_min=v['texture_std_min'],parallax_min=v['minimum_parallax_deg'],min_sources=v['minimum_sources'],max_cost=v['max_cost'],slack=v['competitive_slack'],max_interval=v['max_interval_ray_width_m'])
# Access only named RGB/pose/time arrays; depth remains unloaded until offline labels (not required).
packets={}
for i in p['selected_indices']:
    f=np.load(o/'frames'/f'frame_{i:03d}.npz')
    for eye in ['left','right']:
        packets[i,eye]=dict(frame=i,time_s=float(f['time_s']),eye=eye,rgb=f[eye+'_rgb'],pose=f[eye+'_camera_usd_world'],mask=m[eye])
pred=o/'v1_sparse';pred.mkdir(exist_ok=False);rows=[]
for i in p['selected_indices']:
    d=np.load(o/'rgb_detections'/f'frame_{i:03d}_left.npz');q,ids=select_rgb_anchors(d['ids'],d['corners'],m['left'],maximum)
    past=[k for k in p['selected_indices'] if k<i][-3:]
    sources=[packets[i,'right']]+[packets[k,e] for k in past for e in ['left','right']]
    started=time.perf_counter();t=trace_rgb(packets[i,'left'],sources,q,**kwargs) if len(q) else {}
    filename=pred/f'frame_{i:03d}.npz';np.savez_compressed(filename,anchors_yx=q,anchor_id_corner=np.array(ids,int).reshape(-1,2),**t)
    rows.append(dict(frame=i,anchors=len(q),seconds=time.perf_counter()-started,source_ids=[[r['frame'],r['eye'],r['time_s']] for r in sources],path=str(filename),sha256=hashlib.sha256(filename.read_bytes()).hexdigest()))
manifest=dict(saved_utc=now(),rows=rows,geometry_not_loaded=True,sanitized_fields=list(next(iter(packets.values()))),role='knownpose RGB V1 only; no future/depth/board geometry predictor input')
(pred/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
# Only now load board-world geometry. Whole selected anchors remain in denominator, reject excluded truth as unsupported.
g=json.loads((o/'board_geometry.json').read_text());yb=np.array(g['centres'])[:,1];results=[];all_errors=[];n=kept=good=0
for row in rows:
    i=row['frame'];r=np.load(row['path']);q=r['anchors_yx'];pose=packets[i,'left']['pose'];rays=np.c_[(q[:,1]-v['cx'])/v['focal_px'],(q[:,0]-v['cy'])/v['focal_px'],np.ones(len(q))];world=rays@optical_rotation(pose).T
    z=(g['plane_x']-pose[0,3])/world[:,0];point=pose[:3,3]+z[:,None]*world
    eligible=np.isfinite(z)&(z>0)&(point[:,1]>=yb.min()-.27)&(point[:,1]<=yb.max()+.27)&(point[:,2]>=.02)&(point[:,2]<=2.26)
    confidence=r['confidence'] if len(q) else np.zeros(0,bool);error=np.abs(r['raw_depth_m']-z)*np.linalg.norm(rays,axis=1) if len(q) else np.zeros(0)
    valid=eligible&confidence;correct=valid&(error<=.1);all_errors.extend(error[valid].tolist())
    results.append(dict(frame=i,queries=len(q),eligible_label_queries=int(eligible.sum()),retained=int(valid.sum()),correct_0p10m=int(correct.sum()),source_ids=row['source_ids']))
    np.savez_compressed(pred/f'labels_{i:03d}.npz',expected_depth_m=z,eligible_label=eligible,error_m=error,valid=valid,correct=correct)
    n+=len(q);kept+=int(valid.sum());good+=int(correct.sum())
p90=float(np.percentile(all_errors,90)) if all_errors else None
assessment=dict(predictions_saved_utc=manifest['saved_utc'],labels_evaluated_utc=now(),queries=n,retained=kept,correct_0p10m=good,unconditional_fraction=good/n if n else 0,conditional_p90_m=p90,gate_pass=bool(n>=100 and good>=.9*n and p90 is not None and p90<=.1),rows=results,role='sparse known-pose calibration-board witness only; not original obstacle/estimatedpose/online/navigation admission')
(pred/'assessment.json').write_text(json.dumps(assessment,indent=2)+'\n');print(json.dumps(assessment,indent=2))
