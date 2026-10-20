"""Offline predictor-vs-confidence decomposition, no changed inference or gates."""
import json,hashlib
from pathlib import Path
import numpy as np
from stereo_geometry import FOCAL,BASELINE,restore_native
from audit_stereo import summarize
H=Path(__file__).resolve().parent
masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
y,x=np.indices((480,640));rays=np.stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(x.shape)),axis=-1)
rows=[];pooled={}
for row in json.loads((H/'transaction_development_manifest.json').read_text())['samples']:
    t=np.load(row['truth_file']);rgb=np.load(row['rgb_file']);p=np.load(H/'predictions_transaction_development/fast'/row['sequence']/Path(row['rgb_file']).name)
    z=t['left_depth_m'];pose=t['left_camera_usd_world'];rotation=pose[:3,:3]@np.diag([1,-1,-1])
    with np.errstate(invalid='ignore',divide='ignore'):
        world=pose[:3,3]+(rays*z[...,None])@rotation.T
        vr=y-FOCAL*BASELINE/z
    iv=np.rint(np.clip(np.nan_to_num(vr,nan=-1,posinf=-1,neginf=-1),0,479)).astype(int)
    rz=t['right_depth_m'][iv,x]
    selection=np.isfinite(z)&(z>=.5)&(z<=3)&~masks['left']&(vr>=0)&(vr<=479)&~masks['right'][iv,x]&np.isfinite(rz)&(np.abs(rz-z)<=.03+.01*z)&(np.linalg.norm(world-t['root_position_world_m'],axis=-1)>=.55)&(world[...,2]>=.05)&(world[...,2]<=1.5)
    selection[1::2]=False;selection[:,1::2]=False
    dl=restore_native(p['disparity_px']);positive=np.isfinite(dl)&(dl>0)
    before=np.full(z.shape,np.nan);before[positive]=FOCAL*BASELINE/dl[positive]
    with np.errstate(invalid='ignore'):
        rawerr=np.abs(before-z)*np.linalg.norm(rays,axis=-1)
        conferr=np.abs(p['depth_m']-z)*np.linalg.norm(rays,axis=-1)
    a=np.where(np.isfinite(rawerr),rawerr,np.inf)[selection];b=np.where(np.isfinite(conferr),conferr,np.inf)[selection]
    residual=np.mean(abs(rgb['left_rgb'].astype(float)-rgb['right_rgb'][iv,x].astype(float)),axis=-1)[selection]
    pooled.setdefault(row['sequence'],{'raw':[],'final':[],'rgb':[]})
    for key,val in [('raw',a),('final',b),('rgb',residual)]:pooled[row['sequence']][key].append(val)
    rows.append(dict(sequence=row['sequence'],frame=Path(row['rgb_file']).name,n=int(selection.sum()),raw_left_disparity_only=summarize(a),final_bilateral_confident=summarize(b),rgb_true_partner_mean_absolute_difference_0_255=float(np.mean(residual)) if residual.size else None))
groups={seq:{'raw_left_disparity_only':summarize(np.concatenate(z['raw'])),'final_bilateral_confident':summarize(np.concatenate(z['final'])),'rgb_true_partner_difference_median_0_255':float(np.median(np.concatenate(z['rgb'])))} for seq,z in pooled.items()}
result=dict(role='development predictor/confidence decomposition only; raw-unfiltered is NOT an admitted alternative',groups=groups,rows=rows,limitations='Truth partner residual is descriptive and can include view-dependent rendering/pixel sampling; no fitted offset, threshold, confidence or checkpoint changes. Future truth never fed to predictor.',source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'transaction_development_manifest.json',H/'stereo_geometry.py',H/'fast_stereo.py',H/'audit_stereo.py']})
(H/'transaction_prediction_layers.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
print(json.dumps(groups,indent=2))
