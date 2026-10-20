"""ONE saved-output causal decomposition, no new network/fit/threshold."""
from pathlib import Path
import json,datetime
import numpy as np
from stereo_geometry import FOCAL,BASELINE,restore_native
H=Path(__file__).resolve().parent;folder=H/'dense_adapter_v2_predictions';manifest=json.loads((folder/'manifest.json').read_text());assert len(manifest['rows'])==130
p=json.loads((H/'dense_adapter_fit_v2_protocol.json').read_text());m=np.load(p['mask']);y,x=np.indices((480,640));rays=np.stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(x.shape)],axis=-1);summary={};rows=[]
for frame in range(0,321,5):
 t=np.load(H/'dense_adapter_heldout_run/frames'/f'frame_{frame:03d}.npz');z=t['left_depth_m'];pose=t['left_camera_usd_world']
 with np.errstate(invalid='ignore'):world=pose[:3,3]+(rays*z[...,None])@(pose[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>=.5)&(z<=3)&~m['left']&(np.linalg.norm(world-t['root_position_world_m'],axis=-1)>=.55)
 vr=y-FOCAL*BASELINE/np.maximum(z,1e-9);iy=np.clip(np.rint(np.nan_to_num(vr,nan=-1,posinf=-1,neginf=-1)).astype(int),0,479);rz=t['right_depth_m'][iy,x]
 with np.errstate(invalid='ignore'):overlap=(vr>=0)&(vr<=479)&~m['right'][iy,x]&np.isfinite(rz)&(np.abs(rz-z)<=.03+.01*z)
 groups=dict(body_overlap=valid&overlap&(world[...,2]>=.05)&(world[...,2]<=1.5),floor_overlap=valid&overlap&(np.abs(world[...,2])<=.05))
 for arm in ('b0','adapter'):
  pred=np.load(folder/arm/f'frame_{frame:03d}.npz');d=restore_native(pred['disparity_px']);rawz=np.full(z.shape,np.nan);ok=np.isfinite(d)&(d>0);rawz[ok]=FOCAL*BASELINE/d[ok]
  with np.errstate(invalid='ignore'):rawerror=np.abs(rawz-z)*np.linalg.norm(rays,axis=-1);keeperror=np.abs(pred['depth_m']-z)*np.linalg.norm(rays,axis=-1)
  for group,sel in groups.items():
   sel=sel&(y%2==0)&(x%2==0);rawgood=np.isfinite(rawerror)&(rawerror<=.1);kept=pred['confidence'];keepgood=np.isfinite(keeperror)&(keeperror<=.1)
   r=dict(arm=arm,frame=frame,group=group,queries=int(sel.sum()),raw_correct=int((sel&rawgood).sum()),raw_finite=int((sel&np.isfinite(rawerror)).sum()),kept=int((sel&kept).sum()),kept_correct=int((sel&keepgood).sum()),correct_dropped=int((sel&rawgood&~kept).sum()),wrong_kept=int((sel&kept&~keepgood).sum()),label_above_costspan=int((sel&(FOCAL*BASELINE/np.maximum(z,1e-9)>=416)).sum()))
   rows.append(r);s=summary.setdefault(arm,{}).setdefault(group,{k:0 for k in r if k not in ('arm','frame','group')})
   for k in s:s[k]+=r[k]
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='postprediction labels diagnostic; no model input, masks/bilateral/gatesunchanged; raw_positiveD_withoutbilateral NOT valid observed depth',summary=summary,rows=rows)
(H/'dense_adapter_saved_output_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(summary,indent=2))
