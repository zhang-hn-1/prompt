"""Finite layer counts, no changed gates or new pose predictions."""
from pathlib import Path
import json,numpy as np
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;D=H/'cad_rgbd_odometry';O=H/'nominal_cap_history_run';records=[]
for f in sorted((D/'inputs').glob('*.npz')):
 a=np.load(f);i=int(f.stem.split('_')[1]);d=np.load(O/'source_predictions'/f'frame_{i:03d}.npz');m=np.load(O/'masks'/f'frame_{i:03d}.npz');xy=np.rint(a['xy']).astype(int);x,y=xy.T;z=d['depth_m'][y,x];disp=restore_native(d['disparity_px'])[y,x];conf=d['confidence'][y,x].astype(bool);rng=conf&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(disp)&(disp>1);yp=y-np.where(np.isfinite(disp),disp,0);iy=np.rint(np.clip(yp,0,479)).astype(int);bilateral=rng&(yp>=0)&(yp<=479)&~m['right'][iy,x];rays=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
 with np.errstate(divide='ignore',invalid='ignore'):sigma=np.maximum(abs(584.8888888888889*.6/(disp-1)-z),abs(584.8888888888889*.6/(disp+1)-z))*np.linalg.norm(rays,axis=-1)
 precise=bilateral&(sigma<=.1);valid=a['valid'][y,x];records.append(dict(frame=i,time_s=float(a['time_s']),full_support_ORB=len(x),confidence_at_feature=int(conf.sum()),range_disparity_after_confidence=int(rng.sum()),bilateral_CAD_after_range=int(bilateral.sum()),precision_after_bilateral=int(precise.sum()),final_selfdistance_after_precision=int(valid.sum())))
(D/'support_layer_diagnosis.json').write_text(json.dumps(dict(unchanged_predictions=True,no_regating=True,frames=records),indent=2)+'\n');print(json.dumps(records,indent=2))
