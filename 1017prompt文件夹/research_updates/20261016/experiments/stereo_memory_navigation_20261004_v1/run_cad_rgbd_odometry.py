"""Historical causal prediction; truth labels are absent from this runner."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from feature_tracks import extract
from stereo_geometry import restore_native
from stereo_vo import PoseChain
from cad_rgbd_odometry import estimate_rgbd_increment
H=Path(__file__).resolve().parent;O=H/'nominal_cap_history_run';D=H/'cad_rgbd_odometry';D.mkdir(exist_ok=True);I=D/'inputs';I.mkdir(exist_ok=True);sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();source={};files=sorted((O/'source_predictions').glob('*.npz'));assert len(files)==25
for f in files:
 s=np.load(f);i=int(s['frame']);rgbf=O/'online_inputs'/f'frame_{i:03d}.npz';mf=O/'masks'/f'frame_{i:03d}.npz';r=np.load(rgbf);m=np.load(mf);assert float(r['time_s'])==float(s['time_s']);xy,desc,size=extract(r['left_rgb'],m['left']);z=s['depth_m'];disp=restore_native(s['disparity_px']);y,x=np.indices(z.shape);yp=y-np.where(np.isfinite(disp),disp,0.);yi=np.rint(np.clip(yp,0,479)).astype(int)
 valid=s['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(disp)&(disp>1)&~m['left']&(yp>=0)&(yp<=479)&~m['right'][yi,x]
 # Fixed rig only: optical to base, source precision and self-distance gates.
 rays=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(z.shape)],axis=-1)
 rotation=np.array([[0,0,1],[0,-1,0],[1,0,0]])
 with np.errstate(invalid='ignore',divide='ignore'):
  body=(rays*z[:,:,None])@rotation.T+np.array([.2,.3,.05]);sigma=np.maximum(abs(584.8888888888889*.6/(disp-1)-z),abs(584.8888888888889*.6/(disp+1)-z))*np.linalg.norm(rays,axis=-1)
 valid&=(np.linalg.norm(body,axis=-1)>=.55)&(sigma<=.1)
 np.savez_compressed(I/f'frame_{i:03d}.npz',time_s=float(s['time_s']),xy=xy,desc=desc,depth_m=z,valid=valid)
 for p in (f,rgbf,mf):source[str(p)]=sha(p)
for p in [H/'cad_rgbd_odometry.py',H/'stereo_vo.py',H/'feature_tracks.py',H/'run_cad_rgbd_odometry.py',H/'test_cad_rgbd_odometry.py',H.parent.parent/'docs/superpowers/specs/2026-10-07-cad-masked-rgbd-odometry-design.md']+list(I.glob('*.npz')):source[str(p)]=sha(p)
protocol=dict(utc=now(),source_sha256=source,sequence='nominal_cap_history_run',historical_not_fresh=True,maximum_sequences=1,truth_labels_not_inputs=True,pose_gates=dict(coverage=.9,rms_m=.05,peak_m=.1,rotation_p90_deg=2,pair_translation_m=.05,pair_rotation_deg=2))
(D/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');chain=PoseChain();records=[];previous=None
for f in sorted(I.glob('*.npz')):
 d=np.load(f);assert set(d.files)=={'time_s','xy','desc','depth_m','valid'};frame=int(f.stem.split('_')[1]);p={k:d[k] for k in d.files};p['time_s']=float(p['time_s'])
 if previous is None:pose=chain.pose.copy();result=dict(accepted=True,reason='initial_gauge_identity',transform=None,landmarks=0,inliers=[])
 else:result=estimate_rgbd_increment(previous,p,np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]]));pose=chain.update(result['transform'])
 records.append(dict(frame=frame,time_s=p['time_s'],feature_count=len(p['xy']),qualified_depth_pixels=int(p['valid'].sum()),**result,accumulated_pose=None if pose is None else pose.tolist()));previous=p
 print('RGBD_POSE',frame,result['reason'],result['landmarks'],flush=True)
(D/'predictions.json').write_text(json.dumps(records,indent=2)+'\n');(D/'prediction_completion.json').write_text(json.dumps(dict(utc=now(),prediction_sha256=sha(D/'predictions.json'),records=len(records),truth_labels_not_read=True),indent=2)+'\n')
