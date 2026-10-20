"""One fixed 34-point retained-risk support diagnostic, not an actor filter."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent;c=json.loads((H/'temporal_first_risk_support_protocol_v1.json').read_text());D=H/c['run'];O=H/c['output'];O.mkdir(exist_ok=False)
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in manifest['artifact_sha256'].items())
source=np.load(D/'partial_observations/frame_005.npz');world=source['world'];q=source['query_pixel_id'];xy=np.column_stack([q%640,q//640]).astype(np.float32).reshape(-1,1,2);old=np.load(D/'source_inputs/frame_005.npz');oldgray=cv2.cvtColor(old['left_rgb'],cv2.COLOR_RGB2GRAY);assert len(xy)==34
inputs={};rows=[]
def project(points,T):
 optical_R=T[:3,:3]@np.diag([1,-1,-1]);pc=(points-T[:3,3])@optical_R
 return np.column_stack([584.8888888888889*pc[:,0]/pc[:,2]+319.5,584.8888888888889*pc[:,1]/pc[:,2]+239.5]),pc[:,2]
base=np.load(D/'known_poses/frame_005.npz');birth,z=project(world,base['left_camera_usd_world']);assert np.max(np.abs(birth-xy[:,0]))<1e-4
for i in c['current_frames']:
 f=f'frame_{i:03d}.npz';rgb=np.load(D/'source_inputs'/f);pose=np.load(D/'known_poses'/f);assert float(pose['time_s'])==float(rgb['time_s'])
 gray=cv2.cvtColor(rgb['left_rgb'],cv2.COLOR_RGB2GRAY);tracked,ok,err=cv2.calcOpticalFlowPyrLK(oldgray,gray,xy,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01));back,bok,berr=cv2.calcOpticalFlowPyrLK(gray,oldgray,tracked,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01))
 predicted,depth=project(world,pose['left_camera_usd_world']);fb=np.linalg.norm(back[:,0]-xy[:,0],axis=1);inside=(tracked[:,0,0]>=0)&(tracked[:,0,0]<640)&(tracked[:,0,1]>=0)&(tracked[:,0,1]<480);valid=ok.ravel().astype(bool)&bok.ravel().astype(bool)&inside&np.isfinite(tracked[:,0]).all(1)&(fb<=1)&(err.ravel()<=20)&(depth>0);residual=np.linalg.norm(tracked[:,0]-predicted,axis=1)
 np.savez_compressed(O/f,predicted_xy=predicted,tracked_xy=tracked[:,0],valid_LK=valid,forward_backward_px=fb,reprojection_residual_px=residual,old_query_pixel_id=q,time_s=float(rgb['time_s']))
 rows.append(dict(frame=i,time_s=float(rgb['time_s']),old_points=34,usable_tracks=int(valid.sum()),usable_residual_median_px=float(np.median(residual[valid])) if valid.any() else None,usable_residual_p90_px=float(np.percentile(residual[valid],90)) if valid.any() else None,usable_within1px=int(np.sum(valid&(residual<=1))),no_track_is_unknown_not_negative=True))
 for p in (D/'source_inputs'/f,D/'known_poses'/f,D/'source_inputs/frame_005.npz',D/'partial_observations/frame_005.npz'):inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
 print(rows[-1],flush=True)
(O/'assessment.json').write_text(json.dumps(dict(protocol=c,rows=rows,inputs_sha256=inputs,birth_projection_crosscheck=True,teacher_depth_read=False,no_actor_change=True,knownpose_offline_diagnostic_not_online=True,old_matching_rejections_retained=True),indent=2)+'\n');(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.rglob('*') if p.is_file()}),indent=2)+'\n')
