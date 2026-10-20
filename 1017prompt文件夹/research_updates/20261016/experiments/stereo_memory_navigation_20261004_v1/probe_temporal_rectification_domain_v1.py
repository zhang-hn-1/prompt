"""Throwaway source-only rectification feasibility probe, not a deployable adapter."""
from pathlib import Path
import numpy as np,json,hashlib,datetime,cv2
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'temporal_rectification_domain_probe_v1';assert not D.exists();D.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),current_frames=[235,245,255],lag_s=2.,eyes=['left','right'],cv_stereoRectify_alpha=1.,zero_disparity=True,size=[640,480],knownpose_privileged=True,development_not_fresh=True,no_matcher_or_depth_output=True,throwaway_probe_only=True,code_sha256=sha(__file__));(D/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');rows=[];inputs={}
def load(p):inputs[str(p.resolve())]=sha(p);return np.load(p)
for i in [235,245,255]:
 current=load(O/'known_poses'/f'frame_{i:03d}.npz');old=load(O/'known_poses'/f'frame_{i-20:03d}.npz');cm=load(O/'masks'/f'frame_{i:03d}.npz');om=load(O/'masks'/f'frame_{i-20:03d}.npz')
 for eye in ['left','right']:
  A=current[eye+'_camera_usd_world'];B=old[eye+'_camera_usd_world'];R0=A[:3,:3]@np.diag([1,-1,-1]);R1=B[:3,:3]@np.diag([1,-1,-1]);R=R1.T@R0;t=R1.T@(A[:3,3]-B[:3,3]);a,b,p,q,Q,roi0,roi1=cv2.stereoRectify(K,None,K,None,(640,480),R,t,flags=cv2.CALIB_ZERO_DISPARITY,alpha=1,newImageSize=(640,480));maps=[];valid=[]
  for rot,P,mask in [(a,p,cm[eye]),(b,q,om[eye])]:
   mx,my=cv2.initUndistortRectifyMap(K,None,rot,P[:3,:3],(640,480),cv2.CV_32FC1);finite=np.isfinite(mx)&np.isfinite(my);inside=finite&(mx>=3)&(mx<=636)&(my>=3)&(my<=476);m=cv2.remap(mask.astype(np.uint8),mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=1).astype(bool);valid.append(inside&~m);maps.append((mx,my))
  common=valid[0]&valid[1];yy,xx=np.nonzero(common);row=dict(current_frame=i,source_frame=i-20,eye=eye,baseline_m=float(np.linalg.norm(t)),relative_optical_translation=t.tolist(),new_focal_px=float(p[0,0]),new_principal=p[:2,2].tolist(),rectified_axis='vertical' if abs(q[1,3])>abs(q[0,3]) else 'horizontal',P2_translation=q[:2,3].tolist(),current_valid_pixels=int(valid[0].sum()),past_valid_pixels=int(valid[1].sum()),same_rectified_coordinate_valid_overlap=int(common.sum()),overlap_bbox=[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())] if len(xx) else None,matching_overlap_not_certified=True);rows.append(row)
  np.savez_compressed(D/f'frame_{i:03d}_{eye}.npz',R_current=a,R_past=b,P_current=p,P_past=q,Q=Q,current_map_x=maps[0][0],current_map_y=maps[0][1],past_map_x=maps[1][0],past_map_y=maps[1][1],current_valid=valid[0],past_valid=valid[1])
f=D/'assessment_v1.json';f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,input_sha256=inputs,GT_and_RGB_and_actions_not_read=True,knownpose_privileged=True,no_prediction_or_navigation_gate=True),indent=2)+'\n');print(rows)
