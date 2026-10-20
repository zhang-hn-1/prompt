"""Immutable capture -> official half-pixel MVS interface. No model/GT reads."""
import numpy as np
OPTICAL4=np.diag([1.,-1.,-1.,1.])
def build_request(current,sources,now):
 now=float(now);ct=float(current['time_s']);times=[float(s['time_s']) for s in sources]
 if not np.isfinite(now) or not np.isfinite(ct) or abs(now-ct)>1e-9 or len(times)!=2 or len(set(times))!=2 or not all(np.isfinite(t) and t<ct and 0<ct-t<=2.+1e-8 for t in times):
  raise ValueError('exact current clock and two distinct causal sources within 2s required before payload')
 poses=[];images=[];masks=[]
 for p in [current,*sources]:
  T=np.asarray(p['pose'],float);rgb=np.asarray(p['rgb']);m=np.asarray(p['mask'],bool)
  if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1):raise ValueError('rigid USD world-from-camera pose required')
  if rgb.shape!=(480,640,3) or rgb.dtype!=np.uint8 or m.shape!=(480,640):raise ValueError('frozen native RGB and CAD mask calibration required')
  poses.append(T@OPTICAL4);images.append(rgb.copy());masks.append(m.copy())
 K=np.eye(4);K[0,0]=K[1,1]=584.8888888888889;K[0,2]=320.;K[1,2]=240.
 return dict(cur_image=images[0],src_image=images[1:],cur_pose=poses[0],src_pose=np.stack(poses[1:]),cur_intrinsics=K.copy(),src_intrinsics=np.stack([K.copy(),K.copy()]),current_mask=masks[0],source_masks=np.stack(masks[1:]),current_time_s=ct,source_time_s=np.array(times),knownpose_privileged=True,source_model_admitted=False,learned_geometry_hypothesis_only=True)
