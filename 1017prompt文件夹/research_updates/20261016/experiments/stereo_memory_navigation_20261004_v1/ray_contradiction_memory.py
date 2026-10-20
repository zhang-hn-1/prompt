"""Past stereo ray contradictions remove MODEL only; no observed-free output."""
import numpy as np
from scipy.spatial import cKDTree

def contradict_cores(cores,source_pose_pairs,now):
 if not np.isfinite(now):raise ValueError('current finite clock before payload')
 cores=np.asarray(cores,float)
 if cores.ndim!=2 or cores.shape[1]!=3 or not np.isfinite(cores).all():raise ValueError('finite cores')
 proof=[[] for _ in cores];last=-np.inf
 for q,pose in source_pose_pairs:
  t=float(q['time_s']);pt=float(pose['time_s'])
  if not np.isfinite(t) or not np.isfinite(pt) or t>now+1e-9 or abs(t-pt)>1e-9 or t<=last:raise ValueError('causal matched monotone source clock before payload')
  frame=float(q['frame']);width=float(q['frame_width'])
  if not np.isfinite(frame) or frame!=round(frame) or abs(frame*.1-t)>1e-5 or width!=640.:raise ValueError('exact native ray identity')
  T=np.asarray(pose['left_camera_usd_world'],float)
  if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-9) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1):raise ValueError('rigid current USD camera')
  w=np.asarray(q['world'],float);query=np.asarray(q['query_pixel_id']);partner=np.asarray(q['partner_pixel_id'])
  if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all() or query.shape!=(len(w),) or partner.shape!=query.shape:raise ValueError('finite endpoints with native UID')
  for ids in [query,partner]:
   if not np.issubdtype(ids.dtype,np.integer) or (ids<0).any() or (ids>=480*640).any():raise ValueError('pixel UID')
  last=t
  if not len(w):continue
  R=T[:3,:3]@np.diag([1,-1,-1]);local=(cores-T[:3,3])@R;metric=(w-T[:3,3])@R
  with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
  valid=np.isfinite(uv).all(1)&(local[:,2]>=.5)&(local[:,2]<=3)&(uv[:,0]>=0)&(uv[:,0]<640)&(uv[:,1]>=0)&(uv[:,1]<480)
  ids=np.flatnonzero(valid);pixels=np.column_stack([query%640,query//640]);d,j=cKDTree(pixels).query(uv[ids]);delta=metric[j,2]-local[ids,2];chosen=(d<=np.sqrt(2)+1e-9)&(delta>.2)
  for k,pi,dp,dz in zip(ids[chosen],j[chosen],d[chosen],delta[chosen]):proof[k].append(dict(source_frame=int(frame),time_s=t,query_pixel_id=int(query[pi]),partner_pixel_id=int(partner[pi]),pixel_distance=float(dp),core_opticalZ_m=float(local[k,2]),endpoint_opticalZ_m=float(metric[pi,2]),endpoint_minus_core_Z_m=float(dz),metric_endpoint_world=w[pi].tolist()))
 return np.array([bool(p) for p in proof]),proof
