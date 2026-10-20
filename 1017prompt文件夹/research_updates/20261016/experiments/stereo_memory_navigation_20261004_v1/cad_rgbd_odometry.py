"""Independent metric RGB-D increment, no world poses or future feedback."""
import numpy as np
from stereo_vo import temporal_matches,pose_from_matches

def estimate_rgbd_increment(previous,current,K):
 t0,t1=float(previous['time_s']),float(current['time_s'])
 if not np.isfinite([t0,t1]).all() or t1<=t0:raise ValueError('strictly increasing finite exposure times required')
 xy=np.asarray(previous['xy']);q=np.asarray(current['xy']);depth=np.asarray(previous['depth_m']);valid=np.asarray(previous['valid'],bool)
 if depth.shape!=(480,640) or valid.shape!=depth.shape:raise ValueError('native depth dimensions')
 matches=temporal_matches(xy,previous['desc'],q,current['desc']);points=[];pixels=[];used=[];inv=np.linalg.inv(K)
 for i,j in matches:
  x,y=np.rint(xy[i]).astype(int)
  if not (0<=x<640 and 0<=y<480) or not valid[y,x]:continue
  z=float(depth[y,x])
  if not np.isfinite(z) or z<=0:continue
  points.append((inv@np.r_[xy[i],1.])*z);pixels.append(q[j]);used.append([i,j])
 result=pose_from_matches(points,pixels,K)
 result.update(temporal_matches=len(matches),depth_supported_matches=len(points),matches=used)
 return result
