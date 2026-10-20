"""Past RGB positive consistency; abstention only, not triangulation/free space."""
import numpy as np

def supports(points, frame, time_s, history):
 points=np.asarray(points,float)
 if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():raise ValueError('finite world points')
 frames=[[] for _ in points]; centres=[[] for _ in points]
 seen=set()
 for record in history:
  # Check acquisition metadata before touching a future payload.
  if record['frame']>=frame or record['time_s']>=time_s:continue
  if record['frame'] in seen:raise ValueError('distinct exposure IDs required')
  seen.add(record['frame'])
  T=np.asarray(record['optical_world'],float)
  q=(points-T[:3,3])@T[:3,:3]
  valid=(q[:,2]>=.5)&(q[:,2]<=3.)
  uv=np.zeros((len(q),2));uv[valid]=q[valid,:2]/q[valid,2:3]*584.8888888888889+[319.5,239.5]
  valid &= (uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)
  ids=np.flatnonzero(valid);xy=np.rint(uv[ids]).astype(int)
  ids=ids[record['positive'][xy[:,1],xy[:,0]] & ~record['self_mask'][xy[:,1],xy[:,0]]]
  for j in ids:
   frames[j].append(record['frame']);centres[j].append(T[:3,3].copy())
 keep=np.zeros(len(points),bool);baseline=np.zeros(len(points))
 for j,c in enumerate(centres):
  if len(c)>=2:
   c=np.array(c);baseline[j]=np.linalg.norm(c[:,None]-c[None,:],axis=-1).max();keep[j]=baseline[j]>=.1
 return dict(keep=keep,support_frames=frames,max_support_baseline_m=baseline)
