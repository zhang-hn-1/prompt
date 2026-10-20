"""Positive silhouette support yields only MODEL possibility, never triangulation/free."""
import numpy as np

def possible_set(points,frame,time_s,history):
 p=np.asarray(points,float)
 if p.ndim!=2 or p.shape[1]!=3 or not np.isfinite(p).all() or not np.isfinite(time_s):raise ValueError('finite candidate volume and current clock')
 selected=[];seen=set()
 for r in history:
  if r['frame']>frame or r['time_s']>time_s:continue
  if r['frame'] in seen:continue
  seen.add(r['frame']);selected.append(r)
 support=np.zeros((len(selected),len(p)),bool);pixels=np.full((len(selected),len(p)), -1,np.int32);centres=[]
 for k,r in enumerate(selected):
  T=np.asarray(r['optical_world'],float)
  if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('finite past/current optical transform')
  positive=np.asarray(r['positive']);selfmask=np.asarray(r['self_mask'])
  if positive.shape!=(480,640) or selfmask.shape!=positive.shape or positive.dtype!=np.bool_ or selfmask.dtype!=np.bool_:raise ValueError('boolean measured masks')
  centres.append(T[:3,3].copy());q=(p-T[:3,3])@T[:3,:3];valid=(q[:,2]>=.5)&(q[:,2]<=3.);uv=np.zeros((len(q),2));uv[valid]=q[valid,:2]/q[valid,2:3]*584.8888888888889+[319.5,239.5];valid&=(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479);ids=np.flatnonzero(valid);xy=np.rint(uv[ids]).astype(int);ok=positive[xy[:,1],xy[:,0]]&~selfmask[xy[:,1],xy[:,0]];ids=ids[ok];xy=xy[ok];support[k,ids]=True;pixels[k,ids]=xy[:,1]*640+xy[:,0]
 keep=np.zeros(len(p),bool);baseline=np.zeros(len(p))
 for a in range(len(selected)):
  for b in range(a):
   distance=float(np.linalg.norm(centres[a]-centres[b]));both=support[a]&support[b];baseline[both]=np.maximum(baseline[both],distance)
   if distance>=.1:keep|=both
 return dict(keep=keep,support_count=support.sum(0),max_baseline_m=baseline,support_frames=[r['frame'] for r in selected],support_time_s=[r['time_s'] for r in selected],support_pixel_id=pixels[:,keep],world=p[keep].copy(),label='POSITIVE_CONE_MODEL_POSSIBILITY',certified_free=False,measured_stereo=False,full_3d_certified=False)
