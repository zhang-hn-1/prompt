"""Readonly current appearance corroboration of OLD static geometry, not new depth."""
import cv2,numpy as np

class PastPointRGBMemory:
 def __init__(self):
  self.seeded=False;self.last_time=-np.inf;self.world=np.empty((0,3));self.ids=[np.empty(0,int),np.empty(0,int)];self.geometry_time=np.nan;self.source_frame=-1
 @staticmethod
 def _stamp(packet,now):
  t=float(packet['time_s'])
  if not np.isfinite(t) or not np.isfinite(now) or t>now+1e-9:raise ValueError('future/invalid time before payload')
  return t
 def seed(self,source,rgb,now,*,qualified):
  st=self._stamp(source,now);rt=self._stamp(rgb,now)
  if abs(st-rt)>1e-9 or not 0<=now-st<=.5 or now<self.last_time:raise ValueError('seed synchronization/age')
  if self.seeded:raise ValueError('single seed only; geometry age cannot refresh')
  if not qualified:return
  w=np.asarray(source['world']);sel=(w[:,2]>.05)&(w[:,2]<=1.5);self.world=w[sel].copy();self.ids=[np.asarray(source[k])[sel].copy() for k in ['query_pixel_id','partner_pixel_id']];self.gray=[cv2.cvtColor(np.asarray(rgb[k]).copy(),cv2.COLOR_RGB2GRAY) for k in ['left_rgb','right_rgb']];self.geometry_time=st;self.source_frame=int(source['frame']);self.seeded=True
 def query(self,rgb,pose,mask,now):
  rt=self._stamp(rgb,now);pt=self._stamp(pose,now)
  if abs(rt-now)>1e-9 or abs(pt-now)>1e-9 or now<self.last_time:raise ValueError('current synchronization/rewind')
  self.last_time=now;n=len(self.world);scores=np.full((n,2),np.nan);std=np.full((n,2,2),np.nan);ok=np.ones(n,bool);uvs=[np.full((n,2),np.nan) for _ in range(2)];zs=[np.full(n,np.nan) for _ in range(2)];expired=bool(self.seeded and now-self.geometry_time>10.)
  if self.seeded and now>self.geometry_time:
   for ei,eye in enumerate(['left','right']):
    T=np.asarray(pose[eye+'_camera_usd_world']);R=T[:3,:3]@np.diag([1,-1,-1]);local=(self.world-T[:3,3])@R
    with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*584.8888888888889+[319.5,239.5]
    uvs[ei]=uv;zs[ei]=local[:,2];valid=(local[:,2]>0)&np.isfinite(uv).all(axis=1)&(uv[:,0]>=3)&(uv[:,0]<=636)&(uv[:,1]>=3)&(uv[:,1]<=476);current=cv2.cvtColor(rgb[eye+'_rgb'],cv2.COLOR_RGB2GRAY)
    for j in np.flatnonzero(valid):
     x,y=np.rint(uv[j]).astype(int);ox=int(self.ids[ei][j]%640);oy=int(self.ids[ei][j]//640)
     if mask[eye][y,x] or not(3<=ox<=636 and 3<=oy<=476):valid[j]=False;continue
     a=cv2.getRectSubPix(self.gray[ei],(7,7),(float(ox),float(oy))).astype(float).ravel();b=cv2.getRectSubPix(current,(7,7),tuple(uv[j].astype(float))).astype(float).ravel();std[j,ei]=[a.std(),b.std()];aa=a-a.mean();bb=b-b.mean();den=np.linalg.norm(aa)*np.linalg.norm(bb);scores[j,ei]=float(aa@bb/den) if den>0 else np.nan
    ok&=valid
  else:ok[:]=False
  accepted=ok&np.all(std>=5,axis=(1,2))&np.all(scores>=.8,axis=1)&(not expired)
  return dict(past_world=self.world.copy(),past_query_id=self.ids[0].copy(),past_partner_id=self.ids[1].copy(),source_geometry_time_s=self.geometry_time,source_frame=self.source_frame,current_appearance_time_s=rt,left_uv=uvs[0],right_uv=uvs[1],left_z=zs[0],right_z=zs[1],bilateral_fov_and_mask_valid=ok,patch_std=std,patch_NCC=scores,appearance_corroborated=accepted,geometry_expired=expired,certified_free=False,source_model_admitted=False)
