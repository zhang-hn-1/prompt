"""Current-pair competing flat-floor explanation; no observed-free claim.
The ground plane is the existing TaskH prior; all appearance evidence is current.
"""
import numpy as np
import cv2
F=584.8888888888889;C=np.array([319.5,239.5]);FLIP=np.diag([1,-1,-1])

def _poses(poses,now):
 t=float(poses['time_s'])
 if not np.isfinite(now) or not np.isfinite(t) or t!=now:raise ValueError('current paired pose before payload')
 out=[]
 for eye in ['left','right']:
  T=np.asarray(poses[eye+'_camera_usd_world'],float)
  if T.shape!=(4,4) or not np.isfinite(T).all():raise ValueError('finite camera transform')
  out.append((T[:3,:3]@FLIP,T[:3,3]))
 return out

def ground_partners(q,poses,now):
 (L,lp),(R,rp)=_poses(poses,now);q=np.asarray(q,float)
 if q.ndim!=2 or q.shape[1]!=2 or not np.isfinite(q).all():raise ValueError('finite query pixels')
 rays=np.column_stack([(q-C)/F,np.ones(len(q))]);world_rays=rays@L.T
 s=np.divide(-lp[2],world_rays[:,2],out=np.full(len(q),np.nan),where=abs(world_rays[:,2])>1e-9)
 world=lp+world_rays*s[:,None];optical=(world-rp)@R
 v=F*optical[:,:2]/optical[:,2,None]+C
 valid=np.isfinite(v).all(1)&np.isfinite(s)&(s>=.5)&(s<=3)&(optical[:,2]>0)&(v[:,0]>=0)&(v[:,0]<=639)&(v[:,1]>=0)&(v[:,1]<=479)
 return v,valid

def qualify_points(left,right,q,disparity,depth,poses,now,masks):
 (L,lp),_= _poses(poses,now)
 q=np.asarray(q,float);d=np.asarray(disparity,float);z=np.asarray(depth,float)
 if q.shape!=(len(d),2) or z.shape!=d.shape or not np.isfinite(q).all() or not np.isfinite(d).all() or not np.isfinite(z).all():raise ValueError('aligned finite measurement')
 if any(np.asarray(x).shape!=(480,640,3) or np.asarray(x).dtype!=np.uint8 for x in [left,right]):raise ValueError('paired RGB')
 if any(np.asarray(masks[e]).shape!=(480,640) or np.asarray(masks[e]).dtype!=bool for e in ['left','right']):raise ValueError('current masks')
 dx,dy=np.meshgrid(np.arange(-3,4),np.arange(-3,4));offset=np.column_stack([dx.ravel(),dy.ravel()]);a=q[:,None,:]+offset
 b=a.copy();b[:,:,1]-=d[:,None]
 floor,valid=ground_partners(a.reshape(-1,2),poses,now);floor=floor.reshape(-1,49,2);valid=valid.reshape(-1,49)
 def inside(x):return ((x[:,:,0]>=0)&(x[:,:,0]<=639)&(x[:,:,1]>=0)&(x[:,:,1]<=479)).all(1)
 def sample(img,coords):
  return cv2.remap(img,np.nan_to_num(coords[:,:,0],nan=-1).astype(np.float32),np.nan_to_num(coords[:,:,1],nan=-1).astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
 lg=cv2.cvtColor(left,cv2.COLOR_RGB2GRAY).astype(np.float32);rg=cv2.cvtColor(right,cv2.COLOR_RGB2GRAY).astype(np.float32)
 x=sample(lg,a);y=sample(rg,b);g=sample(rg,floor)
 def ncc(x,y):
  aa=x-x.mean(1,keepdims=True);bb=y-y.mean(1,keepdims=True)
  return (aa*bb).sum(1)/np.maximum(np.linalg.norm(aa,axis=1)*np.linalg.norm(bb,axis=1),1e-12)
 pred=ncc(x,y);prior=ncc(x,g)
 unmasked=(sample(masks['left'].astype(np.float32),a)==0).all(1)&(sample(masks['right'].astype(np.float32),floor)==0).all(1)
 supported=inside(a)&inside(b)&valid.all(1)&unmasked&(x.std(1)>=2)&(g.std(1)>=2)&(prior>=.9)
 rays=np.column_stack([(q-C)/F,np.ones(len(q))]);height=lp[2]+(rays@L.T)[:,2]*z
 ambiguous=(abs(height)>.05)&supported&(prior>=pred)
 return dict(ambiguous=ambiguous,pred_ncc=pred,floor_ncc=prior,floor_supported=supported,predicted_height=height)

def qualify_prediction(pred,left,right,poses,now,masks):
 t=float(pred['time_s'])
 if not np.isfinite(t) or t!=now:raise ValueError('current prediction before payload')
 _poses(poses,now)
 z=np.asarray(pred['depth_m']);confidence=np.asarray(pred['confidence']);d=np.ascontiguousarray(np.rot90(pred['disparity_px'],-1))
 if z.shape!=(480,640) or confidence.shape!=z.shape or confidence.dtype!=bool or d.shape!=z.shape:raise ValueError('native prediction geometry')
 ids=np.flatnonzero(confidence&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(d)&(d>0))
 ambiguous=np.zeros(z.shape,bool)
 for start in range(0,len(ids),2048):
  batch=ids[start:start+2048];q=np.column_stack([batch%640,batch//640]);v=qualify_points(left,right,q,d.ravel()[batch],z.ravel()[batch],poses,now,masks)
  ambiguous.ravel()[batch]=v['ambiguous']
 out=dict(pred);out['confidence']=confidence.copy()&~ambiguous;out['depth_m']=z.copy();out['depth_m'][ambiguous]=np.nan
 out.update(parent_confidence=confidence.copy(),ground_ambiguous=ambiguous,ground_ambiguity_count=int(ambiguous.sum()),flat_ground_prior_explicit=True,ambiguous_is_unknown_not_free=True)
 return out
