"""V2 interpolation support repair ONLY; original scientific V1 remains REJECT."""
import numpy as np,cv2
OPTICAL=np.diag([1.,-1.,-1.])
def project_depth(uv,z,current_pose,source_pose,K):
 uv=np.asarray(uv,float);rays=np.c_[uv,np.ones(len(uv))]@np.linalg.inv(K).T;A=np.asarray(current_pose,float);B=np.asarray(source_pose,float);world=A[:3,3]+(rays*np.asarray(z)[...,None])@(A[:3,:3]@OPTICAL).T;local=(world-B[:3,3])@(B[:3,:3]@OPTICAL);p=local@K.T
 with np.errstate(divide='ignore',invalid='ignore'):xy=p[:,:2]/p[:,2,None]
 return xy,local[:,2]
class TemporalRaySweep:
 def __init__(self,K):self.K=np.asarray(K,float).copy();self.depths=np.round(np.arange(.5,3.00001,.02),8)
 @staticmethod
 def stamp(p,now):
  t=float(p['time_s'])
  if not np.isfinite(t) or not np.isfinite(now) or t>now+1e-9:raise ValueError('future/invalid stamp before payload')
  return t
 def predict(self,current,sources,now):
  ct=self.stamp(current,now);st=[self.stamp(s,now) for s in sources]
  if len(sources)!=2 or abs(ct-now)>1e-9 or not all(0<=now-t<=2.+1e-9 and t<ct for t in st) or len(set(st))!=2:raise ValueError('two distinct causal source exposures required')
  rgb=np.asarray(current['rgb']);gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY).astype(np.float32);shape=gray.shape;C=np.asarray(current['pose'],float);cm=np.asarray(current['mask'],bool);h,w=shape
  depth=np.full(shape,np.nan,np.float32);best=np.full(shape,np.nan,np.float32);margin=best.copy()
  base=[float(np.linalg.norm(np.asarray(s['pose'])[:3,3]-C[:3,3])) for s in sources]
  if min(base)<.05:return dict(depth_m=depth,score=best,uniqueness_margin=margin,source_time_s=np.array(st),current_time_s=ct,baselines_m=np.array(base))
  if cm.shape!=shape:raise ValueError('current mask shape')
  yy,xx=np.indices(shape);uv=np.c_[xx.ravel(),yy.ravel()];mean=lambda x:cv2.boxFilter(x.astype(np.float32),-1,(7,7),normalize=True,borderType=cv2.BORDER_CONSTANT)
  mu=mean(gray);var=np.maximum(0,mean(gray*gray)-mu*mu);valid_current=(mean((~cm).astype(np.float32))>.999)&(var>=25)
  P=[np.asarray(s['pose'],float) for s in sources];sg=[cv2.cvtColor(np.asarray(s['rgb']),cv2.COLOR_RGB2GRAY).astype(np.float32) for s in sources];sm=[np.asarray(s['mask'],bool) for s in sources];rays=(np.c_[uv,np.ones(len(uv))]@np.linalg.inv(self.K).T)@(C[:3,:3]@OPTICAL).T;cost=np.full((len(self.depths),h,w),-np.inf,np.float32)
  for di,z in enumerate(self.depths):
   world=C[:3,3]+rays*z;score=np.ones(shape,np.float32)
   for B,g,m in zip(P,sg,sm):
    pu,pz=project_depth(uv,z,C,B,self.K);mx=pu[:,0].reshape(shape).astype(np.float32);my=pu[:,1].reshape(shape).astype(np.float32);inside=np.isfinite(mx)&np.isfinite(my)&(mx>=0)&(mx<=w-1)&(my>=0)&(my<=h-1)&(pz.reshape(shape)>0);mx=np.nan_to_num(mx,nan=-1000,posinf=-1000,neginf=-1000);my=np.nan_to_num(my,nan=-1000,posinf=-1000,neginf=-1000)
    rem=cv2.remap(g,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0);mask=cv2.remap((~m).astype(np.float32),mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)<=.999;support=mean((inside&~mask).astype(np.float32))>.999;mr=mean(rem);vr=np.maximum(0,mean(rem*rem)-mr*mr);cov=mean(gray*rem)-mu*mr
    with np.errstate(divide='ignore',invalid='ignore'):ncc=cov/np.sqrt(var*vr)
    a=world-C[:3,3];b=world-B[:3,3]
    with np.errstate(divide='ignore',invalid='ignore'):theta=np.degrees(np.arccos(np.clip(np.sum(a*b,axis=1)/(np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)),-1,1))).reshape(shape)
    okay=valid_current&support&(vr>=25)&np.isfinite(ncc)&(theta>=.5);score=np.minimum(score,np.where(okay,ncc,-np.inf))
   cost[di]=score
  ix=np.argmax(cost,axis=0);v=np.max(cost,axis=0);zbest=self.depths[ix];second=np.max(np.where(np.abs(self.depths[:,None,None]-zbest[None])>.100000001,cost,-np.inf),axis=0)
  with np.errstate(invalid='ignore'):delta=v-second
  accept=np.isfinite(v)&np.isfinite(second)&(v>=.8)&(delta>=.1);depth[accept]=zbest[accept];best[np.isfinite(v)]=v[np.isfinite(v)];margin[np.isfinite(delta)]=delta[np.isfinite(delta)]
  return dict(depth_m=depth,score=best,uniqueness_margin=margin,source_time_s=np.array(st),current_time_s=ct,baselines_m=np.array(base))
