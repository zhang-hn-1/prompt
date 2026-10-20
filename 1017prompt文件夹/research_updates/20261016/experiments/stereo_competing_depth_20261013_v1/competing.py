import numpy as np,cv2
FBASE=.6*584.8888888888889

def qualify(rgb,pred,packet,masks,now):
 t=float(now)
 if not np.isfinite(t) or t<0 or any(float(a['time_s'])!=t for a in [rgb,pred,packet]):raise ValueError('current clocks')
 q=packet['query_pixel_id'].astype(int);z=pred['depth_m'].ravel()[q];d=np.rot90(pred['disparity_px'],-1).ravel()[q];xy=np.column_stack([q%640,q//640]);reject=np.zeros(len(q),bool)
 dx,dy=np.meshgrid(np.arange(-3,4),np.arange(-3,4));offset=np.column_stack([dx.ravel(),dy.ravel()]);lg=cv2.cvtColor(rgb['left_rgb'],cv2.COLOR_RGB2GRAY).astype(np.float32);rg=cv2.cvtColor(rgb['right_rgb'],cv2.COLOR_RGB2GRAY).astype(np.float32)
 def sample(im,a):return cv2.remap(im,a[:,:,0].astype(np.float32),a[:,:,1].astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
 def inside(a):return ((a[:,:,0]>=0)&(a[:,:,0]<=639)&(a[:,:,1]>=0)&(a[:,:,1]<=479)).all(1)
 def cost(a,b):
  aa=a-a.mean(1,keepdims=True);bb=b-b.mean(1,keepdims=True);return (aa*bb).sum(1)/np.maximum(np.linalg.norm(aa,axis=1)*np.linalg.norm(bb,axis=1),1e-12),np.abs(a-b).mean(1)
 for start in range(0,len(q),2048):
  ix=slice(start,start+2048);a=xy[ix,None,:]+offset;b=a.astype(float);b[:,:,1]-=d[ix,None];l=sample(lg,a);r=sample(rg,b);pn,ps=cost(l,r);leftok=inside(a)&(l.std(1)>=2)&(sample(masks['left'].astype(np.float32),a)==0).all(1);hit=np.zeros(len(a),bool)
  for delta in [-.3,.3]:
   rz=z[ix]+delta;valid=(rz>=.5)&(rz<=3);mate=a.astype(float);mate[:,:,1]-=(FBASE/np.maximum(rz,.001))[:,None];rp=sample(rg,mate);rn,rs=cost(l,rp);supported=valid&leftok&inside(b)&inside(mate)&(rp.std(1)>=2)&(sample(masks['right'].astype(np.float32),mate)==0).all(1);hit|=supported&(rn>=pn-1e-6)&(rs<=ps+1e-6)
  reject[ix]=hit
 return dict(query=q,keep=~reject,time_s=np.array(t),frame=packet['frame'],excluded_is_unknown=np.array(True),certified_free=np.array(False))
