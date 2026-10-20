"""One fixed RGB-only current stereo confidence test; not source admission/free space."""
import numpy as np
from qualified_hazard_consumer import subset

def filter_packet(packet,left_rgb,right_rgb,left_mask,right_mask,now):
 qt=float(packet['time_s']);t=float(now)
 if not np.isfinite(qt) or not np.isfinite(t) or qt>t or abs(t-qt)>1e-6:raise ValueError('current source clock before payload/RGB')
 l=np.asarray(left_rgb);r=np.asarray(right_rgb);lm=np.asarray(left_mask);rm=np.asarray(right_mask)
 if l.shape!=r.shape or l.ndim!=3 or l.shape[2]!=3 or lm.shape!=l.shape[:2] or rm.shape!=lm.shape or lm.dtype!=bool or rm.dtype!=bool:raise ValueError('two RGB/current boolean CAD masks')
 h,w=l.shape[:2]
 if int(packet['frame_width'])!=w:raise ValueError('pixel identity width')
 q=np.asarray(packet['query_pixel_id']);p=np.asarray(packet['partner_pixel_id'])
 if q.shape!=p.shape or q.ndim!=1 or not np.issubdtype(q.dtype,np.integer) or not np.issubdtype(p.dtype,np.integer):raise ValueError('integer pixel identity')
 if ((q<0)|(p<0)|(q>=h*w)|(p>=h*w)).any():raise ValueError('pixel identity bounds')
 ly,lx=q//w,q%w;ry,rx=p//w,p%w
 off=np.array([(dy,dx) for dy in range(-4,5) for dx in range(-4,5)])
 def patches(rgb,mask,y,x):
  valid=(y>=4)&(y<h-4)&(x>=4)&(x<w-4)
  yy=np.clip(y[:,None]+off[None,:,0],0,h-1);xx=np.clip(x[:,None]+off[None,:,1],0,w-1)
  valid&=~mask[yy,xx].any(1)
  gray=rgb[yy,xx].astype(float)@np.array([.299,.587,.114]);centered=gray-gray.mean(1)[:,None];std=centered.std(1)
  return centered,std,valid
 a,sa,va=patches(l,lm,ly,lx);b,sb,vb=patches(r,rm,ry,rx)
 def corr(x,sx,y,sy):
  den=sx*sy*81
  return np.divide(np.sum(x*y,1),den,out=np.full(len(x),-np.inf),where=den>0)
 ncc=corr(a,sa,b,sb);support=va&vb;best=np.full(len(q),-np.inf)
 for delta in [-4,-3,-2,2,3,4]:
  b2,s2,v2=patches(r,rm,ry+delta,rx);support&=v2;best=np.maximum(best,corr(a,sa,b2,s2))
 texture=(sa>=3)&(sb>=3);good=support&texture&(ncc>=.8)
 with np.errstate(invalid='ignore'):unique=(ncc-best)>=.05
 keep=good&unique
 d=dict(time_s=t,n=len(q),retained=int(keep.sum()),invalid_support=int((~support).sum()),low_texture=int((support&~texture).sum()),low_correlation=int((support&texture&(ncc<.8)).sum()),non_unique=int((good&~unique).sum()),window_px=9,ncc_min=.8,texture_std_min=3.,uniqueness_margin=.05,source_model_admitted=False,certified_free=False)
 return subset(packet,keep),d
