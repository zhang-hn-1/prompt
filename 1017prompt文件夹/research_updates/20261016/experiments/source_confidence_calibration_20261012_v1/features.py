"""Causal current paired-image features; no renderer or future inputs."""
import numpy as np,cv2
def features(rgb,pred,packet):
 # Current input only; caller must not supply renderer truth.
 l=cv2.cvtColor(rgb['left_rgb'],cv2.COLOR_RGB2GRAY).astype(float);r=cv2.cvtColor(rgb['right_rgb'],cv2.COLOR_RGB2GRAY).astype(float)
 q=packet['query_pixel_id'].astype(int);p=packet['partner_pixel_id'].astype(int);x=q%640;y=q//640;xx=p%640;yy=p//640
 z=pred['depth_m'][y,x].astype(float);disparity=pred['disparity_px'];disparity=np.rot90(disparity,-1) if disparity.shape!=pred['depth_m'].shape else disparity;assert disparity.shape==pred['depth_m'].shape;disp=disparity[y,x].astype(float)
 def moment(im):
  m=cv2.boxFilter(im,-1,(7,7),normalize=True);sd=np.sqrt(np.maximum(cv2.boxFilter(im*im,-1,(7,7),normalize=True)-m*m,0));return m,sd
 ml,sl=moment(l);mr,sr=moment(r);valid=(x>=3)&(x<637)&(y>=3)&(y<477)&(xx>=3)&(xx<637)&(yy>=3)&(yy<477)
 ncc=np.zeros(len(q));sad=np.zeros(len(q))
 for a,b in [(a,b) for a in range(-3,4) for b in range(-3,4)]:
  lv=l[np.clip(y+a,0,479),np.clip(x+b,0,639)];rv=r[np.clip(yy+a,0,479),np.clip(xx+b,0,639)];ncc+=(lv-ml[y,x])*(rv-mr[yy,xx]);sad+=np.abs(lv-rv)
 ncc=ncc/(49*np.maximum(sl[y,x]*sr[yy,xx],1));sad/=49*255
 zz=np.nan_to_num(pred['depth_m'].astype(float),nan=0,posinf=0,neginf=0);dx=np.abs(zz[y,np.clip(x+1,0,639)]-zz[y,np.clip(x-1,0,639)]);dy=np.abs(zz[np.clip(y+1,0,479),x]-zz[np.clip(y-1,0,479),x]);world=packet['world']
 X=np.column_stack([z,disp/448,sl[y,x]/255,sr[yy,xx]/255,np.clip(ncc,-1,1),sad,dx,dy,world[:,2],packet['sigma_m'],np.abs(ml[y,x]-mr[yy,xx])/255,valid.astype(float)]);assert np.isfinite(X).all();return X,q
