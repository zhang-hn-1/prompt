"""RGB patch evidence for a known-plane hypothesis; not measured stereo depth."""
import numpy as np
import cv2
from multiview_rgb import select_sources,plane_terms,projected,optical_rotation

def predict_floor(reference,sources,*,focal,cx,cy,patch_radius,min_sources,parallax_min,texture_min,max_cost,depth_min,depth_max):
    sources=select_sources(reference,sources);rgb=np.asarray(reference['rgb']);gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY).astype(np.float32);mask=np.asarray(reference['mask'],bool)
    if mask.shape!=gray.shape:raise ValueError('image/mask shape mismatch')
    h,w=gray.shape;yy,xx=np.mgrid[:h,:w];rays=np.stack([(xx-cx)/focal,(yy-cy)/focal,np.ones((h,w))],axis=-1);pose=np.asarray(reference['pose'],float)
    worldray=rays@optical_rotation(pose).T
    depth=np.divide(-pose[2,3],worldray[...,2],out=np.full((h,w),np.nan),where=abs(worldray[...,2])>1e-12)
    geom=np.isfinite(depth)&(depth>=depth_min)&(depth<=depth_max)
    k=patch_radius*2+1
    mean=lambda a:cv2.boxFilter(a,-1,(k,k),normalize=True,borderType=cv2.BORDER_CONSTANT)
    refmean=mean(gray);refvar=np.maximum(mean(gray*gray)-refmean*refmean,0)
    eligible=geom&(mean((~mask).astype(np.float32))>=1.-1e-6)&(np.sqrt(refvar)>=texture_min)
    costs=[];bits=np.zeros((h,w),np.uint8);supports=np.zeros((h,w),np.uint8)
    for vi,s in enumerate(sources):
        sg=cv2.cvtColor(np.asarray(s['rgb']),cv2.COLOR_RGB2GRAY).astype(np.float32);sm=np.asarray(s['mask'],bool)
        if sg.shape!=(h,w) or sm.shape!=(h,w):raise ValueError('source image/mask shape mismatch')
        x,y,angle,positive=projected(plane_terms(pose,s['pose'],h,w,focal,cx,cy),np.where(geom,depth,1.)[...,None],focal,cx,cy)
        valid=geom&positive&np.isfinite(x)&np.isfinite(y)&(x>=0)&(x<w-1)&(y>=0)&(y<h-1)
        x=np.where(valid,x,-1.);y=np.where(valid,y,-1.)
        x0=np.floor(np.clip(x,0,w-2)).astype(int);y0=np.floor(np.clip(y,0,h-2)).astype(int)
        valid&=~(sm[y0,x0]|sm[y0+1,x0]|sm[y0,x0+1]|sm[y0+1,x0+1])
        warp=cv2.remap(sg,x.astype(np.float32),y.astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        wm=mean(warp);wv=np.maximum(mean(warp*warp)-wm*wm,0);cov=mean(gray*warp)-refmean*wm;denom=np.sqrt(refvar*wv)
        correlation=np.divide(cov,denom,out=np.zeros_like(cov),where=denom>0);cost=1-np.clip(correlation,-1,1)
        good=eligible&(mean(valid.astype(np.float32))>=1-1e-6)&(np.sqrt(wv)>=texture_min)&(angle>=parallax_min)&(cost<=max_cost)
        costs.append(np.where(good,cost,np.inf));supports+=good.astype(np.uint8);bits|=good.astype(np.uint8)*np.uint8(1<<vi)
    confidence=eligible&(supports>=min_sources)
    cost=np.min(np.stack(costs),axis=0) if costs else np.full((h,w),np.inf,np.float32)
    return dict(plane_axis_depth_m=np.where(geom,depth,np.nan).astype(np.float32),depth_m=np.where(confidence,depth,np.nan).astype(np.float32),confidence=confidence,cost=cost.astype(np.float32),view_bits=bits,support_count=supports)
