"""Sparse read-only instrumentation of frozen V1, never accepts source labels."""
import cv2,numpy as np
from multiview_rgb import select_sources,plane_terms,projected


def trace_rgb(reference,sources,anchors_yx,*,depths,focal,cx,cy,patch_radius,texture_min,
              parallax_min,min_sources,max_cost,slack,max_interval):
    sources=select_sources(reference,sources);depths=np.asarray(depths,float)
    if len(depths)<3 or not np.all(np.diff(depths)>0) or not np.all(np.isfinite(depths)) or depths[0]<=0:
        raise ValueError('need increasing positive finite depth grid')
    anchors=np.asarray(anchors_yx,int).reshape(-1,2);n=len(anchors);k=2*patch_radius+1
    gray=cv2.cvtColor(np.asarray(reference['rgb']),cv2.COLOR_RGB2GRAY).astype(np.float32);h,w=gray.shape
    mask=np.asarray(reference['mask'],bool)
    if mask.shape!=(h,w):raise ValueError('reference mask mismatch')
    dy,dx=np.mgrid[-patch_radius:patch_radius+1,-patch_radius:patch_radius+1]
    qy=anchors[:,0,None,None]+dy;qx=anchors[:,1,None,None]+dx
    inside=(qy>=0)&(qy<h)&(qx>=0)&(qx<w);sy=np.clip(qy,0,h-1);sx=np.clip(qx,0,w-1)
    a=gray[sy,sx];body=mask[sy,sx]
    def mean(patches):
        boxes=cv2.boxFilter(patches.reshape(n*k,k),-1,(k,k),normalize=True,borderType=cv2.BORDER_CONSTANT)
        return boxes[np.arange(n)*k+patch_radius,patch_radius]
    am=mean(a);av=np.maximum(mean(a*a)-am*am,0);refstd=np.sqrt(av)
    refvalid=inside.all(axis=(1,2))&~body.any(axis=(1,2))&(refstd>=texture_min)
    shape=(n,len(depths),len(sources))
    source_cost=np.full(shape,np.inf,np.float32);unfiltered=np.full(shape,np.nan,np.float32)
    valid=np.zeros(shape,bool);foot=np.zeros(shape,bool);texture=np.zeros(shape,np.float32);angles=np.zeros(shape,np.float64)
    for vi,s in enumerate(sources):
        sg=cv2.cvtColor(np.asarray(s['rgb']),cv2.COLOR_RGB2GRAY).astype(np.float32);sm=np.asarray(s['mask'],bool)
        if sg.shape!=(h,w) or sm.shape!=(h,w):raise ValueError('source dimensions mismatch')
        full=plane_terms(reference['pose'],s['pose'],h,w,focal,cx,cy)
        terms=(full[0][sy,sx],full[1][sy,sx],full[2],full[3]);del full
        for di,z in enumerate(depths):
            x,y,angle,positive=projected(terms,z,focal,cx,cy)
            finite=positive&np.isfinite(x)&np.isfinite(y);x=np.where(finite,x,-1.);y=np.where(finite,y,-1.)
            isin=finite&(x>=0)&(x<w-1)&(y>=0)&(y<h-1)
            x0=np.floor(np.clip(x,0,w-2)).astype(int);y0=np.floor(np.clip(y,0,h-2)).astype(int)
            support=isin&~(sm[y0,x0]|sm[y0+1,x0]|sm[y0,x0+1]|sm[y0+1,x0+1])
            warped=cv2.remap(sg,x.astype(np.float32).reshape(n*k,k),y.astype(np.float32).reshape(n*k,k),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT).reshape(n,k,k)
            wm=mean(warped);wv=np.maximum(mean(warped*warped)-wm*wm,0.);cov=mean(a*warped)-am*wm
            denom=np.sqrt(av*wv);corr=np.divide(cov,denom,out=np.zeros_like(cov),where=denom>0)
            cost=1.-np.clip(corr,-1.,1.)
            footprint=support.all(axis=(1,2));theta=angle[:,patch_radius,patch_radius]
            good=refvalid&footprint&(np.sqrt(wv)>=texture_min)&(theta>=parallax_min)
            source_cost[:,di,vi]=np.where(good,cost,np.inf);unfiltered[:,di,vi]=cost
            valid[:,di,vi]=good;foot[:,di,vi]=footprint;texture[:,di,vi]=np.sqrt(wv);angles[:,di,vi]=theta
    aggregate=np.full((n,len(depths)),np.inf,np.float32);bits=np.zeros(aggregate.shape,np.uint8)
    if len(sources):
        count=np.isfinite(source_cost).sum(axis=2);ordered=np.sort(source_cost,axis=2)
        lo=np.take_along_axis(ordered,np.maximum((count-1)//2,0)[:,:,None],axis=2)[:,:,0]
        hi=np.take_along_axis(ordered,np.maximum(count//2,0)[:,:,None],axis=2)[:,:,0]
        aggregate=np.where(count>=min_sources,(lo+hi)/2,np.inf).astype(np.float32)
        for vi in range(len(sources)):bits|=valid[:,:,vi].astype(np.uint8)*np.uint8(1<<vi)
    best_index=np.argmin(aggregate,axis=1);best=aggregate[np.arange(n),best_index];finite=np.isfinite(best)
    raw=np.where(finite,depths[best_index],np.nan).astype(np.float32)
    lower=depths.copy();upper=depths.copy();mid=(depths[:-1]+depths[1:])/2;lower[1:]=mid;upper[:-1]=mid
    competitive=np.isfinite(aggregate)&(aggregate<=best[:,None]+slack)
    lo=np.min(np.where(competitive,lower[None],np.inf),axis=1);hi=np.max(np.where(competitive,upper[None],-np.inf),axis=1)
    raynorm=np.sqrt(1+((anchors[:,1]-cx)/focal)**2+((anchors[:,0]-cy)/focal)**2)
    interval=np.where(finite,(hi-lo)*raynorm,np.inf)
    confidence=finite&(best_index>0)&(best_index<len(depths)-1)&(best<=max_cost)&(interval<=max_interval+1e-8)
    return dict(depths_m=depths,cost=aggregate,view_bits=bits,source_cost=source_cost,source_unfiltered_cost=unfiltered,
        source_valid=valid,source_footprint_valid=foot,source_std=texture,source_parallax_deg=angles,
        reference_std=refstd,reference_valid=refvalid,best_index=best_index,raw_depth_m=raw,
        confidence=confidence,interval_ray_width_m=interval.astype(np.float32),competitive=competitive)
