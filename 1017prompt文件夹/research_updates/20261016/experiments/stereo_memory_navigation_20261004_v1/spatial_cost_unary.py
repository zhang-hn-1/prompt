"""Causal RGB-only plane sweep with explicit known camera poses (diagnostic layer)."""
import cv2
import numpy as np


def optical_rotation(pose):
    return np.asarray(pose, float)[:3,:3] @ np.diag([1.,-1.,-1.])


def plane_terms(reference_pose,source_pose,height,width,focal,cx,cy):
    yy,xx=np.mgrid[:height,:width]
    rays=np.stack([(xx-cx)/focal,(yy-cy)/focal,np.ones_like(xx)],axis=-1)
    world_ray=rays @ optical_rotation(reference_pose).T
    delta=np.asarray(reference_pose)[:3,3]-np.asarray(source_pose)[:3,3]
    inverse=np.linalg.inv(optical_rotation(source_pose))
    return world_ray,world_ray@inverse.T,delta,delta@inverse.T


def projected(terms,depth,focal,cx,cy):
    world_ray,source_ray,delta,source_delta=terms
    optical=depth*source_ray+source_delta
    positive=np.isfinite(optical).all(axis=-1)&(optical[...,2]>0)
    with np.errstate(divide='ignore',invalid='ignore'):
        x=optical[...,0]/optical[...,2]*focal+cx
        y=optical[...,1]/optical[...,2]*focal+cy
    source_world_ray=depth*world_ray+delta
    angle=np.degrees(np.arctan2(np.linalg.norm(np.cross(world_ray,source_world_ray),axis=-1),
                               np.sum(world_ray*source_world_ray,axis=-1)))
    return x,y,angle,positive


def project_plane(reference_pose,source_pose,depth,height,width,focal,cx,cy):
    return projected(plane_terms(reference_pose,source_pose,height,width,focal,cx,cy),depth,focal,cx,cy)


def select_sources(reference,sources):
    t=float(reference['time_s']);number=int(reference['frame']);eye=reference['eye']
    if not np.isfinite(t):raise ValueError('nonfinite reference time')
    selected=[];seen=set()
    for s in sources:
        st=float(s['time_s'])
        if not np.isfinite(st):raise ValueError('nonfinite source time')
        if st>t:continue # Exclude future before reading even identity/payload.
        if st==t and s['eye']==eye:continue
        identity=(st,int(s['frame']),s['eye'])
        if identity in seen:raise ValueError('duplicate view')
        if st==t and identity[1]!=number:raise ValueError('inconsistent simultaneous capture')
        seen.add(identity);selected.append(s)
    if len(selected)>8:raise ValueError('view bitmap limited to eight sources')
    return selected


def build_volume(reference,sources,*,depths,focal,cx,cy,patch_radius,texture_min,
                parallax_min,min_sources,max_cost,slack,max_interval):
    sources=select_sources(reference,sources)
    depths=np.asarray(depths,float)
    if len(depths)<3 or not np.all(np.diff(depths)>0) or not np.all(np.isfinite(depths)) or depths[0]<=0:
        raise ValueError('need increasing positive finite depth grid')
    gray=cv2.cvtColor(np.asarray(reference['rgb']),cv2.COLOR_RGB2GRAY).astype(np.float32)
    h,w=gray.shape;mask=np.asarray(reference['mask'],bool)
    if mask.shape!=(h,w):raise ValueError('reference mask mismatch')
    k=2*patch_radius+1;count=k*k
    def mean(a):return cv2.boxFilter(a,-1,(k,k),normalize=True,borderType=cv2.BORDER_CONSTANT)
    refmean=mean(gray);refvar=np.maximum(mean(gray*gray)-refmean*refmean,0.)
    refvalid=(mean((~mask).astype(np.float32))>=1.-1e-6)&(np.sqrt(refvar)>=texture_min)
    yy,xx=np.mgrid[:h,:w];raynorm=np.sqrt(1+((xx-cx)/focal)**2+((yy-cy)/focal)**2)
    prepared=[]
    for s in sources:
        sg=cv2.cvtColor(np.asarray(s['rgb']),cv2.COLOR_RGB2GRAY).astype(np.float32)
        sm=np.asarray(s['mask'],bool)
        if sg.shape!=(h,w) or sm.shape!=(h,w):raise ValueError('source dimensions mismatch')
        prepared.append((sg,sm,plane_terms(reference['pose'],s['pose'],h,w,focal,cx,cy)))
    volume=np.full((len(depths),h,w),np.inf,np.float32);bits=np.zeros(volume.shape,np.uint8)
    for di,z in enumerate(depths):
        viewcost=[];bitmap=np.zeros((h,w),np.uint8)
        for vi,(sg,sm,terms) in enumerate(prepared):
            x,y,angle,positive=projected(terms,z,focal,cx,cy)
            finite=positive&np.isfinite(x)&np.isfinite(y)
            x=np.where(finite,x,-1.);y=np.where(finite,y,-1.)
            inside=finite&(x>=0)&(x<w-1)&(y>=0)&(y<h-1)
            x0=np.floor(np.clip(x,0,w-2)).astype(int);y0=np.floor(np.clip(y,0,h-2)).astype(int)
            valid=inside&~(sm[y0,x0]|sm[y0+1,x0]|sm[y0,x0+1]|sm[y0+1,x0+1])
            warped=cv2.remap(sg,x.astype(np.float32),y.astype(np.float32),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            wm=mean(warped);wv=np.maximum(mean(warped*warped)-wm*wm,0.)
            cov=mean(gray*warped)-refmean*wm
            denom=np.sqrt(refvar*wv)
            corr=np.divide(cov,denom,out=np.zeros_like(cov),where=denom>0)
            good=refvalid&(mean(valid.astype(np.float32))>=1.-1e-6)&(np.sqrt(wv)>=texture_min)&(angle>=parallax_min)
            cost=np.where(good,1.-np.clip(corr,-1.,1.),np.inf).astype(np.float32)
            viewcost.append(cost);bitmap|=good.astype(np.uint8)*np.uint8(1<<vi)
        if viewcost:
            sortedcost=np.sort(np.stack(viewcost),axis=0);n=np.isfinite(sortedcost).sum(axis=0)
            a=np.take_along_axis(sortedcost,np.maximum((n-1)//2,0)[None],axis=0)[0]
            b=np.take_along_axis(sortedcost,np.maximum(n//2,0)[None],axis=0)[0]
            volume[di]=np.where(n>=min_sources,(a+b)/2,np.inf)
            bits[di]=bitmap
    return volume,bits,raynorm
