"""Eight-path inverse-depth regularization; priors never qualify unsupported pixels."""
import numpy as np
from spatial_cost_unary import build_volume

def aggregate_cost(volume, p1=.05, p2=.20):
    cost=np.asarray(volume,np.float32)
    if cost.ndim!=3 or min(cost.shape)<1 or np.isnan(cost).any() or np.isneginf(cost).any():
        raise ValueError('cost must be nonempty D,H,W, finite or positive infinity')
    if not np.isfinite([p1,p2]).all() or not 0<=p1<=p2:
        raise ValueError('need finite 0<=P1<=P2')
    def path(c,sign,shift):
        d,h,w=c.shape;out=np.full_like(c,np.inf);prev=np.full((d,w),np.inf,np.float32)
        for y in (range(h) if sign==1 else range(h-1,-1,-1)):
            prior=np.full_like(prev,np.inf)
            if shift==0:prior=prev
            elif shift==1:prior[:,1:]=prev[:,:-1]
            else:prior[:,:-1]=prev[:,1:]
            low=np.min(prior,axis=0);active=np.isfinite(low)
            lower=np.full_like(prior,np.inf);upper=np.full_like(prior,np.inf)
            lower[1:]=prior[:-1]+p1;upper[:-1]=prior[1:]+p1
            transition=np.minimum(np.minimum(prior,lower),np.minimum(upper,low[None]+p2))
            # Entirely invalid predecessors restart; invalid unary labels never fill.
            normalized=np.zeros_like(prior)
            np.subtract(transition,low[None],out=normalized,where=active[None])
            out[:,y,:]=c[:,y,:]+normalized;prev=out[:,y,:]
        return out
    total=np.zeros_like(cost)
    for sign in (-1,1):
        for shift in (-1,0,1):total+=path(cost,sign,shift)
        total+=path(cost.transpose(0,2,1),sign,0).transpose(0,2,1)
    return total/8

def decode_evidence(volume,bits,raynorm,depths,regularized,*,max_cost=.2,slack=.05,max_interval=.1):
    depths=np.asarray(depths,float);cost=np.asarray(volume,np.float32)
    if len(depths)<3 or not np.isfinite(depths).all() or depths[0]<=0 or not np.all(np.diff(depths)>0):
        raise ValueError('increasing positive depth grid required')
    if cost.shape!=np.shape(regularized) or cost.shape!=np.shape(bits) or cost.shape[0]!=len(depths) or cost.shape[1:]!=np.shape(raynorm):
        raise ValueError('volume/bitmap/raynorm shapes differ')
    best=np.argmin(regularized,axis=0);unary_index=np.argmin(cost,axis=0)
    unary_min=np.take_along_axis(cost,unary_index[None],axis=0)[0]
    chosen=np.take_along_axis(cost,best[None],axis=0)[0]
    regchosen=np.take_along_axis(regularized,best[None],axis=0)[0]
    finite=np.isfinite(chosen)&np.isfinite(regchosen)
    raw=np.where(finite,depths[best],np.nan).astype(np.float32)
    lower=depths.copy();upper=depths.copy();mid=(depths[:-1]+depths[1:])/2;lower[1:]=mid;upper[:-1]=mid
    competitive=np.isfinite(cost)&(cost<=unary_min[None]+slack)
    lo=np.min(np.where(competitive,lower[:,None,None],np.inf),axis=0)
    hi=np.max(np.where(competitive,upper[:,None,None],-np.inf),axis=0)
    interval=np.where(np.isfinite(unary_min),(hi-lo)*raynorm,np.inf)
    supported=finite&(chosen<=unary_min+slack)
    confidence=supported&(best>0)&(best<len(depths)-1)&(chosen<=max_cost)&(interval<=max_interval+1e-8)
    used=np.take_along_axis(bits,best[None],axis=0)[0].copy();used[~finite]=0
    return dict(raw_depth_m=raw,depth_m=np.where(confidence,raw,np.nan).astype(np.float32),confidence=confidence,
        view_bits=used,cost=chosen,interval_ray_width_m=interval.astype(np.float32),winner_index=best.astype(np.int16),
        unary_best_index=unary_index.astype(np.int16),unary_best_cost=unary_min,regularized_cost=regchosen,
        local_competitive_support=supported)

def predict_rgb(reference,sources,*,p1=.05,p2=.20,**options):
    cost,bits,raynorm=build_volume(reference,sources,**options)
    aggregate=aggregate_cost(cost,p1,p2)
    result=decode_evidence(cost,bits,raynorm,options['depths'],aggregate,
        max_cost=options['max_cost'],slack=options['slack'],max_interval=options['max_interval'])
    return result,cost,bits,raynorm
