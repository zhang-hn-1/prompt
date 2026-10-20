"""Float64 observed-support min-plus reference. NOT OpenCV/SGBM reproduction."""
import numpy as np

def aggregate(cost,support,P1=600.,P2=2400.):
    c=np.asarray(cost,dtype=np.float64); m=np.asarray(support)
    if c.ndim!=3 or any(n<1 for n in c.shape) or m.shape!=c.shape or m.dtype!=bool:
        raise ValueError('nonempty HWD cost and same-shape bool support required')
    if not (np.isfinite(P1) and np.isfinite(P2) and 0<=P1<=P2):raise ValueError('finite 0<=P1<=P2 required')
    if np.any(~np.isfinite(c[m])) or np.any(c[m]<0):raise ValueError('supported costs finite/nonnegative required')
    H,W,D=c.shape; paths={}
    for name,dy,dx in [('left',0,1),('right',0,-1),('top',1,0)]:
        arr=np.full(c.shape,np.inf,dtype=np.float64)
        ys=range(H);xs=range(W-1,-1,-1) if dx<0 else range(W)
        for y in ys:
            for x in xs:
                ok=m[y,x]
                if not ok.any():continue
                py,px=y-dy,x-dx
                prev=arr[py,px] if 0<=py<H and 0<=px<W else np.full(D,np.inf)
                finite=np.isfinite(prev)
                if not finite.any():arr[y,x,ok]=c[y,x,ok];continue
                lo=float(prev[finite].min())
                same=prev.copy();near=np.full(D,np.inf)
                near[1:]=prev[:-1]+P1;near[:-1]=np.minimum(near[:-1],prev[1:]+P1)
                trans=np.minimum(np.minimum(same,near),lo+P2)-lo
                arr[y,x,ok]=c[y,x,ok]+trans[ok]
        paths[name]=arr
    total=paths['left']+paths['right']+paths['top']
    return total,paths

def decode(total):
    a=np.asarray(total,dtype=np.float64)
    if a.ndim!=3 or a.shape[-1]<1 or np.isnan(a).any():raise ValueError('HWD aggregate without NaN required')
    minimum=a.min(axis=-1);unique=(np.isfinite(a)&(a==minimum[...,None])).sum(axis=-1)==1
    return np.where(np.isfinite(minimum)&unique,a.argmin(axis=-1),-1).astype(np.int32)
