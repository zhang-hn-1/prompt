"""Float64 RGB/BT/block + threepath reference. Not OpenCV bit equivalence."""
import numpy as np

def fields(rgb):
    a=np.asarray(rgb)
    if a.ndim!=3 or a.shape[2]!=3 or a.dtype!=np.uint8:raise ValueError('HxWx3 uint8 RGB required')
    p=np.pad(a.astype(np.int16),((4,4),(452,452),(0,0)))
    g=np.zeros_like(p)
    dx=2*(p[1:-1,2:]-p[1:-1,:-2])+(p[:-2,2:]-p[:-2,:-2])+(p[2:,2:]-p[2:,:-2])
    g[1:-1,1:-1]=np.clip(dx,-31,31)+31
    f=np.concatenate([g,p],axis=-1);left=np.concatenate([f[:,:1],f[:,:-1]],axis=1);right=np.concatenate([f[:,1:],f[:,-1:]],axis=1)
    halfleft=(f+left)//2;halfright=(f+right)//2
    lo=np.minimum(f,np.minimum(halfleft,halfright));hi=np.maximum(f,np.maximum(halfleft,halfright))
    return f,lo,hi

def build_cost(left,right,disparities):
    a=np.asarray(left);b=np.asarray(right);ds=np.asarray(disparities)
    if a.shape!=b.shape or a.ndim!=3 or a.shape[2]!=3 or min(a.shape[:2])<9:raise ValueError('same RGB shape H,W>=9 required')
    if ds.ndim!=1 or len(ds)<1 or not np.issubdtype(ds.dtype,np.integer) or (abs(ds)>448).any():raise ValueError('integer disparities within +/-448 required')
    H,W=a.shape[:2];af=fields(a);bf=fields(b);cost=np.empty((H,W,len(ds)),np.float64);support=np.zeros(cost.shape,bool)
    aq=[x[2:H+6,450:W+454] for x in af];Y,X=np.indices((H,W));own=(Y>=3)&(Y<H-3)&(X>=4)&(X<W-4)
    for k,d in enumerate(ds.tolist()):
        bp=[x[2:H+6,450-d:W+454-d] for x in bf]
        c0=np.maximum(0,np.maximum(aq[0]-bp[2],bp[1]-aq[0]));c1=np.maximum(0,np.maximum(bp[0]-aq[2],aq[1]-bp[0]));diff=np.minimum(c0,c1)
        pix=diff[:,:,:3].sum(axis=-1,dtype=np.int32)+(diff[:,:,3:]>>2).sum(axis=-1,dtype=np.int32)
        integ=np.pad(pix,((1,0),(1,0))).cumsum(0,dtype=np.int64).cumsum(1,dtype=np.int64)
        cost[:,:,k]=integ[5:,5:]-integ[:-5,5:]-integ[5:,:-5]+integ[:-5,:-5]
        support[:,:,k]=own&(X-d>=4)&(X-d<W-4)
    assert (cost>=0).all() and cost.max()<=9375
    return cost,support

def aggregate(cost,support,keep_paths=False):
    c=np.asarray(cost,dtype=np.float64);m=np.asarray(support)
    if c.ndim!=3 or m.shape!=c.shape or m.dtype!=bool or any(n<1 for n in c.shape):raise ValueError('same HWD float costs/bool support required')
    if np.any(~np.isfinite(c[m])) or np.any(c[m]<0):raise ValueError('supported costs must be finite/nonnegative')
    H,W,D=c.shape;total=np.zeros(c.shape);paths={}
    for name in ['left','right','top']:
        arr=np.full(c.shape,np.inf)
        length=H if name=='top' else W
        order=range(length-1,-1,-1) if name=='right' else range(length)
        batch=W if name=='top' else H;prev=np.full((batch,D),np.inf)
        for i in order:
            cur=c[i,:,:] if name=='top' else c[:,i,:];ok=m[i,:,:] if name=='top' else m[:,i,:]
            low=prev.min(axis=-1);has=np.isfinite(low);lo=np.where(has,low,0.)
            best=prev.copy();best[:,1:]=np.minimum(best[:,1:],prev[:,:-1]+600.);best[:,:-1]=np.minimum(best[:,:-1],prev[:,1:]+600.)
            best=np.minimum(best,lo[:,None]+2400.)-lo[:,None];best[~has,:]=0.
            v=np.where(ok,cur+best,np.inf)
            if name=='top':arr[i,:,:]=v
            else:arr[:,i,:]=v
            prev=v
        total+=arr
        if keep_paths:paths[name]=arr
        else:del arr
    return total,paths

def decode(total,disparities):
    a=np.asarray(total);ds=np.asarray(disparities)
    if a.ndim!=3 or len(ds)!=a.shape[-1] or np.isnan(a).any():raise ValueError('HWD costs and D disparities required')
    H,W,D=a.shape;f=a.reshape(-1,D);idx=f.argmin(axis=1);best=f[np.arange(len(f)),idx]
    count=(np.isfinite(f)&(f==best[:,None])).sum(axis=1);alt=f.copy()
    for offset in [-1,0,1]:
        k=idx+offset;ok=(k>=0)&(k<D);alt[np.flatnonzero(ok),k[ok]]=np.inf
    second=alt.min(axis=1);valid=np.isfinite(best)&(count==1)&(second*.9>best)
    return np.where(valid,ds[idx],np.nan).reshape(H,W).astype(np.float32)
