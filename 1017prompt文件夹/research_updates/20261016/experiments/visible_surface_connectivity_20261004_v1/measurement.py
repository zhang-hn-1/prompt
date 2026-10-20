"""Independent geometric measurement. Future frames and actions are never inputs."""
import numpy as np

def upright_calibration(uv,K,width):
    uv=np.asarray(uv,dtype=float);K=np.asarray(K,dtype=float)
    out=np.column_stack((uv[:,1],width-1-uv[:,0]))
    new=np.array([[K[1,1],0,K[1,2]],[0,K[0,0],width-1-K[0,2]],[0,0,1.]])
    R=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    return out,new,R

def restore_depth(depth):
    return np.ascontiguousarray(np.rot90(np.asarray(depth),-1))

def point_errors(pred,true,rays):
    err=np.abs(np.asarray(pred)-np.asarray(true))*np.linalg.norm(rays,axis=-1)
    return np.where(np.isfinite(pred)&(np.asarray(pred)>0)&np.isfinite(err),err,np.inf)

def stats(values):
    a=np.asarray(values,dtype=float);finite=a[np.isfinite(a)]
    return dict(n=len(a),invalid=int((~np.isfinite(a)).sum()),within_0p3m=float(np.mean(a<=.3)) if len(a) else None,
                p50_m=float(np.median(a)) if len(a) and np.isfinite(np.median(a)) else None,
                p90_m=float(np.percentile(a,90)) if len(a) and len(finite)==len(a) else None,
                finite_p90_m=float(np.percentile(finite,90)) if len(finite) else None)

def subgroup_pass(s):
    return s['n']>=100 and s['within_0p3m']>=.9 and s['p90_m'] is not None and s['p90_m']<=.3

def evaluation_masks(depth,points,root,self_mask):
    d=np.asarray(depth);p=np.asarray(points);valid=np.isfinite(d)&(d>=.5)&(d<=3)&~np.asarray(self_mask)
    valid&=np.linalg.norm(p-root,axis=-1)>=.55
    body=valid&(p[:,2]>=.05)&(p[:,2]<=1.5)
    return valid,body
