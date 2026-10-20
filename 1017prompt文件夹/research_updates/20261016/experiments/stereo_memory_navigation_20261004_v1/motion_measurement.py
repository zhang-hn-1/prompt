"""Offline aligned physical geometry; never supplies future feedback to control."""
import numpy as np

def xy_curvature(time,position,velocity):
    t=np.asarray(time,float);p=np.asarray(position,float);v=np.asarray(velocity,float)
    if p.shape!=v.shape or v.shape!=(len(t),2) or len(t)<3 or not np.all(np.diff(t)>0):raise ValueError('aligned monotonic whole sequence required')
    speed=np.linalg.norm(v,axis=1);a=np.gradient(v,t,axis=0,edge_order=2)
    good=np.isfinite(p).all(1)&np.isfinite(v).all(1)&np.isfinite(a).all(1)&(speed>=.05)
    good[1:-1]&=good[:-2].copy()&good[2:].copy();good[[0,-1]]=False
    k=np.full(len(t),np.nan);k[good]=(v[good,0]*a[good,1]-v[good,1]*a[good,0])/speed[good]**3
    course=np.full(len(t),np.nan)
    indices=np.flatnonzero(good)
    for group in np.split(indices,np.flatnonzero(np.diff(indices)!=1)+1):
        if len(group):course[group]=np.unwrap(np.arctan2(v[group,1],v[group,0]))
    return k,course,good

def spatial_curvature(position):
    p=np.asarray(position,float);a=p[1:-1]-p[:-2];b=p[2:]-p[1:-1];c=p[2:]-p[:-2]
    la=np.linalg.norm(a,axis=1);lb=np.linalg.norm(b,axis=1);lc=np.linalg.norm(c,axis=1)
    valid=(la>=.002)&(lb>=.002)&(lc>1e-10)&np.isfinite(a).all(1)&np.isfinite(b).all(1)
    k=np.full(len(p),np.nan);middle=k[1:-1]
    middle[valid]=2*(a[valid,0]*b[valid,1]-a[valid,1]*b[valid,0])/(la[valid]*lb[valid]*lc[valid])
    return k

def qualification(speed):
    speed=np.asarray(speed,float);counts=np.zeros(len(speed),int);count=0
    for i,s in enumerate(speed):
        count=count+1 if np.isfinite(s) and s<.05 and s>=0 else 0
        counts[i]=count
    return counts>=5,counts
