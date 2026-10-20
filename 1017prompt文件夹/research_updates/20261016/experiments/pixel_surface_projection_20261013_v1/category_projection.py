"""Causal pixel-category/range consumer; no GT or authored scene dependency."""
import numpy as np
from projection import backproject

def raster(world,categories,root,yaw):
    p=np.asarray(world,float).reshape(-1,3);cl=np.asarray(categories,np.uint8)
    if cl.shape!=(len(p),) or not np.isfinite(p).all():raise ValueError('finite points / matching category identities')
    if not np.isin(cl,[0,1,2]).all():raise ValueError('three surface categories')
    c,s=np.cos(yaw),np.sin(yaw);xy=(p[:,:2]-np.asarray(root)[:2])@np.array([[c,-s],[s,c]])
    keep=(xy[:,0]>=0)&(xy[:,0]<4)&(xy[:,1]>=-3)&(xy[:,1]<3)
    idx=np.floor((xy[keep]-[0,-3])/.1+1e-10).astype(int);lab=cl[keep]
    out=np.zeros((40,60),np.uint8)
    for kind in [1,2]:
        counts=np.zeros_like(out,dtype=np.int32);j=idx[lab==kind]
        if len(j):np.add.at(counts,tuple(j.T),1)
        out[counts>=4]=kind
    return out

def point_packet(depth,category,T,root,mask):
    z=np.asarray(depth);lab=np.asarray(category)
    if z.shape!=(480,640) or lab.shape!=z.shape or np.shape(mask)!=z.shape:raise ValueError('native image geometry')
    valid=np.isfinite(z)&(z>=.5)&(z<=3)&np.isin(lab,[1,2])&~np.asarray(mask,bool)
    y,x=np.where(valid);p=backproject(x,y,z[y,x],T)
    outside=np.linalg.norm(p-np.asarray(root),axis=1)>=.55
    return p[outside],lab[y,x][outside],(y*640+x)[outside]
