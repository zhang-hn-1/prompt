"""Supervision only. No rendered geometry is a network input."""
import numpy as np

def rasterize(world,root,yaw):
 p=np.asarray(world,float).reshape(-1,3);root=np.asarray(root,float)
 if root.shape!=(3,) or not np.isfinite(root).all() or not np.isfinite(yaw):raise ValueError('pose')
 p=p[np.isfinite(p).all(1)];d=p[:,:2]-root[:2];c,s=np.cos(yaw),np.sin(yaw);xy=d@np.array([[c,-s],[s,c]])
 inside=(xy[:,0]>=0)&(xy[:,0]<4)&(xy[:,1]>=-3)&(xy[:,1]<3);xy=xy[inside];z=p[inside,2];idx=np.floor((xy-np.array([0,-3]))/.1+1e-10).astype(int);valid=(idx[:,0]>=0)&(idx[:,0]<40)&(idx[:,1]>=0)&(idx[:,1]<60);idx=idx[valid];z=z[valid];code=np.zeros(len(z),int);code[np.abs(z)<=.03]=1;code[(z>.05)&(z<=.5)]=2;code[(z>.5)&(z<=1)]=3;code[(z>1)&(z<=1.5)]=4
 out=np.zeros((40,60),np.uint8)
 for k in range(1,5):
  counts=np.zeros((40,60),int);j=idx[code==k]
  if len(j):np.add.at(counts,tuple(j.T),1)
  out[counts>=4]=k
 return out

def unproject(depth,T,mask):
 z=np.asarray(depth);y,x=np.where(np.isfinite(z)&(z>=.5)&(z<=3)&~mask);rays=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))],1)
 return T[:3,3]+(rays*z[y,x,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
