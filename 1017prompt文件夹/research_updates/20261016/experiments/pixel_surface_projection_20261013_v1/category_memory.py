"""Positive category-obstacle memory only; no reconstructed-height reclassification.

Same recent10s / persistent two-exposure confirmation; no negative/free clearing.
"""
import numpy as np

class CategoryMemory:
    def __init__(self,origin,shape,resolution,viewpoint_required):
        self.origin=np.asarray(origin,float);self.shape=tuple(shape);self.resolution=float(resolution)
        self.recent=np.full(shape,-np.inf);self.confirmed=np.full(shape,-np.inf)
        self.views={};self.frames=set();self.last=-np.inf;self.viewpoint_required=viewpoint_required
    def update(self,world,category,frame,t,view):
        if frame in self.frames or t<self.last or not np.isfinite(t):raise ValueError('causal unique category capture')
        p=np.asarray(world,float).reshape(-1,3);cl=np.asarray(category);view=np.asarray(view,float)
        if cl.shape!=(len(p),) or view.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(view).all():raise ValueError('surface category identity')
        cells=np.floor((p[cl==2,:2]-self.origin)/self.resolution+1e-8).astype(int)
        inside=(cells>=0).all(1)&(cells<self.shape).all(1);cells=np.unique(cells[inside],axis=0)
        for key in map(tuple,cells):
            old=self.views.setdefault(key,[])
            if old and (not self.viewpoint_required or any(np.linalg.norm(view-v)>=self.resolution-1e-8 for v in old)):
                self.confirmed[key]=max(self.confirmed[key],t)
            old.append(view.copy());self.recent[key]=t
        self.frames.add(frame);self.last=t
    def snapshot(self,now):
        if now<self.last:raise ValueError('no future category data')
        recent=np.where(now-self.recent<=10,self.recent,-np.inf);clock=np.maximum(recent,self.confirmed)
        labels=np.full(self.shape,-1,np.int8);labels[np.isfinite(clock)]=1
        return dict(labels=labels,clock=clock,origin=self.origin,resolution=self.resolution,time_s=float(now))
