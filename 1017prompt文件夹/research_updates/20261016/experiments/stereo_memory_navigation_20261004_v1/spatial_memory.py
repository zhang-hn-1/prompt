"""Provenance-preserving gravity-aligned floor/hazard memory.

This 2.5D diagnostic does NOT certify full swept 3D collision volume. Never
observed cells remain unknown. Historical floor is evidence with bounded age,
not model prediction or an assumption of flat terrain in unseen space.
"""
import numpy as np

class SpatialMemory:
    def __init__(self,origin,size,resolution,max_age_s=10.):
        self.origin=np.asarray(origin,float);self.resolution=float(resolution)
        self.floor_time=np.full((size,size),-np.inf)
        self.hazard_time=np.full((size,size),-np.inf)
        self.max_age_s=float(max_age_s);self.time=-np.inf
    def cells(self,points):
        return np.rint((np.asarray(points)[...,:2]-self.origin)/self.resolution).astype(int)
    def update(self,points,time_s,pose_sigma_m):
        if not np.isfinite(time_s) or time_s<self.time:raise ValueError('noncausal timestamp')
        self.time=float(time_s)
        p=np.asarray(points,float).reshape(-1,3)
        p=p[np.isfinite(p).all(axis=1)]
        if not len(p):return
        cells=self.cells(p);shape=np.asarray(self.floor_time.shape)
        inside=((cells>=0)&(cells<shape)).all(axis=1);p=p[inside];cells=cells[inside]
        ground=(np.abs(p[:,2])<=.05)
        hazard=(p[:,2]>.05)&(p[:,2]<=1.5)
        # Uncertain observations cannot certify floor. Retain conservative hazards.
        if np.isfinite(pose_sigma_m) and pose_sigma_m<=.5*self.resolution:
            self.floor_time[cells[ground,0],cells[ground,1]]=time_s
        self.hazard_time[cells[hazard,0],cells[hazard,1]]=time_s
    def status(self,xy,time_s):
        if time_s<self.time:raise ValueError('past query would read future evidence')
        i,j=self.cells(xy)
        if min(i,j)<0 or i>=self.floor_time.shape[0] or j>=self.floor_time.shape[1]:return 'unknown'
        if np.isfinite(self.hazard_time[i,j]):return 'occupied'
        t=self.floor_time[i,j]
        if not np.isfinite(t) or time_s-t>self.max_age_s:return 'unknown'
        return 'current_floor' if abs(time_s-t)<1e-6 else 'historical_floor'
    def labels(self,time_s,current_only=False):
        if time_s<self.time:raise ValueError('past query would read future evidence')
        a=np.full(self.floor_time.shape,-1,np.int8)
        fresh=np.isfinite(self.floor_time)&(time_s-self.floor_time<=self.max_age_s)
        if current_only:fresh&=np.abs(time_s-self.floor_time)<1e-6
        a[fresh]=0;a[np.isfinite(self.hazard_time)]=1
        return a
