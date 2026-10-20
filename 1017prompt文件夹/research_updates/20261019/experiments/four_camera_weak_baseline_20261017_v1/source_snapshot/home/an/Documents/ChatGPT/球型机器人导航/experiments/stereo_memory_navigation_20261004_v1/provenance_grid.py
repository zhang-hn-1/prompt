"""Online 2.5D observed provenance, with unchanged risk A*/clearance consumer.

This arm deliberately permits unknown traversal at the old .3rad/s request cap.
It reads privileged current depth/pose; no full-map or observed-free body prior.
"""
import numpy as np
from risk_grid import ExplorationGrid
from spatial_memory import SpatialMemory


class ProvenanceGrid(ExplorationGrid):
    def __init__(self,origin,shape=(400,400),resolution=.1,memory_mode='history'):
        if memory_mode not in ('current','history'):raise ValueError(memory_mode)
        if len(shape)!=2 or shape[0]!=shape[1]:raise ValueError('square grid required')
        super().__init__(origin,shape=shape,resolution=resolution)
        self.memory_mode=memory_mode
        self.memory=SpatialMemory(origin,shape[0],resolution,max_age_s=10.)
        self.footprint_mask=np.zeros(shape,bool)
    def update_points(self,points,root,time_s,ground_meshes=()):
        if ground_meshes:raise ValueError('point provenance arm excludes mesh interpolation')
        if not np.isfinite(time_s) or time_s<self.time:raise ValueError('noncausal timestamp')
        points=np.asarray(points,float).reshape(-1,3);root=np.asarray(root,float)
        points=points[np.isfinite(points).all(1)&(np.linalg.norm(points-root,axis=1)>=.55)]
        if self.memory_mode=='current':
            self.memory=SpatialMemory(self.origin,self.labels.shape[0],self.resolution,max_age_s=10.)
        # Only this privileged depth+true-pose diagnostic assumes sigma=0.
        self.memory.update(points,float(time_s),pose_sigma_m=0.)
        self.time=float(time_s);self.labels=self.memory.labels(time_s)
        self.last_seen=np.maximum(self.memory.floor_time,self.memory.hazard_time)
    def seed_body(self,xy):
        # Geometry-only footprint diagnostic: never feeds floor/obstacle labels.
        self.footprint_mask[:]=False
        center=self.cells(np.asarray(xy)[None])[0]
        radius=int(np.ceil(.4/self.resolution))
        offsets=np.array([(i,j) for i in range(-radius,radius+1) for j in range(-radius,radius+1) if np.hypot(i,j)*self.resolution<=.4+1e-8])
        cells=offsets+center;cells=cells[self.inside(cells)]
        self.footprint_mask[cells[:,0],cells[:,1]]=True
    def provenance(self):
        # 0unknown,1currentfloor,2historyfloor,3currenthazard,4historyhazard
        result=np.zeros(self.labels.shape,np.uint8)
        floor=self.labels==0;hazard=self.labels==1
        result[floor]=np.where(abs(self.memory.floor_time[floor]-self.time)<1e-6,1,2)
        result[hazard]=np.where(abs(self.memory.hazard_time[hazard]-self.time)<1e-6,3,4)
        return result
    def diagnostics(self):
        codes=self.provenance()
        return dict(memory_mode=self.memory_mode,current_floor=int((codes==1).sum()),historical_floor=int((codes==2).sum()),current_occupied=int((codes==3).sum()),historical_occupied=int((codes==4).sum()),unknown=int((codes==0).sum()),footprint_unknown=int((self.footprint_mask&(self.labels==-1)).sum()),body_prior_observed_free=0,pose_sigma_assumption_m=0.,full_3d_certified=False)
