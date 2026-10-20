"""Causal observed-space map. Unknown cells are blocked, never assumed clear."""
import numpy as np
from scipy.ndimage import distance_transform_edt,label
from benchmark.grid_planning import astar_path

class BeliefGrid:
    def __init__(self,origin,shape=(400,400),resolution=.1):
        self.origin=np.asarray(origin,dtype=float);self.resolution=float(resolution)
        self.labels=np.full(shape,-1,dtype=np.int8);self.last_seen=np.full(shape,-np.inf);self.time=-np.inf
    def cells(self,points):return np.rint((np.asarray(points)[...,:2]-self.origin)/self.resolution).astype(int)
    def inside(self,c):return (c[:,0]>=0)&(c[:,0]<self.labels.shape[0])&(c[:,1]>=0)&(c[:,1]<self.labels.shape[1])
    def seed_body(self,xy):
        c=self.cells(np.asarray(xy)[None])[0];r=int(np.ceil(.4/self.resolution))
        x,y=np.mgrid[-r:r+1,-r:r+1];near=np.column_stack((x.ravel(),y.ravel()))
        near=near[np.linalg.norm(near,axis=1)*self.resolution<=.4+1e-8]+c
        near=near[self.inside(near)];good=self.labels[near[:,0],near[:,1]]!=1
        self.labels[near[good,0],near[good,1]]=0
    def update_points(self,points,root,time_s,ground_meshes=()):
        if time_s<self.time:raise ValueError('past-only monotonic updates required')
        self.time=float(time_s);p=np.asarray(points).reshape(-1,3);root=np.asarray(root)
        p=p[np.isfinite(p).all(1)&(np.linalg.norm(p-root,axis=1)>=.55)]
        floor=p[(p[:,2]>=-.05)&(p[:,2]<=.05)]
        body=p[(p[:,2]>.05)&(p[:,2]<=1.5)]
        # Only actual projected floor hit cells are marked free. Do not ray-clear unseen height.
        for values,kind in [(floor,0),(body,1)]:
            if not len(values):continue
            c=self.cells(values);c=c[self.inside(c)]
            if kind==0:c=c[self.labels[c[:,0],c[:,1]]!=1]
            self.labels[c[:,0],c[:,1]]=kind;self.last_seen[c[:,0],c[:,1]]=time_s
        for mesh in ground_meshes:
            self._raster_observed_floor(mesh,root,time_s)
    def _raster_observed_floor(self,mesh,root,time_s):
        """Only interpolate adjacent, short, valid ground-hit triangles.

        This inherits the baseline's flat-floor 2.5D representation. It is
        not a certificate for unobserved vertical volumes or near-field.
        """
        m=np.asarray(mesh,dtype=float)
        if m.ndim!=3 or m.shape[-1]!=3:raise ValueError('organized HxWx3 hits required')
        a,b,c,d=m[:-1,:-1],m[:-1,1:],m[1:,:-1],m[1:,1:]
        tri=np.concatenate((np.stack((a,b,c),axis=-2).reshape(-1,3,3),
                            np.stack((b,d,c),axis=-2).reshape(-1,3,3)))
        good=np.isfinite(tri).all((1,2))
        good&=((tri[:,:,2]>=-.05)&(tri[:,:,2]<=.05)).all(1)
        good&=(np.linalg.norm(tri-root,axis=-1)>=.55).all(1)
        edges=np.stack((tri[:,1]-tri[:,0],tri[:,2]-tri[:,1],tri[:,0]-tri[:,2]),axis=1)
        good&=(np.linalg.norm(edges,axis=-1)<=.25+1e-10).all(1)
        tri=tri[good,:,:2]
        if not len(tri):return
        q=(tri-self.origin)/self.resolution
        area=np.cross(q[:,1]-q[:,0],q[:,2]-q[:,0])
        q=q[np.abs(area)>1e-10]
        if not len(q):return
        low=np.ceil(q.min(1)-1e-10).astype(int);high=np.floor(q.max(1)+1e-10).astype(int)
        offsets=np.array([(i,j) for i in range(4) for j in range(4)])
        points=low[:,None,:]+offsets[None,:,:]
        valid=(points<=high[:,None,:]).all(-1)
        valid&=(points>=0).all(-1)&(points<np.asarray(self.labels.shape)).all(-1)
        cross=np.stack([np.cross(q[:,(i+1)%3,None,:]-q[:,i,None,:],points-q[:,i,None,:]) for i in range(3)],axis=-1)
        valid&=(cross>=-1e-9).all(-1)|(cross<=1e-9).all(-1)
        cells=points[valid]
        if len(cells):
            cells=cells[self.labels[cells[:,0],cells[:,1]]!=1]
            self.labels[cells[:,0],cells[:,1]]=0;self.last_seen[cells[:,0],cells[:,1]]=time_s
    def plan(self,xy,goal):
        start=self.cells(np.asarray(xy)[None])[0];end=self.cells(np.asarray(goal)[None])[0]
        if not self.inside(start[None])[0]:return None,'outside_fixed_grid'
        free=(self.labels==0)
        clearance=distance_transform_edt(np.pad(free,1))[1:-1,1:-1]*self.resolution
        safe=free&(clearance>=.6-1e-8)
        safe[tuple(start)]=True
        connected,n=label(safe,np.ones((3,3),int));reachable=connected==connected[tuple(start)]
        choices=np.argwhere(reachable)
        world=self.origin+choices*self.resolution
        moved=np.linalg.norm(world-xy,axis=1)>=.3
        if not np.any(moved):return None,'no_known_free_route'
        choices=choices[moved];world=world[moved]
        idx=np.argmin(np.linalg.norm(world-goal,axis=1));target=tuple(choices[idx])
        try:p=astar_path((~safe).astype('uint8'),start=tuple(start),goal=target)
        except ValueError:return None,'no_known_free_route'
        return self.origin+p*self.resolution,'goal' if np.linalg.norm(self.origin+np.asarray(target)*self.resolution-goal)<.2 else 'observed_frontier'

class DirectionLatch:
    def __init__(self):self.direction=0;self.low=0
    def request(self,desired,speed):
        if self.direction==0:self.direction=int(desired);return self.direction
        if desired==self.direction:self.low=0;return self.direction
        self.low=self.low+1 if speed<.05 else 0
        if self.low>=5:self.direction=int(desired);self.low=0;return self.direction
        return 0
