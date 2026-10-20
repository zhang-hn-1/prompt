"""One event-latched route clearance change; current/past observed hazards only.

The unchanged stop radius is an uncertified heuristic, not a physical guarantee.
Before the event the original planner is called and its path returned unchanged.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt
from provenance_grid import ProvenanceGrid
from route_geometry_audit import dense_prefix, hazard_clearance
from benchmark.grid_planning import astar_path
STOP_RADIUS_M = float(.5+.14088407559444638+2**.5*.1/2)

class RouteClearanceGrid(ProvenanceGrid):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.clearance_active=False
        self.clearance_event_time_s=None
        self.clearance_record={}

    def plan(self,xy,goal):
        xy=np.asarray(xy,float);goal=np.asarray(goal,float)
        hazard=self.origin+np.argwhere(self.labels==1)*self.resolution
        current=hazard_clearance([xy],hazard)
        entered=False;local=None
        if not self.clearance_active:
            old_path,old_reason=super().plan(xy,goal)
            if old_path is not None:
                near=int(np.linalg.norm(old_path-xy,axis=1).argmin())
                prefix=dense_prefix(old_path[near:],1.)
                if not np.array_equal(prefix[0],xy):prefix=np.vstack((xy,prefix))
                local=hazard_clearance(prefix,hazard)
                entered=bool(current is not None and current>STOP_RADIUS_M and local is not None and local<STOP_RADIUS_M)
            if entered:
                self.clearance_active=True;self.clearance_event_time_s=float(self.time)
            else:
                self.clearance_record=dict(active=False,entered=False,event_time_s=None,
                    current_hazard_clearance_m=current,old_local_path_clearance_m=local,
                    required_point_clearance_m=.6,certified_stopping_bound=False)
                return old_path,old_reason
        self.clearance_record=dict(active=True,entered=entered,event_time_s=self.clearance_event_time_s,
            current_hazard_clearance_m=current,old_local_path_clearance_m=local,
            required_point_clearance_m=STOP_RADIUS_M,certified_stopping_bound=False)
        start=self.cells(xy[None])[0];end=self.cells(goal[None])[0]
        if not self.inside(start[None])[0] or not self.inside(end[None])[0]:return None,'outside_fixed_grid'
        clearance=distance_transform_edt(np.pad(self.labels!=1,1))[1:-1,1:-1]*self.resolution
        self.clearance_record.update(start_grid_clearance_m=float(clearance[tuple(start)]),goal_grid_clearance_m=float(clearance[tuple(end)]))
        traversable=clearance>=STOP_RADIUS_M-1e-8
        if not traversable[tuple(start)] or not traversable[tuple(end)]:return None,'observed_obstacle_blocks_route'
        try:path=astar_path((~traversable).astype(np.uint8),start=tuple(start),goal=tuple(end))
        except ValueError:return None,'observed_obstacle_blocks_route'
        world=self.origin+path*self.resolution
        # Keep the original unknown-risk accounting and .6m offsets unchanged.
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(world,axis=0),axis=1))]
        near=path[arc<=1.+1e-8]
        offsets=np.array([(i,j) for i in range(-6,7) for j in range(-6,7) if np.hypot(i,j)*self.resolution<=.6+1e-8])
        sampled=(near[:,None,:]+offsets[None,:,:]).reshape(-1,2)
        inside=self.inside(sampled);unknown=~inside
        unknown[inside]=self.labels[sampled[inside,0],sampled[inside,1]]<0
        self.exploration_unknown_fraction=float(unknown.mean())
        return world,'unknown_exploration' if unknown.any() else 'observed_route'
