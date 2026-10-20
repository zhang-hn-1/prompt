"""Bounded-risk 2.5D exploration diagnostic. Unknown is never observed free.

This candidate deliberately permits motion through unknown support in simulation,
with a .3rad/s main limit. It is NOT certified safe and is not RGB navigation.
"""
import sys
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
P='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm'
sys.path.insert(0,P)
OLD=Path(__file__).resolve().parent.parent/'visible_surface_connectivity_20261004_v1'
sys.path.insert(0,str(OLD))
from local_map import BeliefGrid
from benchmark.grid_planning import astar_path

class ExplorationGrid(BeliefGrid):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.exploration_unknown_fraction=1.
    def plan(self,xy,goal):
        start=self.cells(np.asarray(xy)[None])[0];end=self.cells(np.asarray(goal)[None])[0]
        if not self.inside(start[None])[0] or not self.inside(end[None])[0]:return None,'outside_fixed_grid'
        clearance=distance_transform_edt(np.pad(self.labels!=1,1))[1:-1,1:-1]*self.resolution
        traversable=clearance>=.6-1e-8
        if not traversable[tuple(start)] or not traversable[tuple(end)]:return None,'observed_obstacle_blocks_route'
        try:path=astar_path((~traversable).astype(np.uint8),start=tuple(start),goal=tuple(end))
        except ValueError:return None,'observed_obstacle_blocks_route'
        world=self.origin+path*self.resolution
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(world,axis=0),axis=1))]
        near=path[arc<=1.+1e-8]
        offsets=np.array([(i,j) for i in range(-6,7) for j in range(-6,7) if np.hypot(i,j)*self.resolution<=.6+1e-8])
        sampled=(near[:,None,:]+offsets[None,:,:]).reshape(-1,2)
        inside=self.inside(sampled)
        unknown=~inside
        unknown[inside]=self.labels[sampled[inside,0],sampled[inside,1]]<0
        self.exploration_unknown_fraction=float(unknown.mean())
        return world,'unknown_exploration' if unknown.any() else 'observed_route'

def limit_action(action,unknown_fraction):
    action=np.asarray(action,dtype=np.float32).copy()
    if unknown_fraction>0:action[::2]=np.clip(action[::2],-.3,.3)
    return action
