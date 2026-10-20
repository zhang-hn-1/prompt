"""Consume observed obstacle positions, preserving unseen space as unverified."""
import numpy as np


def observed_obstacles(points,position):
    points=np.asarray(points,dtype=float).reshape(-1,3)
    valid=np.isfinite(points).all(axis=1)&(points[:,2]>=.25)&(points[:,2]<=1.5)
    valid&=np.linalg.norm(points-np.r_[position,.4],axis=1)>.55
    points=points[valid,:2]
    return np.unique(np.rint(points/.1).astype(int),axis=0)*.1


def frame_coordinates(points,position,goal_world):
    forward=np.asarray(goal_world)-position
    forward/=max(float(np.linalg.norm(forward)),1e-9)
    left=np.array([-forward[1],forward[0]])
    return (np.asarray(points).reshape(-1,2)-position)@np.column_stack((forward,left))


def front_blocked(points,position,goal_world):
    local=frame_coordinates(points,position,goal_world)
    chosen=(local[:,0]>=.55)&(local[:,0]<=3.5)&(abs(local[:,1])<=.6)
    return bool(chosen.sum()>=6)


def make_detour(position,goal_world,side):
    delta=goal_world-position;distance=float(np.linalg.norm(delta))
    if distance<5:return None
    forward=delta/distance;left=np.array([-forward[1],forward[0]])
    x=np.append(np.arange(0,distance,.05),distance)
    smooth=lambda t:np.clip(t,0,1)**2*(3-2*np.clip(t,0,1))
    offset=np.full_like(x,2.3)
    first=x<=3;last=x>=distance-2.6
    offset[first]=2.3*smooth(x[first]/3)
    offset[last]=2.3*(1-smooth((x[last]-(distance-2.6))/2.6))
    return position+x[:,None]*forward+side*offset[:,None]*left


def choose_side(points,position,goal_world):
    points=np.asarray(points).reshape(-1,2);clearances={}
    for side in (-1,1):
        path=make_detour(position,goal_world,side)
        if path is None or not len(points):clearances[side]=0.
        else:
            # Only the next three metres are evaluated against seen surfaces.
            forward=(goal_world-position)/np.linalg.norm(goal_world-position)
            near=path[(path-position)@forward<=3]
            clearances[side]=float(np.min(np.linalg.norm(near[:,None,:]-points[None,:,:],axis=2)))
    side=1 if clearances[1]>clearances[-1]+1e-9 else -1
    return side,clearances
