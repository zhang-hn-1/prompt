"""Declared diagnostic endpoints, no map/pose/model/parameter search."""
import numpy as np

def short_obstacle_endpoints(start,goal):
 a=np.asarray(start,float);b=np.asarray(goal,float)
 if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('finite3D endpoints required')
 delta=b[:2]-a[:2];distance=np.linalg.norm(delta)
 if distance<=4.:raise ValueError('symmetric2m shortening requires greaterthan4m original span')
 u=delta/distance;x=a.copy();y=b.copy();x[:2]+=2*u;y[:2]-=2*u
 return x,y
