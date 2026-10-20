"""Callable historical domain-error scaffold for literal RED evidence."""
import numpy as np

def measure_turn(xy,yaw):
 p=np.asarray(xy,float);y=np.asarray(yaw,float)
 if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or y.shape!=(len(p),) or not np.isfinite(p).all() or not np.isfinite(y).all():raise ValueError('finite matched XY/yaw required')
 arc=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum());dy=float(np.unwrap(y)[-1]-np.unwrap(y)[0])
 return dict(geometric_available=True,xy_curvature_1_m=dy/arc if arc else 0,body_yaw_change_rad=dy,body_yaw_per_xy_arc_1_m=dy/arc if arc else None)
