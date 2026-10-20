"""Offline finite-window XY circle descriptor; never a prediction/controller."""
import numpy as np

def measure_turn(xy,yaw):
 p=np.asarray(xy,float);y=np.asarray(yaw,float)
 if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or y.shape!=(len(p),) or not np.isfinite(p).all() or not np.isfinite(y).all():raise ValueError('finite matched XY/yaw required')
 arc=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum());dy=float(np.unwrap(y)[-1]-np.unwrap(y)[0]);mid=len(p)//2
 a=p[mid]-p[0];b=p[-1]-p[mid];c=p[-1]-p[0];la,lb,lc=(float(np.linalg.norm(x)) for x in (a,b,c))
 available=arc>=.05 and min(la,lb)>=.01 and lc>=.01
 curvature=float(2*(a[0]*b[1]-a[1]*b[0])/(la*lb*lc)) if available else None
 return dict(geometric_available=bool(available),xy_curvature_1_m=curvature,body_yaw_change_rad=dy,body_yaw_per_xy_arc_1_m=dy/arc if available else None,actual_xy_arc_m=arc,half_chords_m=[la,lb],whole_chord_m=lc,unavailable_reason=None if available else 'insufficient_actual_translation')
