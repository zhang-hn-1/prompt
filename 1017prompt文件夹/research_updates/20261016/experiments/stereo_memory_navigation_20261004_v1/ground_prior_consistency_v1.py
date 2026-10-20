"""Declared plane prior sign annotation only; no filtering, action or certificate."""
import numpy as np

def annotate_ground_prior(packet,now,plane_world):
 t=float(packet['time_s']);n=float(now)
 if not np.isfinite([t,n]).all() or t>n:raise ValueError('past finite acquisition before geometry read')
 plane=np.asarray(plane_world,dtype=float)
 if plane.shape!=(4,) or not np.isfinite(plane).all() or np.linalg.norm(plane[:3])==0:raise ValueError('finite nondegenerate plane')
 w=np.asarray(packet['world'],dtype=float)
 if w.ndim!=2 or w.shape[1]!=3 or not np.isfinite(w).all():raise ValueError('finite provisional points')
 signed=(w@plane[:3]+plane[3])/np.linalg.norm(plane[:3])
 return dict(world=w.copy(),acquisition_time_s=t,ground_plane_world=plane.copy(),signed_plane_distance_m=signed,prior_inconsistent=signed<0,ground_is_prior=True,source_admitted=False,measured_geometry_admitted=False,certified_free=False,all_points_retained=True,pose_uncertainty_not_certified=True,no_action_API=True)
