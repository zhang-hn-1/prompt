"""Actual readonly rigid link readout plus conditional sphere-bottom kinematics."""
import numpy as np

def copy_link_state(time_s,names,poses,velocities):
 names=list(names);t=float(time_s);p=np.asarray(poses,float);v=np.asarray(velocities,float)
 if len(set(names))!=len(names) or not {'base_link','link1'}.issubset(names) or p.shape!=(1,len(names),7) or v.shape!=(1,len(names),6) or not np.isfinite(t) or not np.isfinite(p).all() or not np.isfinite(v).all():raise ValueError('finite complete uniquely named actual link readout required')
 return dict(time_s=t,link_names=names,poses_xyzw=p[0].copy().tolist(),velocities_linear_angular_xyz=v[0].copy().tolist())

def bottom_velocity(linear_center,omega_world,radius):
 v=np.asarray(linear_center,float);w=np.asarray(omega_world,float);r=float(radius)
 if v.shape!=(3,) or w.shape!=(3,) or not np.isfinite(v).all() or not np.isfinite(w).all() or not np.isfinite(r) or r<=0:raise ValueError('finite center velocity/world omega/positive sphere radius required')
 return v+np.cross(w,[0.,0.,-r])
