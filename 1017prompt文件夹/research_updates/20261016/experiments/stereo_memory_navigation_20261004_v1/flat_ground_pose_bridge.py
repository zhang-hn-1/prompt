"""Independent height prior for the authorized flat-ground task, never observed."""
import math
from rolling_pose_bridge import RollingPoseBridge

class FlatGroundPoseBridge(RollingPoseBridge):
 def __init__(self,initial_world_position,initial_world_rotation,*,floor_z_m=0.,radius_m=.4000000059604645):
  floor=float(floor_z_m);radius=float(radius_m)
  if not math.isfinite(floor) or not math.isfinite(radius) or radius<=0:raise ValueError('finite floor and positive CAD radius required')
  self.floor_z_m=floor;self.radius_m=radius
  super().__init__(initial_world_position,initial_world_rotation)
 def at(self,time_s):
  out=super().at(time_s);p=out['root_position_world_m'];old_z=float(p[2]);new_z=self.floor_z_m+self.radius_m;delta=new_z-old_z;p[2]=new_z;out['velocity_world_m_s'][2]=0.
  for eye in ('left','right'):out[eye+'_camera_usd_world'][2,3]+=delta
  out.update(flat_ground_height_prior=True,height_prior_is_observation=False,full_3d_certified=False,floor_height_prior_m=self.floor_z_m,root_center_height_prior_m=new_z,unconstrained_root_world_z_m=old_z)
  return out
