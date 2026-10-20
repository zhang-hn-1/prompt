"""One pursuit geometry input change; actual perception/root yaw unchanged."""
import math
import numpy as np

def proxy_target(target,position,root_yaw,velocity,accepted_direction):
 target=np.asarray(target,float);position=np.asarray(position,float);velocity=np.asarray(velocity,float);yaw=float(root_yaw)
 if target.shape!=(2,) or position.shape!=(2,) or velocity.shape!=(2,) or not all(np.isfinite(x).all() for x in [target,position,velocity]) or not math.isfinite(yaw) or accepted_direction not in ('forward','reverse'):raise ValueError('finite current geometry and accepted direction required')
 if np.linalg.norm(velocity)<.05:return target.copy(),yaw,False
 heading=math.atan2(velocity[1],velocity[0])-(math.pi if accepted_direction=='reverse' else 0.)
 angle=math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))
 if abs(angle)<=1e-12:return target.copy(),yaw,False
 c,s=math.cos(angle),math.sin(angle);return position+np.array([[c,-s],[s,c]])@(target-position),heading,True

def with_velocity_pursuit(base_class):
 class VelocityPursuit(base_class):
  policy_name='experimental_current_velocity_pursuit_geometry'
  def _begin_tracking_audit(self,observation):
   super()._begin_tracking_audit(observation)
   self._tracking_audit.update(velocity_pursuit_evaluated=False,velocity_pursuit_active=False)
  def act(self,observation,pointgoal_base_xy):
   self._velocity_pursuit_observation=observation
   try:return super().act(observation,pointgoal_base_xy)
   finally:self._velocity_pursuit_observation=None
  def _lookahead_target(self,lookahead_m):
   target=super()._lookahead_target(lookahead_m)
   observation=getattr(self,'_velocity_pursuit_observation',None)
   if observation is None:return target
   proxy,heading,active=proxy_target(target,observation.position_world_xy,observation.yaw_world_rad,observation.proprio_history[-1,:2],self._active_drive_direction)
   self._tracking_audit.update(velocity_pursuit_evaluated=True,velocity_pursuit_active=active,actual_reference_lookahead_world_xy=target.tolist(),pursuit_proxy_world_xy=proxy.tolist(),pursuit_geometry_heading_rad=heading,true_root_heading_rad=float(observation.yaw_world_rad),global_perception_yaw_changed=False,reference_xy_changed=False)
   return proxy
 return VelocityPursuit
