"""Single experimental change: forward pursuit uses measured motion heading."""
import math
import numpy as np

def curvature_from_motion(delta_world,velocity_world,body_yaw,direction,original):
 v=np.asarray(velocity_world,dtype=float);d=np.asarray(delta_world,dtype=float)
 if direction!='forward' or np.linalg.norm(v)<.08 or float(v@np.array([math.cos(body_yaw),math.sin(body_yaw)]))<=0:
  return float(original),False
 course=math.atan2(v[1],v[0]);den=float(d@d)
 return (0. if den<=1e-9 else 2*(-math.sin(course)*d[0]+math.cos(course)*d[1])/den),True

def with_course_geometry(base):
 class CourseOracle(base):
  def _compose_tracking_curvature(self,observation,*,waypoint,geometric_curvature_1_m):
   audit=self._tracking_audit
   target=np.array([audit['lookahead_x_m'],audit['lookahead_y_m']]) if 'lookahead_x_m' in audit else self._reference_xy[audit['lookahead_index']]
   k,active=curvature_from_motion(target-observation.position_world_xy,observation.proprio_history[-1,:2],observation.yaw_world_rad,waypoint.drive_direction,geometric_curvature_1_m)
   return k,dict(candidate_active=active,curvature_composition='measured_course_forward' if active else 'pure_pursuit_only',original_body_pursuit_curvature_1_m=float(geometric_curvature_1_m),course_pursuit_curvature_1_m=k,measured_course_rad=math.atan2(observation.proprio_history[-1,1],observation.proprio_history[-1,0]) if active else None)
 return CourseOracle
