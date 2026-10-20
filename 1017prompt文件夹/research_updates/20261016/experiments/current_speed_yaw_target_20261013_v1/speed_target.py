import math,numpy as np

def feedback(main,speed,curvature,measured,gain):
 if not all(math.isfinite(float(x)) for x in [main,speed,curvature,measured,gain]) or main==0 or speed<0 or gain<0:raise ValueError('finite current tracking input')
 sign=1. if main>0 else -1.;desired=curvature*sign*speed;correction=float(np.clip(-sign*gain*(desired-measured),math.radians(-5),math.radians(5)));return desired,correction

def with_current_speed_target(base):
 class CurrentSpeedTarget(base):
  def _request_action(self,main_velocity,auxiliary_angle,*,source):
   self.current_speed_last=None
   a=self._tracking_audit
   if source=='tracking' and a.get('valid',False):
    speed=float(a['world_xy_speed_m_s']);curvature=float(self._desired_curvature_1_m);measured=float(a['measured_world_yaw_rate_rad_s']);gain=float(self.yaw_rate_feedback_gain_s);old=float(a['yaw_rate_feedback_auxiliary_rad']);desired,new=feedback(main_velocity,speed,curvature,measured,gain)
    self.current_speed_last=dict(main_request_rad_s=float(main_velocity),curvature_1_m=curvature,gain_s=gain,upstream_curvature_proxy_legacy=True,measured_speed_m_s=speed,legacy_desired_yaw_rate_rad_s=float(a['desired_world_yaw_rate_rad_s']),desired_yaw_rate_rad_s=desired,measured_yaw_rate_unchanged=measured,old_correction_rad=old,new_correction_rad=new,intervened=bool(new!=old),source='current_native_speed_same_original_yaw_measurement',no_speedfloor=True)
    a['desired_world_yaw_rate_rad_s']=desired;a['yaw_rate_feedback_auxiliary_rad']=new;auxiliary_angle=float(auxiliary_angle)+new-old
   return super()._request_action(main_velocity,auxiliary_angle,source=source)
 return CurrentSpeedTarget
