"""Canonical bridge carrier around already frozen native GravityRolling; no new model."""
import numpy as np
from imu_gravity_rolling import GravityRolling
class NativeRollingShadow:
 def __init__(self):self.model=None;self.initial_normal=None;self.records=[];self.sensors=[]
 def consume(self,sensor,initial_normal=None):
  if set(sensor)!={'time_s','gyro_body_rad_s','linear_acceleration_body_m_s2','main_encoder_rad_s'}:raise ValueError('native gyro/acceleration/time/encoder whitelist required')
  if self.model is None:
   if initial_normal is None:raise ValueError('oneinitial gravity calibration required')
   self.model=GravityRolling(initial_normal);self.initial_normal=self.model.normal.copy()
  result=self.model.update(sensor);R=result['rotation'];shell=R@(np.asarray(sensor['gyro_body_rad_s'])+np.array([0.,float(sensor['main_encoder_rad_s']),0.]))
  self.records.append(dict(time_s=float(sensor['time_s']),position=result['position'].tolist(),rotation=R.tolist(),velocity=result['velocity'].tolist(),shell_omega_gauge=shell.tolist()))
  self.sensors.append({k:np.asarray(v).tolist() for k,v in sensor.items()});return None
