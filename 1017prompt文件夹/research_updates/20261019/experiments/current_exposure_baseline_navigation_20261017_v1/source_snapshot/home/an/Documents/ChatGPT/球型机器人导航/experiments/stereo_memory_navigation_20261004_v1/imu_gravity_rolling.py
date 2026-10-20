"""ONE fixed gravity-assisted gyro/encoder hypothesis; no orientation truth input."""
import numpy as np
from rolling_odometry import exponential

class GravityRolling:
 def __init__(self,normal=(0,0,1)):
  n=np.asarray(normal,float).copy()
  if n.shape!=(3,) or not np.isfinite(n).all() or np.linalg.norm(n)<=0:raise ValueError('initial gravity calibration required')
  self.normal=n/np.linalg.norm(n);self.R=np.eye(3);self.position=np.zeros(3);self.time=None;self.gyro=None;self.velocity=None
 def update(self,packet):
  if set(packet)!={'time_s','gyro_body_rad_s','linear_acceleration_body_m_s2','main_encoder_rad_s'}:raise ValueError('IMU gyro/acceleration/time/encoder only')
  t=float(packet['time_s']);g=np.asarray(packet['gyro_body_rad_s'],float).copy();acc=np.asarray(packet['linear_acceleration_body_m_s2'],float).copy();enc=float(packet['main_encoder_rad_s'])
  if not np.isfinite(t) or g.shape!=(3,) or acc.shape!=(3,) or not np.isfinite(g).all() or not np.isfinite(acc).all() or not np.isfinite(enc) or (self.time is not None and t<=self.time):raise ValueError('finite increasing sensor sequence required')
  dt=0 if self.time is None else t-self.time;R=self.R.copy();pos=self.position.copy()
  if self.time is not None:pos+=self.velocity*dt;R=R@exponential(self.gyro*dt)
  magnitude=float(np.linalg.norm(acc));used=abs(magnitude-9.81)<=.5
  if used:
   measured=acc/magnitude;expected=R.T@self.normal;correction=np.cross(measured,expected);R=R@exponential(correction*dt) # fixedgain1s^-1
  shellomega=R@(g+np.array([0.,enc,0.]));velocity=.4000000059604645*np.cross(shellomega,self.normal)
  self.time=t;self.R=R;self.position=pos;self.gyro=g;self.velocity=velocity
  return dict(position=pos.copy(),rotation=R.copy(),velocity=velocity.copy(),gravity_used=used,acceleration_norm_m_s2=magnitude)
