"""Read-only current local endpoint qualification; never an action consumer."""
import math

class LocalTerminalCapture:
 def __init__(self):
  self.last_time=None
  self.version=None
  self.count=0
 def observe(self,time_s,current_xy,current_velocity_xy,endpoint_xy,reference_version):
  if isinstance(time_s,bool) or not math.isfinite(float(time_s)):
   raise ValueError('finite time required')
  t=float(time_s)
  if self.last_time is not None and t<=self.last_time:
   raise ValueError('strictly increasing sample clock required')
  if isinstance(reference_version,bool) or not isinstance(reference_version,int):
   raise ValueError('integer reference version required')
  vectors=[]
  for value in (current_xy,current_velocity_xy,endpoint_xy):
   if len(value)!=2 or any(isinstance(v,bool) or not math.isfinite(float(v)) for v in value):
    raise ValueError('finite XY vectors required')
   vectors.append([float(v) for v in value])
  xy,velocity,endpoint=vectors
  distance=math.hypot(xy[0]-endpoint[0],xy[1]-endpoint[1])
  speed=math.hypot(*velocity)
  reset=(self.version!=reference_version or self.last_time is None or abs(t-self.last_time-.1)>1e-5)
  if reset:self.count=0
  self.count=self.count+1 if distance<=.2 and speed<.05 else 0
  self.last_time=t
  self.version=reference_version
  return dict(time_s=t,reference_version=reference_version,distance_m=distance,speed_m_s=speed,count=self.count,qualified_now=self.count>=5,scope='read_only_local_endpoint_current_state')
