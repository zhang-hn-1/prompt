"""Navigation sees estimated kinematics + measured joints/actions/RGB only."""
import numpy as np
from scipy.spatial.transform import Rotation
from rolling_pose_bridge import vector,rigid_rotation

class EstimatedNavigationObservation:
 def __init__(self):self.times=[];self.proprio=[];self.quaternions=[]
 def build(self,original,pose,gyro_body,world_goal):
  t=float(pose['time_s']);p=vector(pose['root_position_world_m']);R=rigid_rotation(pose['root_rotation_world']);v=vector(pose['velocity_world_m_s']);gyro=vector(gyro_body);target=vector(world_goal)
  if not np.isfinite(t) or (self.times and t<=self.times[-1]):raise ValueError('strictly causal captures')
  measured=np.asarray(original.proprio_history[-1,9:17],float).copy()
  if measured.shape!=(8,) or not np.isfinite(measured).all():raise ValueError('finite measured joints/actions')
  row=np.r_[v,R@gyro,-R.T@np.array([0,0,1.]),measured]
  q=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
  if self.quaternions and np.dot(q,self.quaternions[-1])<0:q=-q
  self.times.append(t);self.proprio.append(row);self.quaternions.append(q)
  self.proprio=self.proprio[-5:];self.quaternions=self.quaternions[-5:]
  def padded(rows):return np.array([rows[0]]*(5-len(rows))+rows).copy()
  yaw=float(np.arctan2(R[1,0],R[0,0]));c,s=np.cos(yaw),np.sin(yaw);goal=np.array([[c,s],[-s,c]])@(target[:2]-p[:2])
  fields=dict(stereo_left_rgb=np.asarray(original.stereo_left_rgb).copy(),stereo_right_rgb=np.asarray(original.stereo_right_rgb).copy(),action_history=np.asarray(original.action_history).copy(),proprio_history=padded(self.proprio),position_world_xy=p[:2].copy(),yaw_world_rad=yaw,quaternion_world_wxyz_history=padded(self.quaternions))
  return type(original)(**fields),goal
