"""Exact current estimated inputs; no GT fallback or future estimate."""
import numpy as np
from native_pose_consumer_v1 import current_native_pose
from rolling_pose_bridge import vector
def current_native_inputs(record,packet,time_s):
 pose=current_native_pose(record,time_s)
 if packet is None or not np.isfinite(float(packet['time_s'])) or abs(float(packet['time_s'])-float(time_s))>1e-8:raise ValueError('exact current native IMU packet required')
 gyro=vector(packet['gyro_body_rad_s'])
 return pose,gyro,float(np.linalg.norm(np.asarray(pose['velocity_world_m_s'],float)[:2]))
