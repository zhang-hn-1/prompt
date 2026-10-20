"""Online ideal-sensor emulator boundary and read-only rolling pose estimator."""
import numpy as np
from rolling_odometry import RollingOdometry

def emulate_sensor_row(diagnostics):
    q=np.asarray(diagnostics['root_quaternion_world_wxyz'][-1],float)
    q=q/np.linalg.norm(q);w,x,y,z=q
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    sensor=dict(time_s=float(diagnostics['simulation_time_s'][-1]),gyro_body_rad_s=R.T@np.asarray(diagnostics['angular_velocity_world_rad_s'][-1]),
        main_encoder_rad_s=float(diagnostics['joint_velocity_rad_s'][-1][0]))
    return sensor,R.T@np.array([0.,0.,1.])

class OnlineRollingShadow:
    def __init__(self,radius=.4000000059604645):self.radius=radius;self.model=None;self.records=[];self.sensors=[];self.initial_normal=None
    def consume(self,sensor,initial_normal=None):
        if set(sensor)!={'time_s','gyro_body_rad_s','main_encoder_rad_s'}:raise ValueError('sensor-only whitelist required')
        if self.model is None:
            if initial_normal is None:raise ValueError('initial gravity calibration required')
            normal=np.asarray(initial_normal,float).copy();model=RollingOdometry(self.radius,normal)
            result=model.update(sensor['time_s'],sensor['gyro_body_rad_s'],sensor['main_encoder_rad_s'])
            self.initial_normal=normal;self.model=model
        else:result=self.model.update(sensor['time_s'],sensor['gyro_body_rad_s'],sensor['main_encoder_rad_s'])
        self.sensors.append(dict(time_s=float(sensor['time_s']),gyro_body_rad_s=np.asarray(sensor['gyro_body_rad_s']).tolist(),main_encoder_rad_s=float(sensor['main_encoder_rad_s'])))
        self.records.append(dict(time_s=float(sensor['time_s']),**{k:np.asarray(v).tolist() for k,v in result.items()}))
        return None # deliberately never a control request
