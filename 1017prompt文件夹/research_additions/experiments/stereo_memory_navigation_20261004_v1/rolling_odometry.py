"""Causal ideal body gyro + shell-encoder rolling-contact hypothesis, no poses."""
import numpy as np

def exponential(vector):
    v=np.asarray(vector,float);angle=np.linalg.norm(v)
    x,y,z=v;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])
    if angle<1e-8:return np.eye(3)+cross+.5*cross@cross
    return np.eye(3)+(np.sin(angle)/angle)*cross+((1-np.cos(angle))/angle**2)*cross@cross

class RollingOdometry:
    def __init__(self,radius=.400000006,normal=(0,0,1),axis=(0,1,0)):
        self.radius=float(radius);self.normal=np.asarray(normal,float).copy();self.axis=np.asarray(axis,float).copy()
        if not np.isfinite(self.radius) or self.radius<=0 or self.normal.shape!=(3,) or self.axis.shape!=(3,) or not np.isfinite(self.normal).all() or not np.isfinite(self.axis).all() or min(np.linalg.norm(self.normal),np.linalg.norm(self.axis))<=0:
            raise ValueError('finite positive radius and nonzero 3-vector calibration required')
        self.normal/=np.linalg.norm(self.normal);self.axis/=np.linalg.norm(self.axis)
        self.R=np.eye(3);self.position=np.zeros(3);self.time=None;self.gyro=None;self.velocity=None
    def update(self,time_s,gyro_body,main_encoder_rad_s):
        t=float(time_s);gyro=np.asarray(gyro_body,float).copy();encoder=float(main_encoder_rad_s)
        if not np.isfinite(t) or gyro.shape!=(3,) or not np.isfinite(gyro).all() or not np.isfinite(encoder) or (self.time is not None and t<=self.time):
            raise ValueError('finite sensors and strictly increasing current time required')
        if self.time is not None:
            dt=t-self.time;self.position=self.position+self.velocity*dt;self.R=self.R@exponential(self.gyro*dt)
        shell_omega=self.R@(gyro+self.axis*encoder)
        velocity=self.radius*np.cross(shell_omega,self.normal)
        self.time=t;self.gyro=gyro;self.velocity=velocity
        return dict(position=self.position.copy(),rotation=self.R.copy(),velocity=velocity.copy(),shell_omega_gauge=shell_omega.copy())
