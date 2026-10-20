import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter

def bank(wxyz):
 q=np.asarray(wxyz);axis=Rotation.from_quat(q[:,[1,2,3,0]]).apply(np.tile([0.,1.,0.],(len(q),1)))
 return np.arcsin(np.clip(axis[:,2],-1,1))

def geometry(xy,dt=.02,window=31):
 xy=np.asarray(xy);v=savgol_filter(xy,window,3,deriv=1,delta=dt,axis=0);a=savgol_filter(xy,window,3,deriv=2,delta=dt,axis=0);speed=np.linalg.norm(v,axis=1);k=np.full(len(v),np.nan);valid=speed>=.08;k[valid]=(v[valid,0]*a[valid,1]-v[valid,1]*a[valid,0])/speed[valid]**3
 return speed,k
