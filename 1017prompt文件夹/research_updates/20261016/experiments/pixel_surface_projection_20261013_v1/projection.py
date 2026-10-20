"""Fixed calibrated pinhole geometry; no learned or rendered truth dependency."""
import numpy as np
F=584.8888888888889
RIG_ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
FLIP=np.diag([1,-1,-1])
def backproject(u,v,z,T):
 ray=np.stack([(np.asarray(u)-319.5)/F,(np.asarray(v)-239.5)/F,np.ones_like(z)],-1)
 return np.asarray(T)[:3,3]+(ray*np.asarray(z)[...,None])@(np.asarray(T)[:3,:3]@FLIP).T
def project_world(p,T):
 q=(np.asarray(p)-np.asarray(T)[:3,3])@(np.asarray(T)[:3,:3]@FLIP)
 with np.errstate(divide='ignore',invalid='ignore'):return 319.5+F*q[:,0]/q[:,2],239.5+F*q[:,1]/q[:,2],q[:,2]
def root_from_camera(T):
 R=np.asarray(T)[:3,:3]@FLIP@RIG_ROT.T
 return R,np.asarray(T)[:3,3]-R@np.array([.4,.3,.05])
