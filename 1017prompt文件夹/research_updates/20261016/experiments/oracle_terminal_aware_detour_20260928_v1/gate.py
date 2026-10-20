import numpy as np

def clearance(polyline,center,radius):
 p=np.asarray(polyline,float);c=np.asarray(center,float);a=p[:-1];v=p[1:]-a;d=np.sum(v*v,axis=1)
 u=np.divide(np.sum((c-a)*v,axis=1),d,out=np.zeros(len(d)),where=d>0)
 near=a+np.clip(u,0,1)[:,None]*v
 return float(np.linalg.norm(near-c,axis=1).min()-radius)
