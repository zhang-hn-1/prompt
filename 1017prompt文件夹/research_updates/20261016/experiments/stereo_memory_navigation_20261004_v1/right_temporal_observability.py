"""Reuse unchanged old triangulation on supplied frozen native optical poses only."""
from pathlib import Path
import importlib.util,sys
import numpy as np
H=Path(__file__).resolve().parent

def load(path,name):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
geom=load(H.parent/'bilateral_temporal_geometry_navigation_20261003_v1/geometry.py','_right_observability_geometry')
old=sys.modules.get('geometry');sys.modules['geometry']=geom
try:oldtracks=load(H.parent/'clean_visual_hard0_intervention_20261003_v1/temporal_tracks.py','_right_observability_old_tracks')
finally:
 if old is None:sys.modules.pop('geometry',None)
 else:sys.modules['geometry']=old
K=np.array([[geom.FOCAL,0,319.5],[0,geom.FOCAL,239.5],[0,0,1.]])

def triangulate(a,A,b,B):
 a=np.asarray(a,float).reshape(-1,2);b=np.asarray(b,float).reshape(-1,2)
 if a.shape!=b.shape:raise ValueError('paired coordinates')
 A=np.asarray(A,float);B=np.asarray(B,float)
 if A.shape!=(4,4) or B.shape!=(4,4) or not np.isfinite(A).all() or not np.isfinite(B).all():raise ValueError('finite camera optical transforms')
 points,ok=oldtracks.triangulate_matches(a,(A[:3,3],A[:3,:3]),b,(B[:3,3],B[:3,:3]))
 baseline=np.linalg.norm(A[:3,3]-B[:3,3])
 if baseline<.1:return points,np.zeros(len(a),bool)
 qa=(points-A[:3,3])@A[:3,:3];qb=(points-B[:3,3])@B[:3,:3]
 ra=points-A[:3,3];rb=points-B[:3,3]
 with np.errstate(divide='ignore',invalid='ignore'):angle=np.arccos(np.clip((ra*rb).sum(1)/(np.linalg.norm(ra,axis=1)*np.linalg.norm(rb,axis=1)),-1,1))
 relative=B[:3,:3].T@A[:3,:3];t=B[:3,:3].T@(A[:3,3]-B[:3,3]);x,y,z=t;cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);F=np.linalg.inv(K).T@cross@relative@np.linalg.inv(K);ha=np.c_[a,np.ones(len(a))];hb=np.c_[b,np.ones(len(b))];lb=ha@F.T;la=hb@F;res=np.abs((hb*lb).sum(1))
 with np.errstate(divide='ignore',invalid='ignore'):ep=np.maximum(res/np.linalg.norm(lb[:,:2],axis=1),res/np.linalg.norm(la[:,:2],axis=1))
 ok &= (qa[:,2]>=.5)&(qa[:,2]<=3)&(qb[:,2]>=.5)&(qb[:,2]<=3)&(angle>=np.deg2rad(1))&(ep<=1.5)
 return points,ok
