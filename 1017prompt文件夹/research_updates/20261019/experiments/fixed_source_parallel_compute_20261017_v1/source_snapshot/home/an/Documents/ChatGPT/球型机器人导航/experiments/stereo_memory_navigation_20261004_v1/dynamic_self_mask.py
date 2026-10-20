"""Known CAD + current measured three joints only; no scene/depth/image labels."""
from pathlib import Path
import json,hashlib,sys
import numpy as np,cv2
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H.parent/'fixed_rig_self_mask_and_curve_qualification_20261003_v1'))
from self_mask import mask_triangles

def joints_from_proprio(history):
 a=np.asarray(history)
 if a.ndim!=2 or a.shape[1]!=17 or not len(a):raise ValueError('current public17D proprio required')
 q=np.asarray(a[-1,9:12],float)
 if not np.isfinite(q).all():raise ValueError('finite current three joints')
 return q.copy()

def joint_transform(axis,angle,pos0,quat0,pos1,quat1):
 if axis not in ('X','Y','Z') or not np.isfinite(angle):raise ValueError('finite joint angle and valid axis')
 def frame(pos,q):
  q=np.asarray(q,float);pos=np.asarray(pos,float)
  if q.shape!=(4,) or pos.shape!=(3,) or not np.isfinite(q).all() or not np.isfinite(pos).all() or np.linalg.norm(q)==0:raise ValueError('finite joint frame')
  T=np.eye(4);T[:3,:3]=Rotation.from_quat(np.r_[q[1:],q[0]]).as_matrix();T[:3,3]=pos;return T
 J=np.eye(4);J[:3,:3]=Rotation.from_rotvec(np.eye(3)[('X','Y','Z').index(axis)]*angle).as_matrix();return frame(pos0,quat0)@J@np.linalg.inv(frame(pos1,quat1))

class RobotSelfMask:
 joint_names=('joint1','joint2','joint3')
 def __init__(self):
  from pxr import Usd,UsdGeom,UsdPhysics
  info=json.loads((H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json').read_text());self.identity=info
  for f,s in info['source_layers_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
  self.frozen=dict(np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz'));stage=Usd.Stage.Open(info['asset']);cache=UsdGeom.XformCache();self.triangles={};self.joints={}
  for name in ('base_link','link1','link2','link3'):
   body=stage.GetPrimAtPath('/Rotunbot_wheel/'+name);assert body and body.HasAPI(UsdPhysics.RigidBodyAPI);polygons=[]
   for prim in Usd.PrimRange(body):
    if not prim.IsA(UsdGeom.Mesh):continue
    mesh=UsdGeom.Mesh(prim)
    if mesh.ComputeVisibility()=='invisible':continue
    points=np.asarray(mesh.GetPointsAttr().Get(),float);matrix=np.asarray(cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(body).GetInverse());points=(np.c_[points,np.ones(len(points))]@matrix)[:,:3];counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get());indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());assert np.all(counts==3),'explicit CAD triangles only';polygons.append(points[indices.reshape(-1,3)])
   self.triangles[name]=np.vstack(polygons) if polygons else np.empty((0,3,3))
  for name in self.joint_names:
   j=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath('/Rotunbot_wheel/joints/'+name));assert str(j.GetBody0Rel().GetTargets()[0]).endswith('/base_link');body=str(j.GetBody1Rel().GetTargets()[0]).split('/')[-1]
   quat=lambda q:[float(q.GetReal()),*map(float,q.GetImaginary())]
   self.joints[name]=dict(body=body,axis=j.GetAxisAttr().Get(),pos0=list(j.GetLocalPos0Attr().Get()),quat0=quat(j.GetLocalRot0Attr().Get()),pos1=list(j.GetLocalPos1Attr().Get()),quat1=quat(j.GetLocalRot1Attr().Get()))
 def body_transforms(self,joints):
  q=np.asarray(joints,float)
  if q.shape!=(3,) or not np.isfinite(q).all():raise ValueError('finite three current joints')
  out={'base_link':np.eye(4)}
  for name,angle in zip(self.joint_names,q):
   j=self.joints[name];out[j['body']]=joint_transform(j['axis'],angle,j['pos0'],j['quat0'],j['pos1'],j['quat1'])
  return out
 def project(self,joints,parts=('base_link','link1','link2','link3'),include_frozen=True):
  transforms=self.body_transforms(joints);tri=[]
  for name in parts:
   if name not in self.triangles:raise ValueError('known robot part required')
   T=transforms[name];tri.append(self.triangles[name]@T[:3,:3].T+T[:3,3])
  points=np.vstack(tri);rotation=np.array([[0,0,1],[0,-1,0],[1,0,0]]);out={}
  for eye,y in [('left',.3),('right',-.3)]:
   optical=(points-np.array([.2,y,.05]))@rotation;mask=mask_triangles(optical,640,480,584.8888888888889,319.5,239.5);mask=cv2.dilate(mask.astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool);out[eye]=mask|self.frozen[eye] if include_frozen else mask
  return out
