"""Actual accepted-patch support, catches masked bilinear contributors."""
import numpy as np,cv2,os
if os.environ.get("TEMPORAL_SUPPORT_TEST_V1")=="1":
 from temporal_ray_sweep import TemporalRaySweep,project_depth
else:
 from temporal_ray_sweep_v2 import TemporalRaySweep,project_depth
K=np.array([[40.,0,31.5],[0,40.,23.5],[0,0,1.]])
def pose(x):
 T=np.eye(4);T[:3,:3]=np.diag([1.,-1.,-1.]);T[0,3]=x;return T
def test_every_accepted_patch_has_only_unmasked_bilinear_contributors():
 rng=np.random.default_rng(10);texture=rng.integers(0,255,(48,96),dtype=np.uint8);y,x=np.indices((48,64));packets=[]
 for t,tx in [(2.,0.),(1.,.205),(0.,.405)]:
  img=cv2.remap(texture,(x+40*tx).astype(np.float32),y.astype(np.float32),cv2.INTER_LINEAR);mask=np.zeros((48,64),bool)
  if t==1.:mask[:,27]=True
  packets.append(dict(time_s=t,rgb=np.repeat(img[:,:,None],3,axis=2),pose=pose(tx),mask=mask))
 out=TemporalRaySweep(K).predict(packets[0],packets[1:],2.);yy,xx=np.nonzero(np.isfinite(out['depth_m']));assert len(xx)>100
 dy,dx=np.mgrid[-3:4,-3:4];uv=np.c_[np.repeat(xx,49)+np.tile(dx.ravel(),len(xx)),np.repeat(yy,49)+np.tile(dy.ravel(),len(xx))];depth=np.repeat(out['depth_m'][yy,xx],49)
 for s in packets[1:]:
  xy,_=project_depth(uv,depth,packets[0]['pose'],s['pose'],K);support=cv2.remap((~s['mask']).astype(np.float32),xy[:,0].astype(np.float32).reshape(-1,49),xy[:,1].astype(np.float32).reshape(-1,49),cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
  assert np.all(support>.999),'accepted patch includes masked interpolation contributor'
