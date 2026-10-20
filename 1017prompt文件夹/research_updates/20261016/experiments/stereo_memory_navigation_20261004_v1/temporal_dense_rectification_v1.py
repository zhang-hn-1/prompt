"""Calibrated SAME-eye virtual stereo geometry; no depth labels, fitting or actor state."""
import numpy as np
import cv2
K=np.array([[584.8888888888889,0.,319.5],[0.,584.8888888888889,239.5],[0.,0.,1.]])

def rectification(A,B,size=(640,480)):
 A=np.asarray(A,float);B=np.asarray(B,float)
 if A.shape!=(4,4) or B.shape!=(4,4) or not np.isfinite(A).all() or not np.isfinite(B).all():raise ValueError('finite current/past USD camera poses')
 if np.linalg.norm(A[:3,3]-B[:3,3])<1e-9:raise ValueError('nonzero camera baseline required')
 RA=A[:3,:3]@np.diag([1.,-1.,-1.]);RB=B[:3,:3]@np.diag([1.,-1.,-1.])
 for R in (RA,RB):
  if not np.allclose(R.T@R,np.eye(3),atol=1e-5,rtol=0) or not np.isclose(np.linalg.det(R),1.,atol=1e-5):raise ValueError('proper optical rotation')
 relative=RB.T@RA;t=RB.T@(A[:3,3]-B[:3,3])
 R1,R2,P1,P2,Q,roi1,roi2=cv2.stereoRectify(K,None,K,None,size,relative,t,flags=cv2.CALIB_ZERO_DISPARITY,alpha=1)
 if not all(np.isfinite(x).all() for x in (R1,R2,P1,P2,Q)):raise ValueError('finite rectification required')
 axis=int(abs(P2[1,3])>abs(P2[0,3]));vertical=axis==1;mirrored=P2[axis,3]>0
 return dict(A=A.copy(),RA=RA,R=(R1,R2),P=(P1,P2),Q=Q,size=size,vertical=vertical,mirrored=bool(mirrored),normalized_size=size[::-1] if vertical else size,baseline_m=float(np.linalg.norm(t)),ROI=(roi1,roi2))

def normalize_pixels(r,pixel,inverse=False):
 p=np.asarray(pixel,float).copy();width=r['normalized_size'][0]
 if inverse:
  if r['mirrored']:p[:,0]=width-1-p[:,0]
  if r['vertical']:p=np.c_[r['size'][0]-1-p[:,1],p[:,0]]
 else:
  if r['vertical']:p=np.c_[p[:,1],r['size'][0]-1-p[:,0]]
  if r['mirrored']:p[:,0]=width-1-p[:,0]
 return p

def normalize_image(r,array):
 a=np.rot90(array) if r['vertical'] else array
 if r['mirrored']:a=a[:,::-1]
 return np.ascontiguousarray(a)

def normalized_projection(r,pose,world,eye):
 T=np.asarray(pose,float);R=T[:3,:3]@np.diag([1.,-1.,-1.]);camera=(np.asarray(world)-T[:3,3])@R;rect=camera@r['R'][eye].T;h=rect@r['P'][eye][:,:3].T
 return normalize_pixels(r,h[:,:2]/h[:,2:])

def reconstruct_world(r,pixel,disparity):
 native=normalize_pixels(r,pixel,inverse=True);d=np.asarray(disparity,float);d=-d if r['mirrored'] else d
 homogeneous=np.c_[native,d,np.ones(len(native))]@r['Q'].T
 with np.errstate(invalid='ignore',divide='ignore'):rect=homogeneous[:,:3]/homogeneous[:,3:]
 return r['A'][:3,3]+rect@r['R'][0]@r['RA'].T
