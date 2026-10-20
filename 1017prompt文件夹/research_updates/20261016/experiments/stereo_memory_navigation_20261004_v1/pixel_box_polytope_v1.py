"""Fixed +/-1pixel conditional frustum intersection; UNKNOWN, not calibrated truth."""
import numpy as np
from scipy.optimize import linprog

def envelope(query_xy,source_xy,current_pose,past_pose,K,now,past_time):
 if not np.isfinite(now) or not np.isfinite(past_time) or past_time>=now:raise ValueError('strictly past source clock before payload')
 xy=[np.asarray(query_xy,float),np.asarray(source_xy,float)];poses=[np.asarray(current_pose,float),np.asarray(past_pose,float)];k=np.asarray(K,float)
 if k.shape!=(3,3) or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or not np.allclose(k[2],[0,0,1]) or k[0,1]!=0 or k[1,0]!=0:raise ValueError('pinhole K shape')
 A=[];b=[]
 for pixel,T in zip(xy,poses):
  if pixel.shape!=(2,) or T.shape!=(4,4) or not np.isfinite(pixel).all() or not np.isfinite(T).all() or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1):raise ValueError('finite rigid camera/pixel')
  R=T[:3,:3]@np.diag([1,-1,-1]);C=T[:3,3];rx,ry,rz=R.T
  coeff=[k[0,0]*rx+(k[0,2]-pixel[0]-1)*rz,-k[0,0]*rx+(pixel[0]-1-k[0,2])*rz,k[1,1]*ry+(k[1,2]-pixel[1]-1)*rz,-k[1,1]*ry+(pixel[1]-1-k[1,2])*rz,-rz]
  for j,c in enumerate(coeff):A.append(c);b.append(float(c@C)-(1e-9 if j==4 else 0.))
 A=np.array(A);b=np.array(b);base=dict(status=None,lo_world_m=None,hi_world_m=None,constraint_A=A.tolist(),constraint_b=b.tolist(),measured_geometry=False,certified_free=False,statistical_confidence_calibrated=False,geometry_role='UNKNOWN_surface_conditional_pixel_box',pose_uncertainty_included=False,correspondence_assumption_validated=False,source_GT_pose_diagnostic_only=True)
 feasible=linprog(np.zeros(3),A_ub=A,b_ub=b,bounds=[(None,None)]*3,method='highs')
 if feasible.status==2:base['status']='empty';return base
 if not feasible.success:base['status']='solver_failure';base['solver_status']=int(feasible.status);return base
 lo=[];hi=[]
 for j in range(3):
  values=[]
  for sign in [1.,-1.]:
   objective=np.eye(3)[j]*sign;r=linprog(objective,A_ub=A,b_ub=b,bounds=[(None,None)]*3,method='highs')
   if r.status==3:base['status']='unbounded';return base
   if not r.success:base['status']='solver_failure';base['solver_status']=int(r.status);return base
   values.append(float(r.x[j]))
  lo.append(values[0]);hi.append(values[1])
 base.update(status='bounded',lo_world_m=lo,hi_world_m=hi);return base
