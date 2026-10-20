"""Necessary falsification only: neither corner nor axis sampling certifies a bound."""
import itertools,numpy as np
from feature_tracks import _point

def screen(query_xy,source_xy,current_pose,past_pose,K,now,past_time):
 if not np.isfinite(now) or not np.isfinite(past_time) or past_time>=now:raise ValueError('strictly past source clock before payload')
 a=np.asarray(query_xy,float);b=np.asarray(source_xy,float);pose0=np.asarray(current_pose,float);pose1=np.asarray(past_pose,float);k=np.asarray(K,float)
 if a.shape!=(2,) or b.shape!=(2,) or pose0.shape!=(4,4) or pose1.shape!=(4,4) or k.shape!=(3,3) or not all(np.isfinite(v).all() for v in [a,b,pose0,pose1,k]):raise ValueError('finite geometry shapes')
 point,lam,_=_point(a,b,pose0,pose1,k)
 if min(lam)<=0 or not np.isfinite(point).all():raise ValueError('finite positive nominal geometry')
 def perturb(delta):
  ab=np.r_[a,b]+delta;p,ll,_=_point(ab[:2],ab[2:],pose0,pose1,k)
  return float(np.linalg.norm(p-point)) if min(ll)>0 and np.isfinite(p).all() else np.inf
 axes=[perturb(np.eye(4)[j]*s) for j in range(4) for s in [-1.,1.]];corners=[perturb(np.array(v)) for v in itertools.product([-1.,1.],repeat=4)]
 separate=max(axes);joint=max(corners)
 return dict(world=point.tolist(),separate_axis_max_displacement_m=float(separate),joint_max_displacement_m=float(joint),joint_positive_finite_count=int(sum(np.isfinite(v) for v in corners)),joint_corners=16,all_joint_corners_in_separate_radius=bool(np.isfinite(separate) and np.isfinite(joint) and joint<=separate+1e-9),certified_bound=False,measured_geometry=False,certified_free=False,geometry_role='UNKNOWN_surface_diagnostic_only',sampling_does_not_certify_continuous_or_probabilistic_noise=True)
