"""Current bearing plus explicit range hypotheses; never measured 3D/free space."""
import numpy as np
from feature_tracks import rotation,triangulate

def _pose(value):
    a=np.asarray(value,float)
    if a.shape!=(4,4) or not np.isfinite(a).all() or not np.allclose(a[3],[0,0,0,1]):raise ValueError('finite homogeneous camera pose')
    r=a[:3,:3]
    if not np.allclose(r.T@r,np.eye(3),atol=1e-5,rtol=0) or not np.isclose(np.linalg.det(r),1.,atol=1e-5):raise ValueError('proper rotation required')
    return a.copy()

def build_context(xy,current_pose,K,time_s,frame):
    xy=np.asarray(xy,float);k=np.asarray(K,float);t=float(time_s)
    if xy.ndim!=2 or xy.shape[1]!=2 or not np.isfinite(xy).all() or not np.isfinite(t):raise ValueError('finite current pixels/time')
    if k.shape!=(3,3) or not np.isfinite(k).all() or abs(np.linalg.det(k))<1e-9:raise ValueError('invertible calibration')
    return dict(xy=xy.copy(),pose=_pose(current_pose),K=k.copy(),time_s=t,frame=int(frame),role='current_2D_bearings',observed_depth=False)

def assess_view(context,proposed_pose,depth_hypotheses):
    b=_pose(proposed_pose);a=context['pose'];k=context['K'];depths=np.asarray(depth_hypotheses,float)
    if depths.ndim!=1 or not len(depths) or not np.isfinite(depths).all() or (depths<=0).any():raise ValueError('positive explicit optical-depth hypotheses')
    r0=rotation(a);r1=rotation(b);rows=[]
    for i,xy in enumerate(context['xy']):
        ray=np.linalg.solve(k,np.r_[xy,1.]);ray=ray/ray[2]
        for z in depths:
            world=a[:3,3]+r0@(ray*z);camera=r1.T@(world-b[:3,3]);pixel=k@camera
            valid=camera[2]>0
            uv=pixel[:2]/pixel[2] if valid else np.array([np.inf,np.inf])
            inside=bool(valid and 0<=uv[0]<=639 and 0<=uv[1]<=479)
            result=triangulate(xy,uv,a,b,k) if inside else None
            rows.append(dict(query_index=i,optical_depth_hypothesis_m=float(z),predicted_pixel=uv.tolist() if valid else None,in_image=inside,geometry_valid=result is not None,sigma_m=None if result is None else result['sigma_m'],sigma_pass=bool(result is not None and result['sigma_m']<=.1)))
    return dict(rows=rows,role='conditional_geometric_prediction',observed_geometry=False,certified_free=False,range_hypotheses_not_measured=True,physical_camera_response_predicted=False,time_s=context['time_s'],frame=context['frame'])
