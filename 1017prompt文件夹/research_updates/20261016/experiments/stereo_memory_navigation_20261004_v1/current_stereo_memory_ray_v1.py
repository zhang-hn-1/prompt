"""Advisory current-binocular ray evidence; never clears memory or free volume.

Only supplied matched source endpoints are consulted, never renderer teachers.
Source sigma margin is heuristic, not calibrated confidence or pose accuracy.
"""
import numpy as np

def ray_evidence(*,old_point,old_sigma,past_time_s,current_time_s,current_pose,current_packet):
    times=np.array([past_time_s,current_time_s,current_packet['time_s']],float)
    if not np.isfinite(times).all() or not times[0]<times[1] or times[2]!=times[1]:
        raise ValueError('strict past point/current matched source clocks required')
    point=np.asarray(old_point,float);T=np.asarray(current_pose,float);s=float(old_sigma)
    if point.shape!=(3,) or not np.isfinite(point).all() or not np.isfinite(s) or s<0:
        raise ValueError('finite old point/heuristic sigma required')
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0.,0.,0.,1.],atol=1e-9,rtol=0):
        raise ValueError('finite homogeneous pose required')
    R=T[:3,:3]@np.diag([1.,-1.,-1.])
    if not np.allclose(R.T@R,np.eye(3),atol=1e-6,rtol=0) or not np.isclose(np.linalg.det(R),1.,atol=1e-6,rtol=0):
        raise ValueError('proper rigid pose required')
    out=dict(status='unavailable',clear_memory=False,free_volume_certified=False,source_model_admitted=False)
    optical=R.T@(point-T[:3,3])
    if optical[2]<=0:return dict(out,reason='behind_camera')
    x,y=np.floor(optical[:2]/optical[2]*584.8888888888889+[319.5,239.5]+.5).astype(int)
    if not (0<=x<640 and 0<=y<480):return dict(out,reason='offscreen')
    ids=np.asarray(current_packet['query_pixel_id']);world=np.asarray(current_packet['world'],float);sigma=np.asarray(current_packet['sigma_m'],float)
    if ids.ndim!=1 or not np.issubdtype(ids.dtype,np.integer) or world.shape!=(len(ids),3) or sigma.shape!=(len(ids),) or not np.isfinite(world).all() or not np.isfinite(sigma).all() or (sigma<0).any() or (ids<0).any() or (ids>=640*480).any() or len(np.unique(ids))!=len(ids):
        raise ValueError('finite unique matched source packet required')
    matches=np.flatnonzero(ids==y*640+x)
    if not len(matches):return dict(out,reason='no_current_matched_pixel')
    i=int(matches[0]);observed=(R.T@(world[i]-T[:3,3]))[2]
    if observed<=0:raise ValueError('matched endpoint behind camera')
    residual=float(observed-optical[2]);margin=float(.1+2*(s+sigma[i]))
    status='contradiction' if residual>margin else 'occluded' if residual<-margin else 'support_compatible'
    return dict(out,status=status,reason='current_matched_binocular_endpoint',query_pixel=int(ids[i]),depth_residual_m=residual,heuristic_margin_m=margin)
