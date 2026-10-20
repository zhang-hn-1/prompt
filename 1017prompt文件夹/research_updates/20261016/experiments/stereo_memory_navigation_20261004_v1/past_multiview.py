"""Offline source-point diagnostics. None of these truth labels are online observations."""
import numpy as np


def measure_view(points_world, pose_usd_saved, depth, mask, focal, cx, cy, visibility_tolerance,
                 predicted=None, confidence=None, rgb_tolerance=.1):
    points=np.asarray(points_world,float).reshape(-1,3);pose=np.asarray(pose_usd_saved,float)
    depth=np.asarray(depth);mask=np.asarray(mask,bool)
    if depth.ndim!=2 or depth.shape!=mask.shape:raise ValueError('depth/mask shape mismatch')
    world_from_optical=pose[:3,:3]@np.diag([1.,-1.,-1.])
    optical=(points-pose[:3,3])@np.linalg.inv(world_from_optical).T
    z=optical[:,2]
    with np.errstate(invalid='ignore',divide='ignore'):
        xy=optical[:,:2]/z[:,None]*focal+np.array([cx,cy])
        norm=np.sqrt(1+np.sum((optical[:,:2]/z[:,None])**2,axis=1))
    front=np.isfinite(optical).all(axis=1)&(z>0)&np.isfinite(xy).all(axis=1)
    safe_xy=np.where(front[:,None],xy,0.)
    height,width=depth.shape
    inside=front&(safe_xy[:,0]>=0)&(safe_xy[:,0]<width-1)&(safe_xy[:,1]>=0)&(safe_xy[:,1]<height-1)
    # Index only clipped coordinates; outside flags never become measured labels.
    x0=np.floor(np.clip(safe_xy[:,0],0,width-2)).astype(int)
    y0=np.floor(np.clip(safe_xy[:,1],0,height-2)).astype(int)
    xx=np.stack([x0,x0+1,x0,x0+1],axis=1); yy=np.stack([y0,y0,y0+1,y0+1],axis=1)
    body=mask[yy,xx].any(axis=1)
    footprint=depth[yy,xx]
    finite_depth=np.isfinite(footprint).all(axis=1)&(footprint>0).all(axis=1)
    with np.errstate(invalid='ignore'):
        error=np.max(abs(footprint-z[:,None])*norm[:,None],axis=1)
    status=np.zeros(len(points),np.uint8)
    status[front]=1
    status[inside]=2
    status[inside&~body]=3
    status[inside&~body&finite_depth]=4
    visible=inside&~body&finite_depth&(error<=visibility_tolerance)
    status[visible]=5
    rgb_supported=np.zeros(len(points),bool);rgb_accurate=rgb_supported.copy()
    rgb_error=np.full(len(points),np.inf)
    if (predicted is None)!=(confidence is None):raise ValueError('prediction and confidence required together')
    if predicted is not None:
        predicted=np.asarray(predicted);confidence=np.asarray(confidence)
        if predicted.shape!=depth.shape or confidence.shape!=depth.shape:raise ValueError('prediction/confidence shape mismatch')
        p=predicted[yy,xx];c=confidence[yy,xx]
        rgb_supported=inside&~body&np.isfinite(p).all(axis=1)&(p>0).all(axis=1)&np.isfinite(c).all(axis=1)&(c>0).all(axis=1)
        with np.errstate(invalid='ignore'):
            rgb_error=np.max(abs(p-z[:,None])*norm[:,None],axis=1)
        rgb_error[~rgb_supported|~np.isfinite(rgb_error)]=np.inf
        rgb_accurate=rgb_supported&visible&(rgb_error<=rgb_tolerance)
    return dict(status=status,xy=xy,axis_depth_m=z,visible=visible,rgb_supported=rgb_supported,rgb_accurate=rgb_accurate,rgb_error_m=rgb_error)


def measure_history(points_world, query_time_s, pairs, masks, focal, cx, cy, visibility_tolerance, rgb_tolerance):
    if not np.isfinite(query_time_s):raise ValueError('query time must be finite')
    past=[]
    for pair in pairs:
        t=float(pair['time_s'])
        if not np.isfinite(t):raise ValueError('capture time must be finite')
        # Do not access image/pose/prediction fields on current/future records.
        if t<query_time_s:past.append(pair)
    past.sort(key=lambda p:(p['time_s'],p['frame']))
    n=len(np.asarray(points_world).reshape(-1,3));out={}
    for group in ['any_eye','stereo','rgb_reliable']:
        out[group]=np.zeros(n,bool)
    for group in ['any','stereo','rgb']:
        out['first_'+group+'_frame']=np.full(n,-1,int)
        out['first_'+group+'_time_s']=np.full(n,np.nan)
    out['first_any_eye_bits']=np.zeros(n,np.uint8)
    for pair in past:
        left=measure_view(points_world,pair['left_pose'],pair['left_depth'],masks['left'],focal,cx,cy,
                          visibility_tolerance,pair['predicted_left'],pair['confidence_left'],rgb_tolerance)
        right=measure_view(points_world,pair['right_pose'],pair['right_depth'],masks['right'],focal,cx,cy,visibility_tolerance)
        bits=left['visible'].astype(np.uint8)+2*right['visible'].astype(np.uint8)
        flags={'any':bits>0,'stereo':left['visible']&right['visible'],
               'rgb':left['rgb_accurate']&right['visible']}
        for name,flag in flags.items():
            key={'any':'any_eye','stereo':'stereo','rgb':'rgb_reliable'}[name]
            first=flag&~out[key]
            out['first_'+name+'_frame'][first]=int(pair['frame'])
            out['first_'+name+'_time_s'][first]=float(pair['time_s'])
            if name=='any':out['first_any_eye_bits'][first]=bits[first]
            out[key]|=flag
    return out
