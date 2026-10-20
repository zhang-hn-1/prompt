"""Past/current-only availability features; no teacher loader or future file access.

Unmatched measured cells remain risk/unknown, never become free.
Pose inputs retain the known/estimated diagnostic provenance of their run.
"""
import numpy as np
from memory_temporal_ray_labels_v2 import memory_cells

def past_only_features(*,past,current,pose,history,history_time_s,current_time_s,memory_origin,memory_resolution):
    t=float(current_time_s)
    times=np.array([past['acquisition_time_s'],current['acquisition_time_s'],pose['time_s'],history_time_s,t],float)
    if not np.isfinite(times).all() or not times[0]<t or not np.allclose(times[1:],t,atol=1e-8,rtol=0):
        raise ValueError('strict past source and same-current feature clocks required')
    old=np.asarray(past['world'],float);now=np.asarray(current['world'],float)
    if old.ndim!=2 or now.ndim!=2 or old.shape[1]!=3 or now.shape[1]!=3 or not np.isfinite(old).all() or not np.isfinite(now).all():
        raise ValueError('finite measured point arrays required')
    old=old[:64]
    occupied={tuple(c) for c in memory_cells(now[:,:2],memory_origin,memory_resolution)}
    indices=np.array([i for i,c in enumerate(memory_cells(old[:,:2],memory_origin,memory_resolution)) if tuple(c) not in occupied],int)
    T=np.asarray(pose['left_camera_usd_world'],float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0.,0.,0.,1.],atol=1e-9,rtol=0):
        raise ValueError('finite homogeneous current source pose required')
    R=T[:3,:3]@np.diag([1.,-1.,-1.])
    if not np.allclose(R.T@R,np.eye(3),atol=1e-6,rtol=0) or not np.isclose(np.linalg.det(R),1.,atol=1e-6,rtol=0):
        raise ValueError('proper rigid current source pose required')
    actions=np.asarray(history['action_history'],float);proprio=np.asarray(history['proprio_history'],float)
    if actions.shape!=(5,10) or proprio.shape!=(5,17) or not np.isfinite(actions).all() or not np.isfinite(proprio).all():
        raise ValueError('finite frozen history schema required')
    optical=(old[indices]-T[:3,3])@R
    mean_previous=actions[-1].reshape(5,2).mean(axis=0)
    tail=np.array([t-times[0],*mean_previous,np.linalg.norm(proprio[-1,:2])])
    features=np.concatenate([optical,np.tile(tail,(len(indices),1))],axis=1)
    return dict(source_indices=indices,features=features,points_world=old[indices].copy(),current_support_absence_not_free=True)
