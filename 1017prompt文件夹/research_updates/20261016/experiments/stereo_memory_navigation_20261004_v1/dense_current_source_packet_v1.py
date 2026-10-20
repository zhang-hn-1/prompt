"""Diagnostic CURRENT dense binocular endpoints.

Derived from frozen dense_source_points.point_packet; ONLY stride2 sampling
is removed. No interpolation, training teacher, memory clearing or actor.
"""
import numpy as np

def dense_point_packet(prediction,known_pose,left_mask,right_mask,focal=584.8888888888889,baseline=.6,cx=319.5,cy=239.5):
 if set(prediction)!={'depth_m','confidence','disparity_native_px','frame','time_s'} or set(known_pose)!={'left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s'}:raise ValueError('input white-list')
 t=float(prediction['time_s']);pt=float(known_pose['time_s'])
 if not np.isfinite(t) or not np.isfinite(pt) or t!=pt:raise ValueError('invalid time')
 pose=np.asarray(known_pose['left_camera_usd_world'],float);root=np.asarray(known_pose['root_position_world_m'],float);rpose=np.asarray(known_pose['right_camera_usd_world'],float)
 for p in (pose,rpose):
  if p.shape!=(4,4) or not np.isfinite(p).all() or not np.allclose(p[3],[0,0,0,1],atol=1e-8) or not np.allclose(p[:3,:3].T@p[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(p[:3,:3]),1,atol=1e-6):raise ValueError('invalid rigidpose')
 if root.shape!=(3,) or not np.isfinite(root).all():raise ValueError('invalid root state')
 z=np.asarray(prediction['depth_m']);d=np.asarray(prediction['disparity_native_px']);confidence=np.asarray(prediction['confidence'],bool);lm=np.asarray(left_mask,bool);rm=np.asarray(right_mask,bool)
 if not (z.shape==d.shape==confidence.shape==lm.shape==rm.shape) or z.ndim!=2:raise ValueError('shape')
 y,x=np.indices(z.shape);vp=y-np.where(np.isfinite(d),d,0);iy=np.rint(np.clip(vp,0,z.shape[0]-1)).astype(int)
 valid=confidence&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(d)&(d>1)&~lm&(vp>=0)&(vp<=z.shape[0]-1)&~rm[iy,x]
 yy,xx=np.where(valid);rays=np.column_stack([(xx-cx)/focal,(yy-cy)/focal,np.ones(len(xx))]);depth=z[yy,xx];disparity=d[yy,xx];world=pose[:3,3]+(rays*depth[:,None])@(pose[:3,:3]@np.diag([1,-1,-1])).T
 sigma=np.maximum(np.abs(focal*baseline/(disparity-1)-depth),np.abs(focal*baseline/(disparity+1)-depth))*np.linalg.norm(rays,axis=-1)
 outside_self=np.linalg.norm(world-root,axis=-1)>=.55
 return dict(world=world[outside_self],sigma_m=sigma[outside_self],query_pixel_id=(yy*z.shape[1]+xx)[outside_self],partner_pixel_id=(iy[yy,xx]*z.shape[1]+xx)[outside_self],frame=int(prediction['frame']),time_s=t,pose_sigma_m=0.,source_model_admitted=False,certified_free=False,frame_width=z.shape[1])
