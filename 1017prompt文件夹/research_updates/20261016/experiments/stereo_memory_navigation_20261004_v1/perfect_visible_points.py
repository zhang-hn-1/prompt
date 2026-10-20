"""Privileged ideal currently binocular-visible renderer endpoints. NOT RGB inference."""
import numpy as np
F=584.8888888888889

def perfect_packet(current,left_mask,right_mask,frame):
 if set(current)!={'left_depth_m','right_depth_m','left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s'}:raise ValueError('current privileged input white-list')
 t=float(current['time_s'])
 if not np.isfinite(t):raise ValueError('current time')
 left=np.asarray(current['left_depth_m']);right=np.asarray(current['right_depth_m']);lm=np.asarray(left_mask,bool);rm=np.asarray(right_mask,bool)
 if not(left.shape==right.shape==lm.shape==rm.shape==(480,640)):raise ValueError('rig shape')
 pose=np.asarray(current['left_camera_usd_world']);rpose=np.asarray(current['right_camera_usd_world']);root=np.asarray(current['root_position_world_m'])
 for p in (pose,rpose):
  if p.shape!=(4,4) or not np.isfinite(p).all() or not np.allclose(p[3],[0,0,0,1]) or not np.allclose(p[:3,:3].T@p[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(p[:3,:3]),1,atol=1e-6):raise ValueError('rigid pose')
 if root.shape!=(3,) or not np.isfinite(root).all():raise ValueError('root')
 y,x=np.mgrid[0:480:2,0:640:2];z=left[y,x];valid=np.isfinite(z)&(z>=.5)&(z<=3)&~lm[y,x];yy=y[valid];xx=x[valid];z=z[valid]
 rays=np.column_stack([(xx-319.5)/F,(yy-239.5)/F,np.ones(len(xx))]);world=pose[:3,3]+(rays*z[:,None])@(pose[:3,:3]@np.diag([1,-1,-1])).T
 xyz=(world-rpose[:3,3])@(rpose[:3,:3]@np.diag([1,-1,-1]))
 with np.errstate(invalid='ignore',divide='ignore'):u=F*xyz[:,0]/xyz[:,2]+319.5;v=F*xyz[:,1]/xyz[:,2]+239.5
 valid=np.isfinite(xyz).all(1)&np.isfinite(u)&np.isfinite(v)&(xyz[:,2]>0)&(u>=0)&(u<=639)&(v>=0)&(v<=479)&(np.linalg.norm(world-root,axis=1)>=.55)
 ix=np.rint(np.nan_to_num(np.clip(u,0,639))).astype(int);iy=np.rint(np.nan_to_num(np.clip(v,0,479))).astype(int);rz=right[iy,ix]
 valid&=~rm[iy,ix]&np.isfinite(rz)&(rz>0)&(np.abs(rz-xyz[:,2])<=.03+.01*xyz[:,2])
 return dict(world=world[valid],sigma_m=np.zeros(int(valid.sum())),query_pixel_id=(yy*640+xx)[valid],partner_pixel_id=(iy*640+ix)[valid],frame=int(frame),time_s=t,pose_sigma_m=0.,privileged_source=True,certified_free=False,frame_width=640)
