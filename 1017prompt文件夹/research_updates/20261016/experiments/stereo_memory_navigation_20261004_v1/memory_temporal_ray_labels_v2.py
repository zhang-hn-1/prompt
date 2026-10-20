"""OFFLINE teacher-ray availability only; must never be supplied to a policy.
Ground/elevated point support cannot certify complete 3D freedom or occupancy.
"""
import numpy as np

def memory_cells(xy,origin,resolution):
 origin=np.asarray(origin,float);resolution=float(resolution);xy=np.asarray(xy,float)
 if origin.shape!=(2,) or not np.isfinite(origin).all() or not np.isfinite(resolution) or resolution<=0:raise ValueError('explicit finite memory origin and positive resolution required')
 if xy.shape[-1]!=2 or not np.isfinite(xy).all():raise ValueError('finite XY required')
 return np.rint((xy-origin)/resolution).astype(int)

def label_ray(*,point_world,camera_usd_world,depth_m,self_mask,past_time_s,current_time_s,memory_origin,memory_resolution):
 times=np.asarray([past_time_s,current_time_s],float)
 if not np.isfinite(times).all() or times[1]<=times[0]:raise ValueError('strict finite later teacher capture required')
 point=np.asarray(point_world,float);T=np.asarray(camera_usd_world,float);depth=np.asarray(depth_m,float);mask=np.asarray(self_mask)
 if point.shape!=(3,) or T.shape!=(4,4) or not np.isfinite(point).all() or not np.isfinite(T).all():raise ValueError('finite calibrated point/transform required')
 if depth.shape!=(480,640) or mask.shape!=depth.shape or mask.dtype!=bool:raise ValueError('explicit same-camera depth/body-mask required')
 memory_cells(point[:2],memory_origin,memory_resolution)
 if not np.allclose(T[3],np.array([0.,0.,0.,1.]),atol=1e-9,rtol=0):raise ValueError('homogeneous camera transform required')
 R=T[:3,:3]@np.diag([1.,-1.,-1.])
 if not np.allclose(R.T@R,np.eye(3),atol=1e-6,rtol=0) or not np.isclose(np.linalg.det(R),1.,atol=1e-6,rtol=0):raise ValueError('rigid camera transform required')
 optical=R.T@(point-T[:3,3]);out=dict(label='unlabelled',reason=None,offline_teacher_only=True,free_volume_certified=False)
 if optical[2]<=0:return dict(out,reason='behind_camera')
 uv=optical[:2]/optical[2]*584.8888888888889+np.array([319.5,239.5]);x,y=np.floor(uv+.5).astype(int)
 if not (0<=x<640 and 0<=y<480):return dict(out,reason='offscreen')
 if mask[y,x]:return dict(out,reason='masked')
 z=depth[y,x]
 if not np.isfinite(z) or z<=0:return dict(out,reason='invalid_depth')
 if z<optical[2]-.02:return dict(out,reason='occluded')
 if z>optical[2]+.02:return dict(out,reason='disoccluded')
 ray=np.array([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,1.]);actual=T[:3,3]+R@(ray*z)
 if not np.array_equal(memory_cells(actual[:2],memory_origin,memory_resolution),memory_cells(point[:2],memory_origin,memory_resolution)):return dict(out,reason='different_world_cell')
 label='ground_support' if -.02<=actual[2]<=.05 else 'elevated_surface_support' if .05<actual[2]<=1.5 else 'unlabelled'
 return dict(out,label=label,reason='visible_same_cell' if label!='unlabelled' else 'height_outside_target',teacher_surface_world=actual.tolist(),query_pixel=[int(x),int(y)])
