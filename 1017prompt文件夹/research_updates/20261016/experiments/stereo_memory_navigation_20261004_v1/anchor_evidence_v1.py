"""ONE paired-pixel evidence gate, never a depth/class/free-space assertion."""
import numpy as np

def qualify(query_rgb,source_rgb,query_xy,source_xy,query_CAD,source_CAD,now,past_time):
 if not np.isfinite(now) or not np.isfinite(past_time) or past_time>=now:raise ValueError('strictly past source clock before payload')
 a=np.asarray(query_rgb);b=np.asarray(source_rgb);m=np.asarray(query_CAD);n=np.asarray(source_CAD)
 if a.ndim!=3 or a.shape[-1]!=3 or b.shape!=a.shape or a.dtype!=np.uint8 or b.dtype!=np.uint8 or m.shape!=a.shape[:2] or n.shape!=a.shape[:2] or m.dtype!=bool or n.dtype!=bool:raise ValueError('RGB uint8 and paired boolean CAD contract')
 def status(rgb,xy,mask):
  p=np.asarray(xy,float)
  if p.shape!=(2,) or not np.isfinite(p).all():raise ValueError('finite paired pixel coordinates')
  x,y=np.rint(p).astype(int);height,width=mask.shape
  if x<0 or y<0 or x>=width or y>=height:return 'out_of_image'
  if mask[y,x]:return 'CAD_occluded'
  if np.all(rgb[y,x]==0):return 'black_RGB_unresolved'
  return 'nonblack_RGB_candidate'
 q=status(a,query_xy,m);s=status(b,source_xy,n)
 return dict(query_status=q,source_status=s,eligible_anchor=q==s=='nonblack_RGB_candidate',geometry_role='UNKNOWN',measured_geometry=False,certified_free=False,source_model_admitted=False,nonblack_not_surface_certificate=True,black_not_absence_certificate=True)
