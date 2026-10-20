"""Current binocular/past-observation reliability features; never teacher truth."""
import numpy as np
from scipy.spatial import cKDTree
FEATURE_NAMES=['left_R_mean','left_G_mean','left_B_mean','left_R_std','left_G_std','left_B_std','right_R_mean','right_G_mean','right_B_mean','right_R_std','right_G_std','right_B_std','left_patch_gradient','right_patch_gradient','observed_depth_m','observed_disparity_px','estimated_height_flat_prior_m','observed_sigma_m','current_cluster_count','current_cluster_span_X_m','current_cluster_span_Y_m','past_distinct_capture_count','last_past_age_s']
KEYS={'time_s','frame','world','query_pixel_id','partner_pixel_id','sigma_m','depth_m','disparity_native_px','left_rgb','right_rgb'}
def build_features(packet,past_records,now):
 if set(packet)!=KEYS:raise ValueError('current estimated observations whitelist; no teacher fields')
 t=float(packet['time_s']);now=float(now)
 if not np.isfinite([t,now]).all() or t!=now:raise ValueError('exact current capture required before payload')
 for r in past_records:
  rt=float(r['time_s'])
  if not np.isfinite(rt) or rt>now:raise ValueError('past clocks required before any world/image payload')
 w=np.asarray(packet['world'],float).reshape(-1,3);n=len(w);ids=[np.asarray(packet[k]) for k in ('query_pixel_id','partner_pixel_id')];numeric=[np.asarray(packet[k],float) for k in ('depth_m','disparity_native_px','sigma_m')]
 if not np.isfinite(w).all() or any(a.shape!=(n,) for a in ids+numeric) or any(not np.isfinite(a).all() for a in numeric):raise ValueError('aligned finite observed endpoints')
 for a in ids:
  if not np.issubdtype(a.dtype,np.integer) or (a<0).any() or (a>=640*480).any():raise ValueError('native pixel IDs')
 images=[np.asarray(packet[k]) for k in ('left_rgb','right_rgb')]
 if any(a.shape!=(480,640,3) or a.dtype!=np.uint8 for a in images):raise ValueError('current binocular RGB shape/dtype')
 if n==0:return np.empty((0,len(FEATURE_NAMES)))
 tree=cKDTree(w[:,:2]);neighbors=tree.query_ball_point(w[:,:2],r=.1)
 counts=np.zeros(n);age=np.full(n,10.);epochs={}
 for r in past_records:
  rt=float(r['time_s']);frame=int(r['frame'])
  if rt>=now or now-rt>10.:continue
  if frame in epochs and epochs[frame]!=rt:raise ValueError('one acquisition time per actual frame')
  if frame in epochs:continue
  epochs[frame]=rt;old=np.asarray(r['world'],float).reshape(-1,3)
  if not np.isfinite(old).all():raise ValueError('finite historical measured points')
  if len(old):
   distance,_=cKDTree(old[:,:2]).query(w[:,:2],distance_upper_bound=.1);hit=np.isfinite(distance);counts[hit]+=1;age[hit]=np.minimum(age[hit],now-rt)
 features=[]
 for i in range(n):
  patch_stats=[];gradients=[]
  for image,pixel in zip(images,[ids[0][i],ids[1][i]]):
   y,x=divmod(int(pixel),640);patch=image[max(0,y-2):min(480,y+3),max(0,x-2):min(640,x+3)].astype(float)/255.
   patch_stats.extend(np.mean(patch,axis=(0,1)).tolist()+np.std(patch,axis=(0,1)).tolist());gray=patch.mean(axis=-1);g=[np.abs(np.diff(gray,axis=a)).mean() for a in (0,1) if gray.shape[a]>1];gradients.append(float(np.mean(g)) if g else 0.)
  group=w[neighbors[i],:2];span=np.ptp(group,axis=0)
  features.append(patch_stats+gradients+[float(numeric[0][i]),float(numeric[1][i]),float(w[i,2]),float(numeric[2][i]),float(len(group)),float(span[0]),float(span[1]),float(counts[i]),float(age[i])])
 result=np.array(features,float);assert result.shape==(n,len(FEATURE_NAMES)) and np.isfinite(result).all();return result
