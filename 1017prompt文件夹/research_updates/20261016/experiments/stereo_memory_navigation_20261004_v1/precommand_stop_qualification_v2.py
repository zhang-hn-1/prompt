"""Actual pre-command qualification. Inclusive capture boundary, no future samples."""
import numpy as np

def qualify(world_xy_speed,lo,hi):
 speed=np.asarray(world_xy_speed,float)
 if speed.ndim!=1 or not isinstance(lo,int) or not isinstance(hi,int) or not 0<=lo<=hi<len(speed):raise ValueError('valid inclusive measured capture indices')
 measured=speed[lo:hi+1]
 if not np.isfinite(measured).all() or (measured<0).any():raise ValueError('finite nonnegative current/past actual worldXY speeds')
 count=0;qualified=None;rebounds=0
 for frame,value in enumerate(measured,lo):
  previous=count;count=count+1 if value<.05 else 0
  if previous and count==0:rebounds+=1
  if count==5 and qualified is None:qualified=frame
 return dict(qualified_frame=qualified,rebound_resets=rebounds,measured_last_inclusive=hi)
