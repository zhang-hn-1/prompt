import numpy as np
def stop_qualification(times,velocities,start_s):
 times=np.asarray(times,float);velocities=np.asarray(velocities,float)
 if times.ndim!=1 or velocities.ndim!=2 or len(times)!=len(velocities) or velocities.shape[1]<2 or not np.isfinite(times).all() or not np.isfinite(velocities).all() or not np.allclose(np.diff(times),.1,atol=1e-5,rtol=0):raise ValueError('complete finite consecutive10Hz capture feedback required')
 count=0;first=None;rebound=False
 for t,v in zip(times,np.asarray(velocities,dtype=np.float32)):
  if t<start_s-1e-5:continue
  if float(np.linalg.norm(v[:2]))<.05:
   count+=1
   if count>=5 and first is None:first=float(t)
  else:
   if first is not None:rebound=True
   count=0
 return dict(first_qualified_s=first,last_count=count,rebound_after_qualification=rebound,actual_worldXY_not_requested_target=True)
