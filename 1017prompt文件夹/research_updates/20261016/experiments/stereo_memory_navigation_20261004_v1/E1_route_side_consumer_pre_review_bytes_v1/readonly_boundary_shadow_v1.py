"""Independent read-only consumer until first request fork; never future labels."""
import copy
import numpy as np

class ReadOnlyBoundaryShadow:
 def __init__(self,baseline,*,enabled=True):
  self.baseline=baseline;self.active=bool(enabled);self.rows=[];self.last_time=-np.inf;self.first_request_fork=None
 def observe(self,candidate_request,observation,goal,source,current):
  if not self.active:return None
  t=float(current['time_s']);s=float(source['time_s']);frame=int(current['frame'])
  if not np.isfinite(t) or not np.isfinite(s) or s>t or t<=self.last_time or frame!=len(self.rows):raise ValueError('causal consecutive shadow query')
  for r in source['records']:
   birth=float(r['time_s'])
   if not np.isfinite(birth) or birth>s:raise ValueError('future source records before payload')
  request=np.asarray(candidate_request,float)
  if request.shape!=(10,) or not np.isfinite(request).all():raise ValueError('finite candidate request')
  self.baseline.set_source_packet(copy.deepcopy(source));self.baseline.set_current_packet(copy.deepcopy(current));action=np.asarray(self.baseline.act(copy.deepcopy(observation),copy.deepcopy(goal)),float)
  if action.shape!=(10,) or not np.isfinite(action).all():raise ValueError('finite baseline shadow request')
  same=bool(np.array_equal(action,request));fork=not same
  row=dict(frame=frame,time_s=t,source_frame=int(source['frame']),source_time_s=s,B0_request=action.tolist(),candidate_request=request.tolist(),same_request=same,first_request_fork=fork,B0_reference=np.asarray(self.baseline.tracker._reference_xy).tolist(),actual_online_current_past_only=True,no_B0_truth_after_fork=True)
  self.rows.append(row);self.last_time=t
  if fork:self.first_request_fork=frame;self.active=False
  return row
