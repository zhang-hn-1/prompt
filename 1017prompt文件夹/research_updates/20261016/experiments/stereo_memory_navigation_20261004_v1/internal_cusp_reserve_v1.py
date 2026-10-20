"""One final boost qualifier; geometric reserve heuristic, not certified stopping."""
import numpy as np
from measured_speed_consumer_v1 import RESERVE

def cusp_context(policy,o):
 result=dict(eligible=False,remaining_arc_m=None,cusp_index=None,reason='ineligible',certified_stopping_bound=False,observed_free=False)
 # Clock gate precedes all installed reference/state reads.
 try:
  now=float(policy.time);bt=float(policy.reference_decision['measured_time_s'])
  if not np.isfinite([now,bt]).all() or bt>now+1e-8 or now-bt>.5+1e-6:result['reason']='not_current_past_boundary';return result
 except (AttributeError,KeyError,TypeError,ValueError):return result
 try:
  tr=policy.tracker
  if tr._active_drive_direction!='forward' or tr._pending_drive_direction is not None or tr._braking_state!='tracking':return result
  reference=tr._reference;xy=np.asarray(tr._reference_xy,float);position=np.asarray(o.position_world_xy,float);i=int(tr._path_index)
  if position.shape!=(2,) or xy.ndim!=2 or xy.shape[1]!=2 or len(xy)!=len(reference) or not 0<=i<len(reference) or not np.isfinite(np.r_[position,xy.ravel()]).all():result['reason']='invalid_current_reference';return result
  if reference[i].drive_direction!='forward':return result
  cusp=next((j for j in range(i+1,len(reference)) if reference[j].drive_direction!='forward'),None)
  if cusp is None:result['reason']='no_internal_cusp';return result
  begin=i
  while begin>0 and reference[begin-1].drive_direction=='forward':begin-=1
  line=xy[begin:cusp+1];delta=np.diff(line,axis=0);length=np.linalg.norm(delta,axis=1)
  if not len(length) or np.any(length<=0):result['reason']='invalid_current_reference';return result
  u=np.clip(np.sum((position-line[:-1])*delta,axis=1)/length**2,0,1);projection=line[:-1]+u[:,None]*delta;j=int(np.argmin(np.linalg.norm(projection-position,axis=1)));remaining=float(length[j:].sum()-u[j]*length[j]);result.update(remaining_arc_m=remaining,cusp_index=cusp,eligible=remaining<=RESERVE,reason='internal_cusp_same_reserve' if remaining<=RESERVE else 'outside_same_reserve')
  return result
 except (AttributeError,TypeError,ValueError,IndexError):
  result['reason']='invalid_current_reference';return result

def with_internal_cusp_reserve(base,*,enabled):
 class InternalCuspReserve(base):
  def act(self,o,goal):
   action=np.asarray(super().act(o,goal),np.float32).copy();row=self.records[-1];before=action.copy();context=cusp_context(self,o)
   apply=bool(enabled and context['eligible'] and row.get('measured_speed_control_applied',False) and np.all(action[::2]>.5))
   if apply:action[::2]=.5
   row.update(internal_cusp_reserve_context=context,internal_cusp_reserve_enabled=bool(enabled),internal_cusp_reserve_applied=apply,internal_cusp_parent_request=before.tolist(),internal_cusp_reserve_m=RESERVE,request=action.tolist())
   return action
 return InternalCuspReserve
