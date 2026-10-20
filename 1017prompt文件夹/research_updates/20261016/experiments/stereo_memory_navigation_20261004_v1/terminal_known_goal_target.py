"""Stateless current/given geometry only; does not change reference or stop logic."""
import math
import numpy as np

def proposal(*,position,yaw,goal,reference,directions,path_index,accepted_direction,target,tracking_valid):
 p=np.asarray(position,float);g=np.asarray(goal,float);ref=np.asarray(reference,float);old=np.asarray(target,float)
 if p.shape!=(2,) or g.shape!=(2,) or old.shape!=(2,) or ref.ndim!=2 or ref.shape[1]!=2 or len(ref)<2 or not all(np.isfinite(a).all() for a in [p,g,ref,old]) or not math.isfinite(yaw):raise ValueError('finite current geometry required')
 if accepted_direction not in ('forward','reverse') or len(directions)!=len(ref) or not 0<=path_index<len(ref):raise ValueError('explicit current accepted/reference directions required')
 reason='not_current_terminal_tracking';use=False
 if tracking_valid and np.array_equal(old,ref[-1]) and np.linalg.norm(g-ref[-1])<=.2+1e-8:
  if all(d==accepted_direction for d in directions[path_index:]):
   c,s=math.cos(yaw),math.sin(yaw);dx,dy=g-p;ahead=(c*dx+s*dy)*(1 if accepted_direction=='forward' else -1)
   if ahead>0:use=True;reason='current_terminal_known_goal'
   else:reason='known_goal_behind_accepted_direction'
  else:reason='unaccepted_reference_direction_remains'
 chosen=g.copy() if use else old.copy();dx,dy=chosen-p;c,s=math.cos(yaw),math.sin(yaw);distance2=dx*dx+dy*dy;curvature=0 if distance2<=1e-9 else 2*(-s*dx+c*dy)/distance2
 return dict(target=chosen,curvature=float(curvature),changed=bool(use and not np.array_equal(old,g)),reason=reason,no_reference_or_direction_mutation=True)
