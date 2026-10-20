"""Planning direction intent only; empirical coast/stop qualifications remain frozen."""
import types
from guarded_reference import boundary_state
import numpy as np
from minimum_prefix_transition_consumer_v1 import with_first_primitive_direction,measured_first_direction

def measured_forward_intent(observation,state):
 p=np.asarray(observation.proprio_history,float);yaw=float(observation.yaw_world_rad)
 if p.ndim!=2 or p.shape[1]!=17 or not len(p) or not np.isfinite(p).all() or not np.isfinite(yaw):raise ValueError('finite current proprio17/yaw required')
 heading=np.array([np.cos(yaw),np.sin(yaw)])
 eligible=(state['active_direction']=='forward' and state['pending_direction'] is None and state['braking_state']=='tracking' and p[-1,12]>.1 and np.linalg.norm(p[-1,:2])>.05 and p[-1,:2]@heading>0)
 return 'forward' if eligible else None

def with_current_forward_intent(base,*,candidate=True):
 original=with_first_primitive_direction(base)
 class CurrentForwardIntent(original):
  def act(self,observation,goal):
   # Exact inherited act, one call; only replace this deterministic qualification.
   source=original.act;scope=dict(source.__globals__);qualifier=measured_forward_intent if candidate else measured_first_direction
   state=boundary_state(self.tracker) if self.tracker._context is not None else None
   old=None if state is None else measured_first_direction(observation,state)
   proposed=None if state is None else measured_forward_intent(observation,state)
   scope['measured_first_direction']=qualifier
   method=types.FunctionType(source.__code__,scope,closure=source.__closure__)
   action=method(self,observation,goal)
   row=self.records[-1];query=row.get('first_primitive_direction_query')
   row.update(forward_intent_variant='candidate' if candidate else 'b0',forward_intent_fields_valid=True,forward_intent_original=old,forward_intent_proposed=proposed,forward_intent_decoupling_qualified=bool(candidate and old is None and proposed=='forward'),forward_intent_decoupling_replanned=bool(candidate and old is None and proposed=='forward' and query is not None and row.get('local_plan_created') and query['direction']=='forward'),forward_intent_no_stop_domain_extension=True)
   return action
 return CurrentForwardIntent
