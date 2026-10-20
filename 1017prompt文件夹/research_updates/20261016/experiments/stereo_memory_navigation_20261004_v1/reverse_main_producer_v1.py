"""One empirical final main producer, never a nonzeroaux model admission."""
import numpy as np

def with_reverse_main(base,*,enabled):
 class ReverseMain(base):
  def act(self,o,goal):
   action=np.asarray(super().act(o,goal),np.float32).copy();before=action.copy();row=self.records[-1];tr=self.tracker;diag=tr._tracking_audit;nominal=np.asarray(row['uncapped_request'],np.float32)
   if action.shape!=(10,) or nominal.shape!=(10,) or not np.isfinite(action).all() or not np.isfinite(nominal).all():raise ValueError('finite current requests required')
   eligible=bool(diag.get('valid',False) and diag.get('source')=='tracking' and tr._active_drive_direction=='reverse' and tr._pending_drive_direction is None and tr._braking_state=='tracking' and np.all(action[::2]==np.float32(-.3)) and np.all(nominal[::2]==np.float32(-.5)))
   if enabled and eligible:action[::2]=-.5
   assert np.array_equal(action[1::2],before[1::2])
   row.update(reverse_main_tracking_audit=dict(valid=bool(diag.get('valid',False)),source=diag.get('source')),reverse_main_enabled=bool(enabled),reverse_main_eligible=eligible,reverse_main_applied=not np.array_equal(action,before),reverse_main_parent_request=before.tolist(),reverse_nonzeroaux_model_admitted=False,reverse_empirical_response_unknown=True,request=action.tolist())
   return action
 return ReverseMain

def policy_class(enabled):
 from forward_intent_policy_factory_v1 import policy_class as original
 return with_reverse_main(original('b0'),enabled=enabled)
