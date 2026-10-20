"""One empirical current-bank eligibility for old boost, no inverse-model extension."""
import math
import numpy as np
from roll_pid import bank_from_quaternion
BANK_ERROR_LIMIT=math.radians(3)
BANK_NUMERIC_TOL=32*np.finfo(float).eps
def target_bank(aux):
 aux=float(aux)
 if not math.isfinite(aux):raise ValueError('finite requested angle')
 return .75*float(np.clip(aux,-math.pi/6,math.pi/6))

def with_bank_qualification(base,*,enabled):
 class BankQualifiedBoost(base):
  def act(self,o,goal):
   action=np.asarray(super().act(o,goal),np.float32).copy();before=action.copy();tr=self.tracker;row=self.records[-1];diag=tr._tracking_audit
   if action.shape!=(10,) or not np.isfinite(action).all():raise ValueError('finite current protocol block10')
   normal=bool(diag.get('valid',False) and diag.get('source')=='tracking' and tr._active_drive_direction=='forward' and tr._pending_drive_direction is None and tr._braking_state=='tracking')
   eligible=normal and bool(np.all(action[::2]==action[0]) and .5<float(action[0])<=1. and np.all(action[1::2]==action[1]))
   bank=desired=error=None
   if eligible:
    qs=np.asarray(o.quaternion_world_wxyz_history,float)
    if qs.ndim!=2 or qs.shape[1]!=4 or not len(qs):raise ValueError('current measured wxyz quaternion history')
    bank=bank_from_quaternion(qs[-1]);desired=target_bank(action[1]);error=desired-bank
    if enabled and abs(error)>BANK_ERROR_LIMIT+BANK_NUMERIC_TOL:action[::2]=.5
   assert np.array_equal(action[1::2],before[1::2])
   row.update(bank_qualification_tracking_audit=dict(valid=bool(diag.get('valid',False)),source=diag.get('source')),bank_qualification_enabled=bool(enabled),bank_qualification_eligible=eligible,bank_qualification_applied=not np.array_equal(action,before),bank_qualification_measured_bank_rad=bank,bank_qualification_current_requested_bank_rad=desired,bank_qualification_current_error_rad=error,bank_qualification_limit_rad=BANK_ERROR_LIMIT,bank_qualification_numeric_tol_rad=BANK_NUMERIC_TOL,bank_qualification_parent_request=before.tolist(),bank_qualification_safety_certified=False,bank_qualification_no_model_admission=True,request=action.tolist())
   return action
 return BankQualifiedBoost

def policy_class(enabled):
 from forward_intent_policy_factory_v1 import policy_class as original
 return with_bank_qualification(original('b0'),enabled=enabled)
