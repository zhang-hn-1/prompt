"""Given historical request program, NOT an online B0 tracker or navigator.
Mode annotation is known program input, never current tracker diagnostics.
"""
import numpy as np
from goal_braking_measured_bank_hold import GoalBrakingMeasuredBankHold
class BankHoldRequestProgram:
 def __init__(self,requests,modes,arm):
  self.requests=np.asarray(requests,np.float32).copy();self.requests.setflags(write=False);self.modes=tuple(modes)
  if self.requests.ndim!=2 or self.requests.shape[1]!=10 or len(self.requests)!=len(self.modes) or arm not in ['B0','C']:raise ValueError('given program shape and declared arm')
  self.arm=arm;self.hold=GoalBrakingMeasuredBankHold();self.next_frame=0;self.records=[]
 def act(self,frame,current_bank_rad,measurement_time_s,now):
  if frame!=self.next_frame or frame>=len(self.requests):raise ValueError('sequential current given-program frame only')
  raw=self.requests[frame].copy();mode=self.modes[frame]
  if not np.isfinite(raw).all() or mode not in ['tracking','GIVEN_PROGRAM_GOAL_PAUSE']:raise ValueError('finite current given request and annotated mode')
  pairs=raw.reshape(5,2);goalpause=mode=='GIVEN_PROGRAM_GOAL_PAUSE'
  if goalpause and np.any(pairs[:,0]!=0):raise ValueError('given goalpause only has zero-main commands')
  if self.arm=='C':
   state='goal_braking' if goalpause else 'given_tracking'
   output=np.array([self.hold.request(p,state,current_bank_rad,measurement_time_s,now) for p in pairs],np.float32).reshape(10)
  else:output=raw.copy()
  np.testing.assert_array_equal(output.reshape(5,2)[:,0],pairs[:,0]);self.records.append(dict(frame=frame,time_s=float(now),mode=mode,mode_is_given_program_not_actual_tracker=True,entry_bank_from_current_native_rad=float(current_bank_rad),measurement_time_s=float(measurement_time_s),given_request=raw.tolist(),actual_request=output.tolist(),arm=self.arm));self.next_frame+=1;return output
