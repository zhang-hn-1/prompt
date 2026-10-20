"""Unique current-bank goal-braking request source. No main/stop-rule authority."""
import numpy as np
class GoalBrakingMeasuredBankHold:
 def __init__(self):
  self.latched_bank=None;self._episode=False;self._invalid=False;self.records=[]
 def request(self,parent,state,bank_rad,measurement_time_s,now):
  a=np.asarray(parent).copy()
  if a.shape!=(2,) or not np.isfinite(a).all():raise ValueError('finite two-element actual current request')
  if state!='goal_braking' or a[0]!=0:
   self.latched_bank=None;self._episode=False;self._invalid=False;return a
  if not self._episode:
   self._episode=True
   valid=np.isfinite(bank_rad) and np.isfinite(measurement_time_s) and np.isfinite(now) and 0<=now-measurement_time_s<=.100000001 and abs(bank_rad)<=.75*np.deg2rad(30.)
   self._invalid=not valid
   if valid:self.latched_bank=float(bank_rad)
  if not self._invalid:
   a[1]=self.latched_bank/.75
  self.records.append(dict(time_s=float(now),eligible=not self._invalid,latched_bank_rad=self.latched_bank,main_preserved=True,parent_aux_rad=float(np.asarray(parent)[1]),candidate_aux_rad=float(a[1])))
  return a
