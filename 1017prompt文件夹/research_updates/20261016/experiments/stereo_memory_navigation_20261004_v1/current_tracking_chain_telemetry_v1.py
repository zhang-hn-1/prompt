"""Capture only this call's real tracker record, never call policy twice."""
import copy

def with_current_tracking_chain(base,*,enabled):
 class CurrentTrackingChain(base):
  def act(self,o,goal):
   before=len(self.tracker._tracking_records)
   action=super().act(o,goal)
   after=len(self.tracker._tracking_records)
   if after-before not in (0,1):raise RuntimeError('exactly one or zero real tracker call')
   chain={'valid':False,'source':'tracker_not_called','fresh_tracker_record':False}
   if after==before+1:
    chain=copy.deepcopy(self.tracker._tracking_records[-1]);chain['fresh_tracker_record']=True
   if enabled:self.records[-1]['current_tracking_chain']=chain
   return action
 return CurrentTrackingChain

def policy_class(enabled):
 from forward_intent_policy_factory_v1 import policy_class as original
 return with_current_tracking_chain(original('b0'),enabled=enabled)
