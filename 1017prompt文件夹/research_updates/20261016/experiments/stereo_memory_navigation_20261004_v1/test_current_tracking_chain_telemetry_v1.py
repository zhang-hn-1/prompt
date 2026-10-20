from types import SimpleNamespace
import numpy as np
import pytest
from current_tracking_chain_telemetry_v1 import with_current_tracking_chain
class Parent:
 def __init__(self):self.tracker=SimpleNamespace(_tracking_records=[]);self.records=[];self.n=0;self.mode='normal'
 def act(self,o,g):
  self.n+=1
  if self.mode!='skip':self.tracker._tracking_records.append({'valid':self.mode=='normal','source':'tracking' if self.mode=='normal' else 'planned_zero','requested_curvature_1_m':.25})
  self.records.append({'request':[.5,0.]*5});return np.array([.5,0.]*5,dtype=np.float32)
def test_single_parent_call_same_action_and_state_on_off():
 a=with_current_tracking_chain(Parent,enabled=False)();b=with_current_tracking_chain(Parent,enabled=True)();x=a.act(None,None);y=b.act(None,None)
 np.testing.assert_array_equal(x,y);assert a.n==b.n==1 and a.tracker._tracking_records==b.tracker._tracking_records
 assert b.records[-1]['current_tracking_chain']['requested_curvature_1_m']==.25
 b.records[-1]['current_tracking_chain']['requested_curvature_1_m']=99
 assert b.tracker._tracking_records[-1]['requested_curvature_1_m']==.25
def test_skip_never_reads_stale_tracker_values():
 p=with_current_tracking_chain(Parent,enabled=True)();p.tracker._tracking_records=[{'valid':True,'source':'tracking','requested_curvature_1_m':99}];p.mode='skip';p.act(None,None)
 assert p.records[-1]['current_tracking_chain']=={'valid':False,'source':'tracker_not_called','fresh_tracker_record':False}
def test_current_early_return_not_normal_tracking():
 p=with_current_tracking_chain(Parent,enabled=True)();p.mode='zero';p.act(None,None)
 assert not p.records[-1]['current_tracking_chain']['valid'] and p.records[-1]['current_tracking_chain']['source']=='planned_zero'
