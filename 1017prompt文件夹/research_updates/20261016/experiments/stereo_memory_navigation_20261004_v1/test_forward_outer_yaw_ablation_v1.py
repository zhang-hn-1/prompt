from types import SimpleNamespace
import numpy as np
from forward_outer_yaw_ablation_v1 import with_forward_outer_yaw_ablation
class Parent:
 def __init__(self):
  from scripts.run_pointgoal_benchmark import load_action_protocol
  from pathlib import Path
  self.protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));self.tracker=SimpleNamespace(_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking');self.records=[];self.n=0;self.mode='tracking'
 def act(self,o,g):
  self.n+=1;self.records.append({'current_tracking_chain':{'valid':True,'source':self.mode,'fresh_tracker_record':True,'calibrated_bank_mapping':{'active':True,'direction':'forward','encoded_aux_rad':.2}},'request':[1.,.25]*5});return np.array([1.,.25]*5,np.float32)
def test_single_act_only_aux_actual_change_main_exact():
 a=with_forward_outer_yaw_ablation(Parent,enabled=False)();b=with_forward_outer_yaw_ablation(Parent,enabled=True)();x=a.act(None,None);y=b.act(None,None)
 assert a.n==b.n==1;np.testing.assert_array_equal(x[::2],y[::2]);assert np.all(y[1::2]==np.float32(.2)) and b.records[-1]['forward_outer_yaw_override']
def test_reverse_and_early_return_parent_request_preserved():
 for mode,direction in [('tracking','reverse'),('goal_braking','forward')]:
  p=with_forward_outer_yaw_ablation(Parent,enabled=True)();p.mode=mode;p.tracker._active_drive_direction=direction;out=p.act(None,None);np.testing.assert_array_equal(out,np.array([1.,.25]*5,np.float32));assert not p.records[-1]['forward_outer_yaw_override']

def test_stale_mapping_payload_never_read():
 class Poison:
  def __getitem__(self,key):raise AssertionError('stale payload must not be consumed')
 class SkippedParent(Parent):
  def act(self,o,g):
   action=super().act(o,g);self.records[-1]['current_tracking_chain']['fresh_tracker_record']=False;self.records[-1]['current_tracking_chain']['calibrated_bank_mapping']=Poison();return action
 p=with_forward_outer_yaw_ablation(SkippedParent,enabled=True)();out=p.act(None,None)
 np.testing.assert_array_equal(out,np.array([1.,.25]*5,np.float32));assert not p.records[-1]['forward_outer_yaw_eligible']
