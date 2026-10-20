from types import SimpleNamespace
import numpy as np
from local_reserve_pacing_consumer_v1 import with_local_reserve_pacing,RESERVE_M
class Base:
 def __init__(self):
  self.calls=0;self.records=[];self.tracker=SimpleNamespace(tracking_diagnostics={'valid':True,'source':'tracking'},_active_drive_direction='forward',_pending_drive_direction=None,_braking_state='tracking',_reference_xy=np.array([[0.,0.],[2.1,0.]]))
 def act(self,observation,goal):
  self.calls+=1;a=np.tile(np.array([.5,0.],np.float32),5);self.records.append({'request':a.tolist()});return a

def obs(x):return SimpleNamespace(position_world_xy=np.array([x,0.]),quaternion_world_wxyz_history=np.array([[1.,0.,0.,0.]]))
def test_far_straight_emits_real_high_main_exactly_one_act():
 p=with_local_reserve_pacing(Base)();a=p.act(obs(0.),np.array([8.8,0.]));assert p.calls==1 and np.all(a[::2]==1.) and np.all(a[1::2]==0)
 assert p.records[-1]['local_reserve_pacing_applied']
def test_measured_tail_plus_existing_margin_qualifier_restores_original_request():
 assert RESERVE_M>.9
 p=with_local_reserve_pacing(Base)();a=p.act(obs(1.5),np.array([7.3,0.]));assert p.calls==1 and np.all(a[::2]==.5) and np.all(a[1::2]==0)
 assert not p.records[-1]['local_reserve_pacing_applied']
 assert 'local_endpoint_inside_measured_reserve' in p.records[-1]['local_reserve_pacing_reasons']
def test_global_goal_also_blocks_pacing_even_when_reference_endpoint_far():
 p=with_local_reserve_pacing(Base)();a=p.act(obs(0.),np.array([.3,0.]));assert np.all(a[::2]==.5) and p.calls==1
