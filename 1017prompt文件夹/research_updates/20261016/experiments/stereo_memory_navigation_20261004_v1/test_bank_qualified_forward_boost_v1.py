import math
from types import SimpleNamespace
import numpy as np
from bank_qualified_forward_boost_v1 import with_bank_qualification,target_bank
from roll_pid import BankPID

def fixture(main=.9,aux=.5,bank=0.,direction='forward',pending=None,braking='tracking',valid=True,source='tracking',enabled=True):
 class Base:
  def __init__(self):
   self.calls=0;self.records=[];self.tracker=SimpleNamespace(_tracking_audit=dict(valid=valid,source=source),_active_drive_direction=direction,_pending_drive_direction=pending,_braking_state=braking)
  def act(self,o,goal):
   self.calls+=1;action=np.tile([main,aux],5).astype(np.float32);self.records.append(dict(request=action.tolist()));return action
 o=SimpleNamespace(quaternion_world_wxyz_history=np.array([[math.cos(bank/2),math.sin(bank/2),0,0]]));p=with_bank_qualification(Base,enabled=enabled)();return p,o

def test_current_error_suppresses_boost_only_main_once():
 p,o=fixture();a=p.act(o,[1,0]);assert p.calls==1;assert np.all(a[::2]==.5);assert np.all(a[1::2]==np.float32(.5));assert p.records[-1]['bank_qualification_applied']

def test_all_original_fallbacks_and_good_bank():
 for opts in [dict(direction='reverse'),dict(pending='reverse'),dict(braking='direction_change'),dict(valid=False),dict(source='goal_braking'),dict(main=.5),dict(enabled=False),dict(aux=0.,bank=0.)]:
  p,o=fixture(**opts);a=p.act(o,[1,0]);assert p.calls==1;assert not p.records[-1]['bank_qualification_applied'];assert np.all(a[::2]==np.float32(opts.get('main',.9)))

def test_target_matches_actual_pid_protocol_angle():
 for aux in [0.,math.radians(35),-math.radians(35)]:
  pid=BankPID();pid.step(np.array([.9,aux]),0.);assert target_bank(aux)==pid.last['desired_bank_rad']

def test_boundary_nonfinite_and_other_main_are_not_hidden_tuning():
 import pytest
 p,o=fixture(aux=0.,bank=math.radians(3));p.act(o,[1,0]);assert not p.records[-1]['bank_qualification_applied']
 p,o=fixture(aux=0.,bank=math.radians(3)+1e-6);p.act(o,[1,0]);assert p.records[-1]['bank_qualification_applied']
 p,o=fixture(main=1.1);a=p.act(o,[1,0]);assert np.all(a[::2]==np.float32(1.1)) and not p.records[-1]['bank_qualification_applied']
 p,o=fixture();o.quaternion_world_wxyz_history[:]=np.nan
 with pytest.raises(ValueError):p.act(o,[1,0])
