from types import SimpleNamespace
import numpy as np
from block_bridge import BlockBridge
class FakeRuntime:
 def __init__(self,contact=False):
  self._physics_diagnostics={k:[] for k in ('raw_command_target','simulation_time_s','position_world_xy')};self.calls=0;self.contact=contact;self._action_protocol=SimpleNamespace(controls_per_block=5)
 def _clock_metadata(self):return (1.+len(self._physics_diagnostics.get("simulation_time_s",[]))*.02,)
 def execute(self,block):
  assert block.shape==(10,);self.calls+=1
  for pair in block.reshape(5,2):
   n=len(self._physics_diagnostics['simulation_time_s']);self._physics_diagnostics['simulation_time_s'].append(1.+(n+1)*.02);self._physics_diagnostics['raw_command_target'].append(pair);self._physics_diagnostics['position_world_xy'].append((n*.01,0.))
  return SimpleNamespace(physical_contact=self.contact,collision=False)
def bridge(tmp_path):
 b=BlockBridge(tmp_path/'log','s');b.anchor(1.,left_rgb=b'l',right_rgb=b'r',estimated_xy=(0.,0.),estimated_velocity=(0.,0.),estimated_heading=0.,source='native_imu_rolling',initial_gauge=True,flat_ground_prior=True);b.commit(tuple((.3,.1) for _ in range(25)),initial_supervision_xy=(0.,0.));return b
def test_original_blocks(tmp_path):
 b=bridge(tmp_path);r=FakeRuntime()
 for _ in range(5):b.block(r)
 assert r.calls==5 and b.qualify()['scorable']
def test_contact_censored(tmp_path):
 b=bridge(tmp_path);r=FakeRuntime(True);b.block(r);assert b.qualify()['reason']=='physical_contact_censored'
def test_missing_diagnostics(tmp_path):
 b=bridge(tmp_path);r=FakeRuntime();r._physics_diagnostics={};
 import pytest
 with pytest.raises(ValueError):b.block(r)
 assert r.calls==0

def test_poison_prevents_runtime_call(tmp_path):
 import threading,pytest
 b=bridge(tmp_path);r=FakeRuntime()
 def other():
  try:b.journal.require_owner()
  except ValueError:pass
 t=threading.Thread(target=other);t.start();t.join()
 with pytest.raises(ValueError):b.block(r)
 assert r.calls==0
