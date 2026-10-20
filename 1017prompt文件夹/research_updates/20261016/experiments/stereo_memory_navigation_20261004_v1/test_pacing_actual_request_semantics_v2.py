from types import SimpleNamespace as S
import numpy as np
from pathlib import Path
from benchmark.policies import OraclePathPolicy
from scripts.run_pointgoal_benchmark import load_action_protocol

def test_original_request_emission_relabels_fresh_geometry_to_tracking():
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 tracker=object.__new__(OraclePathPolicy);tracker.protocol=protocol;tracker._tracking_records=[]
 tracker._tracking_audit={'valid':True,'source':'tracking_geometry'}
 tracker._active_drive_direction='forward';tracker._pending_drive_direction=None;tracker._braking_state='tracking'
 from straight_pacing_consumer_v2 import with_straight_pacing
 class Base:
  def act(self,o,g):
   a=tracker._request_action(.5,.001,source='tracking');self.records.append({'request':a.tolist()});return a
 x=with_straight_pacing(Base)();x.tracker=tracker;x.records=[]
 action=x.act(S(quaternion_world_wxyz_history=np.array([[1.,0,0,0]])),None)
 assert tracker.tracking_diagnostics['source']=='tracking' and tracker.tracking_diagnostics['valid']
 assert np.all(action[::2]==1.),'current real emission source tracking must permit the unchanged intended eligibility'
