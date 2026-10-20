from pathlib import Path
from types import SimpleNamespace as S
import math
import numpy as np
import pytest
from benchmark.policies import OraclePathPolicy
from scripts.run_pointgoal_benchmark import load_action_protocol
from current_request_context_v1 import build_request_context

def emission(valid=True,aux=math.radians(35)):
 p=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 t=object.__new__(OraclePathPolicy);t.protocol=p;t._tracking_records=[]
 t._active_drive_direction='forward';t._pending_drive_direction=None;t._braking_state='tracking'
 o=S(position_world_xy=np.array([1.,2.]),yaw_world_rad=0.,proprio_history=np.zeros((1,17)))
 t._begin_tracking_audit(o);t._tracking_audit.update(valid=valid,source='tracking_geometry',requested_curvature_1_m=.1)
 a=t._request_action(.5,aux,source='tracking')
 return t,o,a

def test_actual_emission_raw_protocol_and_outer_issued_are_separate():
 t,o,a=emission();issued=a.copy();issued[::2]=.3
 c=build_request_context(frame=5,time_s=.5,observation=o,tracker=t,before_emission_count=0,issued_request=issued)
 assert c.raw_tracker_component==(.5,float(np.float32(math.radians(35))))
 assert c.protocol_tracker_component==(.5,float(np.float32(math.radians(30))))
 assert c.issued_request==tuple(issued)
 assert c.geometry_computed_this_call and c.normal_tracking_this_call
 assert c.emission_source=='tracking'
 assert c.applied_target is None

def test_stale_no_new_emission_rejected():
 t,o,a=emission()
 with pytest.raises(ValueError,match='one new emission'):
  build_request_context(frame=5,time_s=.5,observation=o,tracker=t,before_emission_count=1,issued_request=a)

def test_early_return_cannot_reuse_old_geometry_or_promote_source():
 t,o,a=emission(valid=False,aux=0.)
 c=build_request_context(frame=5,time_s=.5,observation=o,tracker=t,before_emission_count=0,issued_request=a)
 assert not c.geometry_computed_this_call and not c.normal_tracking_this_call
 assert c.requested_curvature_1_m is None

def test_current_observation_mismatch_rejected():
 t,o,a=emission();o.position_world_xy=np.array([1.1,2.])
 with pytest.raises(ValueError,match='current observation'):
  build_request_context(frame=5,time_s=.5,observation=o,tracker=t,before_emission_count=0,issued_request=a)

def test_original_reference_reset_record_list_is_explicit_not_stale():
 t,o,a=emission();old_records=t._tracking_records
 t._tracking_records=[];t._begin_tracking_audit(o);t._tracking_audit.update(valid=True,requested_curvature_1_m=.1)
 a=t._request_action(.5,0.,source='tracking')
 c=build_request_context(frame=6,time_s=.6,observation=o,tracker=t,before_emission_count=1,before_emission_records=old_records,issued_request=a)
 assert c.records_reset_during_act and c.emission_count==1
