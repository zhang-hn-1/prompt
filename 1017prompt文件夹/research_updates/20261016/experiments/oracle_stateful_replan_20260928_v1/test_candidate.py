import importlib.util
from pathlib import Path
import numpy as np
from benchmark.kinodynamic_planning import KinodynamicPlanResult,KinodynamicWaypoint
from candidate import ReplanOracle

spec=importlib.util.spec_from_file_location('helpers','/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/tests/test_round8_tracking_audit.py')
h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)

def build(callback):
    p=ReplanOracle(h._protocol(),replan_callback=callback)
    p.reset(h._context(h._straight_reference()))
    return p

def test_replan_only_at_event_once_and_preserves_direction_latch():
    calls=[]
    ref=h._straight_reference('reverse')
    def cb(obs,goal,*,active_direction):
        assert active_direction=="forward"
        calls.append((obs.position_world_xy.copy(),goal.copy()))
        # Test callback is a reference provider; reversing must still brake.
        return KinodynamicPlanResult('success',ref,0,1.2,0)
    p=build(cb);p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    assert calls==[]
    p.handoff_index=0;p._path_index=1
    p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    assert len(calls)==1
    assert p._active_drive_direction=='forward'
    assert p._pending_drive_direction=='reverse'
    assert np.all(p.tracking_records[-1]['policy_protocol_main_command_rad_s']==0)
    p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    assert len(calls)==1

def test_failure_retains_reference_and_does_not_retry():
    calls=[]
    def cb(obs,goal,*,active_direction):
        assert active_direction=="forward"
        calls.append(1)
        return KinodynamicPlanResult('node_budget_exhausted',(),1,0.,0)
    p=build(cb);original=p._reference;p.handoff_index=0;p._path_index=1
    p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    assert p._reference==original
    assert calls==[1]
