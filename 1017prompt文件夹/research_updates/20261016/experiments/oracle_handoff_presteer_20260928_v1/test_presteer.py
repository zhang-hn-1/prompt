import importlib.util
import numpy as np
from candidate import PresteerOracle
from benchmark.kinodynamic_planning import KinodynamicWaypoint
spec=importlib.util.spec_from_file_location('helpers','/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/tests/test_round8_tracking_audit.py')
h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
def build():
    ref=(KinodynamicWaypoint(0.,0.,0.,'reverse',-.5,-.3),KinodynamicWaypoint(-.3,0.,0.,'forward',.5,.4),KinodynamicWaypoint(1.,0.,0.,'forward',0.,0.))
    p=PresteerOracle(h._protocol());p.reset(h._context(ref));p._path_index=1;return p
def test_braking_prepares_known_target_aux_and_keeps_main_zero():
    p=build();a=p._handle_direction_change('forward',.2)
    assert np.all(a[::2]==0.)
    np.testing.assert_allclose(a[1::2],.4)
    assert p._active_drive_direction=='reverse' and p._stopped_cycles==0
def test_original_five_samples_and_rebound_clear():
    p=build()
    for _ in range(4):assert p._handle_direction_change('forward',.01) is not None
    assert p._handle_direction_change('forward',.06) is not None and p._stopped_cycles==0
    for _ in range(4):assert p._handle_direction_change('forward',.01) is not None
    assert p._handle_direction_change('forward',.01) is None
    assert p._active_drive_direction=='forward'
def test_second_handoff_and_goal_braking_unchanged():
    p=build();p.handoff_index=1
    assert np.all(p._zero_action('direction_change_braking')==0.)
    assert np.all(p._zero_action('goal_braking')==0.)

def test_event_preview_matches_projection_without_mutation():
    p=build();p._path_index=0;p.preview_position=np.array([-.3,0.])
    assert p.ready and p.upcoming_index==1
    assert p._path_index==0 and p._active_drive_direction=='reverse'
    p._project_to_reference(p.preview_position)
    assert p._path_index==p.upcoming_index==1
