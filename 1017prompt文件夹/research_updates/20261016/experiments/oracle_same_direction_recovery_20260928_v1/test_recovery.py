import importlib.util
import numpy as np
from pathlib import Path
from candidate import RecoveryOracle
spec=importlib.util.spec_from_file_location('helpers','/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/tests/test_round8_tracking_audit.py')
h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
def build():
    p=RecoveryOracle(h._protocol());p.reset(h._context(h._straight_reference()));return p
def test_no_intervention_before_live_event():
    p=build();p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    assert not p.recovery_active
    assert p._scheduled_main_command_rad_s==.5
def test_event_changes_actual_request_without_reference_edit():
    p=build();reference=p._reference;p.handoff_index=0;p._path_index=1
    a=p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    assert p.recovery_active and p._reference is reference
    assert p._scheduled_main_command_rad_s==1.
    assert np.all(a[::2]==1.)
def test_intervention_releases_on_next_reverse_request_without_forcing_switch():
    p=build();p.handoff_index=0;p._path_index=1
    p.act(h._observation(),pointgoal_base_xy=(2.,0.))
    a=p._handle_direction_change('reverse',.2)
    assert not p.recovery_active
    assert p._active_drive_direction=='forward' and p._pending_drive_direction=='reverse'
    assert np.all(a==0.) and p._stopped_cycles==0
