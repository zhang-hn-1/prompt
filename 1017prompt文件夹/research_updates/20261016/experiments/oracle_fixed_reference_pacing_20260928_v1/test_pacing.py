from candidate import PlannedSpeedOracle
import importlib.util
spec=importlib.util.spec_from_file_location('helpers','/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/tests/test_round8_tracking_audit.py')
h=importlib.util.module_from_spec(spec);spec.loader.exec_module(h)
def test_restore_only_planned_main_not_above_reference():
    p=PlannedSpeedOracle(h._protocol())
    for planned,baseline in [(1.5,.5),(-1.,.5),(.5,.5)]:
        actual=p._schedule_main_magnitude(h._observation(),baseline_magnitude=baseline,planned_main_command_rad_s=planned,desired_curvature_1_m=1.,cross_track_error_m=.2)
        assert actual==abs(planned)
def test_original_stop_confirmation_and_rebound():
    p=PlannedSpeedOracle(h._protocol());p.reset(h._context(h._straight_reference()))
    for _ in range(4):assert p._handle_direction_change('reverse',.01) is not None
    assert p._handle_direction_change('reverse',.06) is not None and p._stopped_cycles==0
    for _ in range(4):assert p._handle_direction_change('reverse',.01) is not None
    assert p._handle_direction_change('reverse',.01) is None
