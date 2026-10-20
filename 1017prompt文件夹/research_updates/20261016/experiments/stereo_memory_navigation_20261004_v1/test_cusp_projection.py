"""Real inherited tracker tests; no planner or simulator calls."""
import copy
import numpy as np
import pytest

import cusp_projection as m
from oracle_rgb_shadow import make_oracle, reference_from_arrays
from depth_local_curve import install_local_reference, P
from benchmark.policies import EpisodeContext, OraclePathPolicy, PolicyObservation
from training.action_protocol import load_action_protocol
from root_direction_consumer import RootDirectionPolicy


def protocol():
    return load_action_protocol(P / 'configs/data_contract.yaml')


def plan(sign=1):
    # Short opposite segment followed by the accepted direction, exactly the bug pattern.
    return reference_from_arrays([[0, 0, 0], [1, 0, 0], [.9, .1, 0], [2, .1, 0]],
                                [sign, -sign, sign, sign],
                                [[.5*sign, 0], [-.5*sign, 0], [.5*sign, 0], [0, 0]], 2.2, 1)


def tracker(fixed=True, sign=1):
    t = m.make_cusp_oracle(protocol()) if fixed else make_oracle(protocol())
    install_local_reference(t, plan(sign))
    return t


def observation(xy, speed):
    rgb = np.zeros((480, 640, 3), np.uint8)
    proprio = np.zeros((2, 17), np.float32)
    proprio[-1, 0] = speed
    return PolicyObservation(rgb, rgb, proprio, np.zeros((2, 10), np.float32),
                             np.asarray(xy, float), 0.)


@pytest.mark.parametrize('sign', [1, -1])
def test_short_opposite_cusp_cannot_skip_before_acceptance(sign):
    # Break: current opposite waypoint is excluded from the direction-boundary scan.
    old, fixed = tracker(False, sign), tracker(True, sign)
    for t in [old, fixed]:
        t._path_index = 1
        t._pending_drive_direction = 'reverse' if sign > 0 else 'forward'
        t._stopped_cycles = 2
    old._project_to_reference(np.array([.9, .1]))
    error = fixed._project_to_reference(np.array([.9, .1]))
    assert old.path_index == 2  # Explicit inherited reproducer, not a model assumption.
    assert fixed.path_index == 1 and error == pytest.approx(np.sqrt(.02))
    assert fixed._stopped_cycles == 2 and fixed._active_drive_direction == ('forward' if sign > 0 else 'reverse')


def test_cusp_entry_is_allowed_before_any_pending_latch():
    # Break: waiting for pending before allowing entry makes reversal impossible to start.
    t = tracker()
    assert t._pending_drive_direction is None
    t._project_to_reference(np.array([1., 0.]))
    assert t.path_index == 1  # The original boundary waypoint remains reachable.
    t._project_to_reference(np.array([.9, .1]))
    assert t.path_index == 1  # Hold uses reference vs accepted, independent of pending.


@pytest.mark.parametrize('sign', [1, -1])
def test_real_act_keeps_five_low_and_rebound_then_releases(sign):
    # Break: projection silently cancels pending, changes qualification or deadlocks acceptance.
    t = tracker(sign=sign)
    t._path_index = 1
    xy = [.9, .1]
    for speed, count in [(.04, 1), (.04, 2), (.06, 0), (.04, 1), (.04, 2), (.04, 3), (.04, 4)]:
        action = t.act(observation(xy, speed), [3., 0.])
        np.testing.assert_array_equal(action, np.zeros(10, np.float32))
        assert t.path_index == 1 and t._stopped_cycles == count
        assert t._request_source == 'direction_change_braking'
    action = t.act(observation(xy, .04), [3., 0.])
    assert action[0] * sign < 0 and t._stopped_cycles == 0
    assert t._pending_drive_direction is None and t._active_drive_direction == ('reverse' if sign > 0 else 'forward')
    # Acceptance releases the original projection on the next call; no extra condition.
    t._project_to_reference(np.array([.9, .1]))
    assert t.path_index == 2


@pytest.mark.parametrize('sign', [1, -1])
def test_same_direction_full_act_matches_frozen_tracker(sign):
    # Break: candidate changes ordinary projection, geometry, inverse map or request layers.
    old, fixed = tracker(False, sign), tracker(True, sign)
    straight = reference_from_arrays([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]],
                                    [sign]*4, [[.5*sign, 0]]*3+[[0, 0]], 3., 1)
    for t in [old, fixed]:
        install_local_reference(t, straight)
    for xy, speed in [([.1, .05], .1), ([.6, .05], .2), ([1.2, 0.], .3)]:
        np.testing.assert_array_equal(old.act(observation(xy, speed), [5., 0.]),
                                      fixed.act(observation(xy, speed), [5., 0.]))
        assert old.diagnostics == fixed.diagnostics
        assert old.request_diagnostics == fixed.request_diagnostics
        assert old.tracking_diagnostics == fixed.tracking_diagnostics


def test_legitimate_new_reference_can_cancel_old_pending_without_hidden_reset():
    # Break: globally latching old cusp locks an obsolete reference despite valid replacement.
    t = tracker(); t._path_index = 1
    t._pending_drive_direction = 'reverse'; t._stopped_cycles = 3
    straight = reference_from_arrays([[.9, .1, 0], [2, .1, 0], [3, .1, 0]], [1]*3,
                                    [[.5, 0], [.5, 0], [0, 0]], 2.1, 1)
    old_context = t._context
    install_local_reference(t, straight)
    assert t._context is not old_context and t._stopped_cycles == 3
    assert t._pending_drive_direction == 'reverse' and t._active_drive_direction == 'forward'
    action = t.act(observation([.9, .1], .2), [5., 0.])
    assert action[0] > 0 and t._pending_drive_direction is None and t._stopped_cycles == 0


def test_composed_outer_keeps_original_root_consumer_and_act_identity():
    # Break: installing the correction also changes planner or root-cost consumption.
    p = m.CuspProjectionPolicy(protocol())
    assert m.CuspProjectionPolicy.act is RootDirectionPolicy.act
    assert m.CuspProjectionPolicy.plan_boundary is RootDirectionPolicy.plan_boundary
    assert p.tracker.act.__func__ is OraclePathPolicy.act
    assert p.tracker._context is None and p.tracker._reference is None
    p.reset(EpisodeContext('fixture', False, None, p.protocol.protocol_id))
    assert p.tracker._context is None and p.tracker.mapping_records == []


@pytest.mark.parametrize('bad', [None, 'stopped'])
def test_invalid_accepted_direction_rejected_before_projection_mutation(bad):
    # Break: an undefined accepted direction becomes an indefinite cusp hold.
    t = tracker(); t._path_index = 1; t._active_drive_direction = bad
    with pytest.raises(ValueError, match='accepted direction'):
        t._project_to_reference(np.array([.9, .1]))
    assert t.path_index == 1
