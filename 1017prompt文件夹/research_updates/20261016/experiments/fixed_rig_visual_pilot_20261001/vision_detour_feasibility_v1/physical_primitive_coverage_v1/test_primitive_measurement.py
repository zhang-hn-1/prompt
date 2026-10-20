"""Contract tests for actual-command probes and horizon labels."""

import importlib

import numpy as np
import pytest


def api():
    return importlib.import_module("primitive_measurement")


def test_reverse_is_requested_as_negative_main_without_invented_steering_sign():
    m = api()
    block = m.primitive_block("reverse_positive", controls_per_block=5)
    assert block.shape == (10,)
    assert np.all(block.reshape(5, 2)[:, 0] == -2.0)
    assert np.allclose(block.reshape(5, 2)[:, 1], np.pi / 6)
    assert np.allclose(m.primitive_block("stop", controls_per_block=5), 0)


def test_stationary_survival_cannot_be_a_moving_alternative():
    m = api()
    assert m.local_outcome(6.0, 0.1, any_contact=False, measurement_available=True) == "survived_no_motion"
    assert m.local_outcome(6.0, 0.3, any_contact=False, measurement_available=True) == "survived_moving"


def test_missing_tail_or_contact_sensor_is_not_contact_free_evidence():
    m = api()
    assert m.local_outcome(2.0, 0.4, any_contact=False, measurement_available=True) == "insufficient"
    assert m.local_outcome(6.0, 0.4, any_contact=False, measurement_available=False) == "insufficient"
    assert m.local_outcome(2.4, 0.4, any_contact=True, measurement_available=True) == "contact"


def test_joint_and_smoother_mismatch_rejects_identical_xy_starts():
    m = api()
    a = {"xy": [0.0, 0.0], "joint": [0.0, 0.0, 0.0], "smoother": [0.0, 0.0]}
    assert m.same_start(a, a)
    b = {**a, "joint": [0.0, 0.001, 0.0]}
    assert not m.same_start(a, b)
    assert not m.same_start(a, {**a, "smoother": [0.1, 0.0]})


def test_unknown_primitive_cannot_launch_a_parameter_scan():
    with pytest.raises(ValueError):
        api().primitive_block("other", controls_per_block=5)
