"""Reverse tracking must respect the existing geometric actuator convention."""

import importlib

import numpy as np

from training.action_protocol import ActionProtocol


def test_reverse_target_behind_robot_drives_backward():
    protocol = ActionProtocol("ball_action_v2_aux30", (-2.0, -np.pi / 6),
                              (2.0, np.pi / 6), 5)
    fn = importlib.import_module("reverse_route").reverse_tracker_action
    action = fn(protocol, np.array([-5.0, 0.0])).reshape(5, 2)
    assert np.allclose(action[:, 0], -2.0)
    assert np.allclose(action[:, 1], 0.0)


def test_reverse_steering_sign_and_goal_stop():
    protocol = ActionProtocol("ball_action_v2_aux30", (-2.0, -np.pi / 6),
                              (2.0, np.pi / 6), 5)
    fn = importlib.import_module("reverse_route").reverse_tracker_action
    behind_left = fn(protocol, np.array([-5.0, 1.0])).reshape(5, 2)
    behind_right = fn(protocol, np.array([-5.0, -1.0])).reshape(5, 2)
    assert np.all(behind_left[:, 0] < 0)
    assert np.all(behind_left[:, 1] < 0)
    assert np.all(behind_right[:, 1] > 0)
    assert np.allclose(fn(protocol, np.array([0.1, 0.0])), 0.0)
