import numpy as np

from rig_variant import front_narrow_positions, front_external_positions


def test_front_narrow_rig_has_shorter_baseline_and_stays_inside_shell():
    left, right = front_narrow_positions()
    assert np.allclose(left, [0.35, 0.1, 0.1])
    assert np.allclose(right, [0.35, -0.1, 0.1])
    assert abs(np.linalg.norm(left - right) - 0.2) < 1e-12
    assert np.linalg.norm(left) < 0.4
    assert np.linalg.norm(right) < 0.4


def test_front_external_rig_clears_shell_radius():
    left, right = front_external_positions()
    assert np.allclose(left, [0.48, 0.1, 0.1])
    assert np.allclose(right, [0.48, -0.1, 0.1])
    assert abs(np.linalg.norm(left - right) - 0.2) < 1e-12
    assert np.linalg.norm(left) > 0.4
    assert np.linalg.norm(right) > 0.4
