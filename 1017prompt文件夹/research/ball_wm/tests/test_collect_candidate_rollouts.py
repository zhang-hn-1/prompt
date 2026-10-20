import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from collect_candidate_rollouts import fixed_candidate_action_sequences
from training.action_protocol import ActionProtocol


def _aux30_protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )


def test_fixed_candidate_actions_are_reproducible_and_include_primitives():
    protocol = _aux30_protocol()
    action_min = np.asarray(protocol.minimum, dtype=np.float32)
    action_max = np.asarray(protocol.maximum, dtype=np.float32)
    actions_a = fixed_candidate_action_sequences(5, 16, protocol, 3072)
    actions_b = fixed_candidate_action_sequences(5, 16, protocol, 3072)

    assert actions_a.shape == (16, 5, 10)
    assert actions_a.dtype == np.float32
    assert np.array_equal(actions_a, actions_b)
    controls = actions_a.reshape(16, 5, 5, 2)
    assert np.all(controls >= action_min)
    assert np.all(controls <= action_max)
    assert np.allclose(controls[0], 0.0)
    assert np.any(np.abs(controls[1:, ..., 0]) > 0.0)
    assert np.any(np.abs(controls[1:, ..., 1]) > 0.0)


def test_fixed_candidate_actions_reject_protocol_mismatch():
    with pytest.raises(ValueError, match="horizon=5"):
        fixed_candidate_action_sequences(4, 16, _aux30_protocol(), 1)
    with pytest.raises(ValueError, match="at least 8"):
        fixed_candidate_action_sequences(5, 7, _aux30_protocol(), 1)
