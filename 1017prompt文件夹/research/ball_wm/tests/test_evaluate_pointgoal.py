import sys
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from training.action_protocol import ActionProtocol
from training.evaluate_pointgoal import _goal_info, _sample_random_action_sequences


class _FakeDataset:
    def _frame_arrays(self, episode_index: int, start: int):
        assert episode_index == 3
        assert start == 11
        return {
            "pixels_left": torch.arange(6 * 3 * 2 * 2, dtype=torch.float32).reshape(6, 3, 2, 2),
            "pixels_right": torch.arange(6 * 3 * 2 * 2, dtype=torch.float32).reshape(6, 3, 2, 2) + 100,
            "action": torch.arange(6 * 10, dtype=torch.float32).reshape(6, 10),
            "proprio": torch.arange(6 * 17, dtype=torch.float32).reshape(6, 17),
        }


def test_goal_info_uses_requested_goal_frame():
    goal = _goal_info(_FakeDataset(), episode_index=3, goal_frame=11, history=5)

    expected_left = torch.arange(3 * 2 * 2, dtype=torch.float32).reshape(1, 1, 3, 2, 2)
    expected_right = expected_left + 100
    assert torch.equal(goal["pixels_left"], expected_left)
    assert torch.equal(goal["pixels_right"], expected_right)
    assert torch.equal(goal["action"], torch.arange(10, dtype=torch.float32).reshape(1, 1, 10))
    assert torch.equal(goal["proprio"], torch.arange(17, dtype=torch.float32).reshape(1, 1, 17))


def test_random_baseline_samples_only_inside_action_protocol():
    protocol = ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )
    torch.manual_seed(3072)

    actions = _sample_random_action_sequences(protocol, 7, 3, torch.device("cpu"))

    assert actions.shape == (1, 7, 3, 10)
    bounds = protocol.block_bounds_torch()
    assert torch.all(actions >= bounds[:, 0])
    assert torch.all(actions <= bounds[:, 1])
