from pathlib import Path

import numpy as np
import pytest

from benchmark.policies import ActionProtocol, EpisodeContext, PolicyObservation
from benchmark.world_model_policy import LEGACY_AUXILIARY_LIMIT_RAD, WorldModelPolicy


def _protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )


def _observation() -> PolicyObservation:
    return PolicyObservation(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=np.zeros((5, 17), dtype=np.float32),
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.zeros(2, dtype=np.float64),
        yaw_world_rad=0.0,
    )


def test_legacy_limit_is_inside_benchmark_protocol() -> None:
    protocol = _protocol()
    assert -protocol.minimum[1] > LEGACY_AUXILIARY_LIMIT_RAD
    assert protocol.maximum[1] > LEGACY_AUXILIARY_LIMIT_RAD


def test_policy_rejects_legacy_checkpoint_without_explicit_opt_in(tmp_path: Path) -> None:
    torch = pytest.importorskip("torch")
    checkpoint = tmp_path / "legacy.ckpt"
    checkpoint.write_bytes(b"not a checkpoint")
    with pytest.raises(Exception):
        WorldModelPolicy(_protocol(), checkpoint=checkpoint)


def test_policy_context_has_model_shapes(monkeypatch: pytest.MonkeyPatch) -> None:
    policy = object.__new__(WorldModelPolicy)
    policy.config = type("Config", (), {"image_size": 8, "history_size": 5, "proprio_dim": 17, "action_dim": 10})()
    policy._left_history = []
    policy._right_history = []
    context = policy._model_context(_observation())
    assert context["pixels_left"].shape == (1, 5, 3, 8, 8)
    assert context["pixels_right"].shape == (1, 5, 3, 8, 8)
    assert context["proprio"].shape == (1, 5, 17)
    assert context["action"].shape == (1, 5, 10)
    assert np.allclose(context["action"][0, -1].numpy(), 0.0)
