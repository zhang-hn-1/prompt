"""Measured-frame parity for the benchmark adapter, without launching Isaac."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig
from benchmark.policies import PolicyObservation
from benchmark.world_model_policy import WorldModelPolicy
from training.action_protocol import ActionProtocol


def _observation(**changes) -> PolicyObservation:
    values = dict(
        stereo_left_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        stereo_right_rgb=np.zeros((480, 640, 3), dtype=np.uint8),
        proprio_history=np.tile(np.arange(1, 18, dtype=np.float32), (5, 1)),
        action_history=np.zeros((5, 10), dtype=np.float32),
        position_world_xy=np.zeros(2),
        yaw_world_rad=0.0,
    )
    values.update(changes)
    return PolicyObservation(**values)


def _policy(frame: str | None = None) -> WorldModelPolicy:
    policy = object.__new__(WorldModelPolicy)
    policy.config = SimpleNamespace(image_size=8, history_size=5, proprio_dim=17, action_dim=10)
    if frame is not None:
        policy.config.proprio_velocity_frame = frame
    policy._left_history = []
    policy._right_history = []
    return policy


def test_base_policy_rotates_each_history_row_with_its_measured_orientation():
    half = np.sqrt(0.5)
    quaternion = np.asarray(
        [(1, 0, 0, 0), (half, 0, 0, half), (half, half, 0, 0),
         (-half, 0, 0, -half), (0, 0, 0, 1)],
        dtype=np.float32,
    )
    observation = _observation(quaternion_world_wxyz_history=quaternion)
    original = observation.proprio_history.copy()
    context = _policy("base")._model_context(observation)

    expected = np.asarray(
        [(1, 2, 3, 4, 5, 6), (2, -1, 3, 5, -4, 6), (1, 3, -2, 4, 6, -5),
         (2, -1, 3, 5, -4, 6), (-1, -2, 3, -4, -5, 6)],
        dtype=np.float32,
    )
    np.testing.assert_allclose(context["proprio"][0, :, :6], expected, atol=1e-6)
    np.testing.assert_array_equal(context["proprio"][0, :, 6:], original[:, 6:])
    np.testing.assert_array_equal(observation.proprio_history, original)


def test_legacy_policy_without_frame_or_quaternion_keeps_world_proprio():
    observation = _observation()
    context = _policy()._model_context(observation)
    np.testing.assert_array_equal(context["proprio"][0], observation.proprio_history)


def test_base_policy_requires_measured_quaternion_history():
    with pytest.raises(ValueError, match="quaternion"):
        _policy("base")._model_context(_observation(yaw_world_rad=np.pi / 2))


def test_only_base_policy_declares_measured_quaternion_input():
    assert "quaternion_world_wxyz_history" in _policy("base").online_input_permissions
    assert "quaternion_world_wxyz_history" not in _policy().online_input_permissions


@pytest.mark.parametrize(
    "quaternion",
    [np.tile((1, 0, 0, 0), (4, 1)), np.zeros((5, 4)), np.full((5, 4), np.nan)],
)
def test_observation_rejects_incomplete_or_invalid_quaternion_history(quaternion):
    with pytest.raises(ValueError, match="quaternion"):
        _observation(quaternion_world_wxyz_history=quaternion)


def test_observation_copies_and_freezes_measured_quaternion_history():
    quaternion = np.tile((1.0, 0.0, 0.0, 0.0), (5, 1))
    observation = _observation(quaternion_world_wxyz_history=quaternion)
    quaternion[:] = (0.0, 0.0, 0.0, 1.0)
    np.testing.assert_array_equal(
        observation.quaternion_world_wxyz_history, np.tile((1, 0, 0, 0), (5, 1))
    )
    with pytest.raises(ValueError):
        observation.quaternion_world_wxyz_history[0, 0] = 0.0


def _checkpoint_policy(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, payload) -> WorldModelPolicy:
    import benchmark.world_model_policy as module

    checkpoint = tmp_path / "frame.ckpt"
    torch.save(payload, checkpoint)
    # The model architecture is irrelevant to checkpoint input-contract validation.
    monkeypatch.setattr(module, "build_joint_model", lambda _config: torch.nn.Identity())
    protocol = ActionProtocol("ball_action_v2_aux30", (-2.0, -0.5235987756), (2.0, 0.5235987756), 5)
    return WorldModelPolicy(protocol, checkpoint=checkpoint, device="cpu")


def test_legacy_checkpoint_missing_velocity_frame_keeps_world_inputs(tmp_path, monkeypatch):
    policy = _checkpoint_policy(tmp_path, monkeypatch, {
        "action_protocol_id": "ball_action_v2_aux30", "model": {},
        "config": {"pose_head_enabled": True, "image_size": 8},
    })
    observation = _observation()
    context = policy._model_context(observation)
    np.testing.assert_array_equal(context["proprio"][0], observation.proprio_history)
    assert policy.adapter_metadata["proprio_velocity_frame"] == "world"


def test_checkpoint_rejects_disagreeing_velocity_frame_metadata(tmp_path, monkeypatch):
    with pytest.raises(ValueError, match="proprio_velocity_frame|velocity frame"):
        _checkpoint_policy(tmp_path, monkeypatch, {
            "action_protocol_id": "ball_action_v2_aux30", "model": {},
            "config": {"pose_head_enabled": True, "proprio_velocity_frame": "world"},
            "proprio_velocity_frame": "base",
        })


def _capture_runtime(tmp_path: Path) -> IsaacBenchmarkRuntime:
    runtime = IsaacBenchmarkRuntime(IsaacRuntimeConfig(
        scene_root=tmp_path, contract_path=tmp_path / "contract.yaml", gui=False, record_video=False,
    ))
    runtime._clock_metadata = lambda: (0.0, 0, 0.0, 0)
    runtime._actual_positions = []
    attribute = SimpleNamespace(Set=lambda _value: None)
    runtime._actual_curve = SimpleNamespace(
        GetCurveVertexCountsAttr=lambda: attribute, GetPointsAttr=lambda: attribute,
    )
    runtime._Gf = SimpleNamespace(Vec3f=lambda *values: values)
    runtime._left_camera = runtime._right_camera = runtime._third_camera = object()
    runtime._get_rgb = lambda _camera: np.zeros((480, 640, 3), dtype=np.uint8)
    runtime._occupancy = SimpleNamespace(grid=np.zeros((2, 2)), global_origin_xy=np.zeros(2))
    runtime._contract = SimpleNamespace(bev=object())
    runtime._crop_global_occupancy = lambda *_args: np.zeros((2, 2))
    runtime._obstacle_clearance = lambda *_args: 1.0
    runtime._physical_contact = lambda: False
    quaternion = np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float32)
    runtime._robot = SimpleNamespace(
        quaternion=quaternion,
        get_world_pose=lambda: (np.asarray((0, 0, 0.45)), quaternion),
        get_joint_positions=lambda: np.asarray((0.1, 0.2, 0.3)),
        get_joint_velocities=lambda: np.asarray((0.4, 0.5, 0.6)),
        get_linear_velocity=lambda: np.asarray((1, 2, 3)),
        get_angular_velocity=lambda: np.asarray((4, 5, 6)),
    )
    return runtime


def test_capture_preserves_world_inputs_and_aligns_bounded_quaternion_history(tmp_path):
    runtime = _capture_runtime(tmp_path)
    initial = runtime._capture().observation
    np.testing.assert_array_equal(initial.quaternion_world_wxyz_history, np.tile((1, 0, 0, 0), (5, 1)))
    half = np.sqrt(0.5)
    runtime._robot.quaternion[:] = (half, 0, 0, half)
    second = runtime._capture().observation
    np.testing.assert_array_equal(second.quaternion_world_wxyz_history[:4], np.tile((1, 0, 0, 0), (4, 1)))
    np.testing.assert_allclose(second.quaternion_world_wxyz_history[-1], (half, 0, 0, half))
    np.testing.assert_array_equal(second.proprio_history[:, :6], np.tile((1, 2, 3, 4, 5, 6), (5, 1)))
    for _ in range(5):
        runtime._capture()
    assert len(runtime._history_proprio) == len(runtime._history_quaternion_world_wxyz) == 5
    np.testing.assert_array_equal(initial.quaternion_world_wxyz_history, np.tile((1, 0, 0, 0), (5, 1)))


def test_benchmark_base_context_matches_recorded_state_proprio(tmp_path):
    from training.ball_wm_dataset import _proprio17

    runtime = _capture_runtime(tmp_path)
    half = np.sqrt(0.5)
    runtime._robot.quaternion[:] = (half, half, 0, 0)
    runtime._previous_action[:] = (0.7, 0.8)
    observation = runtime._capture().observation
    _, quaternion = runtime._robot.get_world_pose()
    states = {
        "quat": quaternion[None].copy(), "lin_vel": runtime._robot.get_linear_velocity()[None],
        "ang_vel": runtime._robot.get_angular_velocity()[None],
        "joint_pos": runtime._robot.get_joint_positions()[None],
        "joint_vel": runtime._robot.get_joint_velocities()[None],
        "applied_action": runtime._previous_action[None].copy(),
    }
    recorded = _proprio17(states, 0, velocity_frame="base")
    context = _policy("base")._model_context(observation)
    np.testing.assert_allclose(context["proprio"][0, -1], recorded, atol=1e-6)
    np.testing.assert_allclose(recorded[:6], (1, 3, -2, 4, 6, -5), atol=1e-6)
