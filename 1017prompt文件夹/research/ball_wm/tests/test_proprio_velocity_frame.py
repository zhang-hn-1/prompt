"""Behavior tests for the versioned world/base velocity boundary."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from training.ball_wm_dataset import BallWMDataset, _proprio17
from training.joint_jepa import JointModelConfig
from test_ball_wm_dataset import _write_synthetic_episode


def measured_state(quaternion):
    return {
        'quat': np.array([quaternion], dtype=np.float32),
        'lin_vel': np.array([[1, 2, 3]], dtype=np.float32),
        'ang_vel': np.array([[4, 5, 6]], dtype=np.float32),
        'joint_pos': np.array([[.1, .2, .3]], dtype=np.float32),
        'joint_vel': np.array([[.4, .5, .6]], dtype=np.float32),
        'applied_action': np.array([[.7, .8]], dtype=np.float32),
    }


@pytest.mark.parametrize('quaternion,expected', [
    ([1, 0, 0, 0], [1, 2, 3, 4, 5, 6]),
    ([2**-.5, 0, 0, 2**-.5], [2, -1, 3, 5, -4, 6]),
    ([2**-.5, 2**-.5, 0, 0], [1, 3, -2, 4, 6, -5]),
    ([2**-.5, 0, 2**-.5, 0], [-3, 2, 1, -6, 5, 4]),
])
def test_measured_velocity_uses_full_current_base_rotation(quaternion, expected):
    states = measured_state(quaternion)
    legacy = _proprio17(states, 0)
    corrected = _proprio17(states, 0, velocity_frame='base')
    np.testing.assert_allclose(corrected[:6], expected, atol=1e-6)
    np.testing.assert_array_equal(corrected[6:], legacy[6:])
    np.testing.assert_array_equal(states['lin_vel'][0], [1, 2, 3])


def test_global_yaw_rotation_preserves_corrected_proprio():
    original = measured_state([1, 0, 0, 0])
    rotated = measured_state([2**-.5, 0, 0, 2**-.5])
    rotated['lin_vel'][0] = [-2, 1, 3]
    rotated['ang_vel'][0] = [-5, 4, 6]
    np.testing.assert_allclose(
        _proprio17(original, 0, velocity_frame='base'),
        _proprio17(rotated, 0, velocity_frame='base'), atol=1e-6,
    )
    assert not np.allclose(_proprio17(original, 0)[:6], _proprio17(rotated, 0)[:6])


def test_versioned_transform_uses_each_rows_quaternion_and_leaves_input_unchanged():
    from training.proprioception import transform_proprio_velocities
    values = np.tile(np.arange(17, dtype=np.float32), (2, 1))
    before = values.copy()
    quaternions = np.array([[1, 0, 0, 0], [2**-.5, 0, 0, 2**-.5]])
    result = transform_proprio_velocities(values, quaternions, frame='base')
    np.testing.assert_allclose(result[1, :6], [1, 0, 2, 4, -3, 5], atol=1e-6)
    np.testing.assert_array_equal(result[0], values[0])
    np.testing.assert_array_equal(result[:, 6:], values[:, 6:])
    np.testing.assert_array_equal(values, before)
    np.testing.assert_array_equal(transform_proprio_velocities(values, None), values)


def test_base_transform_normalizes_quaternion_and_rejects_missing_or_invalid_history():
    from training.proprioception import transform_proprio_velocities
    values = np.arange(17, dtype=np.float32)
    result = transform_proprio_velocities(values, [2, 0, 0, 2], frame='base')
    np.testing.assert_allclose(result[:6], [1, 0, 2, 4, -3, 5], atol=1e-6)
    for invalid in (None, [0, 0, 0, 0], [np.nan, 0, 0, 1], [[1, 0, 0, 0]]):
        with pytest.raises(ValueError, match='quaternion'):
            transform_proprio_velocities(values, invalid, frame='base')


def test_dataset_and_causal_evaluator_use_selected_frame_without_changing_targets(tmp_path):
    from scripts.evaluate_wm_diverse_prediction import CausalEvaluationDataset
    episode = _write_synthetic_episode(tmp_path, 'ep0000', 'scene', num_states=60, num_frames=12)
    with np.load(episode/'states.npz') as data:
        states = {k: data[k] for k in data.files}
    states['quat'][:] = [2**-.5, 0, 0, 2**-.5]
    states['lin_vel'][:] = [1, 2, 3]
    states['ang_vel'][:] = [4, 5, 6]
    np.savez_compressed(episode/'states.npz', **states)
    kwargs = dict(history_frames=5, rollout_horizon=5, image_size=(16,16), action_source='commanded_action')
    old = BallWMDataset(tmp_path, **kwargs)
    new = BallWMDataset(tmp_path, **kwargs, proprio_velocity_frame='base')
    old_sample, new_sample = old[0], new[0]
    torch.testing.assert_close(new_sample['proprio'][0,:6], torch.tensor([2.,-1.,3.,5.,-4.,6.]))
    for key in old_sample:
        if key != 'proprio':
            torch.testing.assert_close(new_sample[key], old_sample[key])
    causal = CausalEvaluationDataset(new, [0])[0]
    torch.testing.assert_close(causal['proprio'], new_sample['proprio'][:5])


def test_config_and_checkpoint_do_not_silently_reinterpret_old_weights():
    from training.proprioception import checkpoint_velocity_frame
    assert JointModelConfig().proprio_velocity_frame == 'world'
    assert checkpoint_velocity_frame({'config': {}}) == 'world'
    assert checkpoint_velocity_frame({'config': {'proprio_velocity_frame': 'base'}}) == 'base'
    with pytest.raises(ValueError, match='proprio_velocity_frame'):
        JointModelConfig(proprio_velocity_frame='camera')
    with pytest.raises(ValueError, match='proprio_velocity_frame'):
        checkpoint_velocity_frame({'config': {'proprio_velocity_frame': 'base'}, 'proprio_velocity_frame': 'world'})


def test_future_measured_attitudes_cannot_change_base_context(tmp_path):
    from scripts.evaluate_wm_diverse_prediction import CausalEvaluationDataset
    from training.ball_wm_dataset import _load_episode
    episode = _write_synthetic_episode(tmp_path, 'ep0000', 'scene', num_states=60, num_frames=12)
    source = BallWMDataset(tmp_path, history_frames=5, rollout_horizon=5,
                          image_size=(16,16), action_source='commanded_action',
                          proprio_velocity_frame='base')
    evaluator = CausalEvaluationDataset(source, [0])
    before = evaluator[0]
    # Frame 4 ends at physical row 24. Everything below changes only future feedback.
    states = _load_episode(str(episode))['states']
    states['quat'][25:] = [0, 0, 0, 1]
    states['lin_vel'][25:] = [10, 20, 30]
    states['ang_vel'][25:] = [40, 50, 60]
    states['applied_action'][25:] = [-1, -.5]
    after = evaluator[0]
    for key in ('pixels_left','pixels_right','proprio','action','future_action'):
        torch.testing.assert_close(before[key], after[key])
    assert not torch.equal(before['future_relative_pose'], after['future_relative_pose'])


def test_training_loader_manifest_and_warm_start_respect_frame(tmp_path, monkeypatch):
    import training.train_joint as trainer
    for i, scene in enumerate(('a','b','c')):
        _write_synthetic_episode(tmp_path/'data', f'ep{i:04d}', scene)
    config = tmp_path/'config.yaml'
    config.write_text('history_size: 3\nrollout_horizon: 3\nproprio_velocity_frame: base\n')
    split = tmp_path/'split.json'; split.write_text(json.dumps({'train':['a'],'val':['b'],'test':['c']}))
    out = tmp_path/'out'
    monkeypatch.setattr(sys, 'argv', ['train_joint','--data_root',str(tmp_path/'data'), '--config',str(config),
        '--scene_split_manifest',str(split),'--out_dir',str(out),'--epochs','0','--batch_size','1','--device','cpu'])
    monkeypatch.setattr(trainer, 'build_joint_model', lambda _: torch.nn.Linear(1,1))
    trainer.main()
    manifest = json.loads((out/'run_manifest.json').read_text())
    assert manifest['proprio_velocity_frame'] == 'base'
    assert manifest['config']['proprio_velocity_frame'] == 'base'
    with pytest.raises(ValueError, match='proprio_velocity_frame'):
        trainer.require_initial_checkpoint_velocity_frame({'config':{}}, 'base')
