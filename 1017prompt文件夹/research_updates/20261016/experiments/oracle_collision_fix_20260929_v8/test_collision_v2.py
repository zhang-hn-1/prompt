"""Versioned collision scoring, causal sampling, and failure audit regressions."""
from pathlib import Path
from types import SimpleNamespace
import pytest
from benchmark.isaac_runtime import IsaacBenchmarkRuntime, IsaacRuntimeConfig
from scripts.run_pointgoal_benchmark import _build_parser, _child_command, _evaluation_protocol_id

def runtime(values, mode='physical_contact_v2', available=True):
    r = object.__new__(IsaacBenchmarkRuntime)
    r.config = SimpleNamespace(collision_mode=mode)
    r._contact_available = available
    r._block_physical_contact = False
    r._collision_contact_samples = []
    r._clock_metadata = lambda : (0.02, 1, 0.02, 1)
    it = iter(values)
    r._physical_contact = lambda : next(it)
    return r

def test_negative_map_without_contact_is_not_collision():
    r = runtime([False])
    assert r._collision_state(-0.04) == (False, False)

def test_positive_map_with_contact_is_collision():
    r = runtime([True])
    assert r._collision_state(0.024) == (True, True)

def test_transient_contact_latched_until_capture():
    r = runtime([False, True, False, False, False, False])
    for _ in range(5):
        r._sample_collision_contact('physics')
    assert r._collision_state(0.1) == (True, True)
    assert len(r.collision_contact_samples) == 6

def test_unavailable_contact_is_error_not_safe():
    r = runtime([False], available=False)
    with pytest.raises(RuntimeError, match='contact'):
        r._collision_state(-0.04)

def test_legacy_preserves_map_rule():
    r = runtime([False], 'map_clearance_v1')
    assert r._collision_state(-0.04) == (False, True)
    r = runtime([True], 'map_clearance_v1')
    assert r._collision_state(0.02) == (True, False)

def test_cli_defaults_contact_and_child_propagates():
    args = _build_parser().parse_args(['--manifest', 'manifest.json', '--split', 'dev', '--policy', 'oracle', '--scene-root', '/tmp', '--run-root', '/tmp/new', '--headless'])
    assert args.collision_mode == 'physical_contact_v2'
    child = _child_command(args, 'case')
    assert child[child.index('--collision-mode') + 1] == 'physical_contact_v2'
    assert _evaluation_protocol_id(args) == 'ball_pointgoal_v1_2_contact_35s'
    args.collision_mode = 'map_clearance_v1'
    assert _evaluation_protocol_id(args) == 'ball_pointgoal_v1_1_35s'

def test_config_rejects_unknown_mode():
    with pytest.raises(ValueError, match='collision_mode'):
        IsaacRuntimeConfig(scene_root=Path('/tmp'), contract_path=Path('/tmp/a'), gui=False, record_video=False, collision_mode='bad')

def test_execute_samples_each_tick_and_clears_previous_block():
    import numpy as np
    r = runtime([False, True, False, False, False, False] + [False] * 6)
    r._episode = object()
    r.config.gui = False
    r._world = SimpleNamespace(step=lambda **kw: None, render=lambda : None)
    r._action_protocol = SimpleNamespace(controls_per_block=5, minimum=(-2.0, -0.5), maximum=(2.0, 0.5), clip_block=lambda x: x)
    r._action_smoother = SimpleNamespace(step=lambda target, *args: target)
    r._controller = SimpleNamespace(apply_action=lambda action: None)
    r._ArticulationAction = lambda **kw: kw
    r._history_actions = []
    r._capture = lambda : r._collision_state(0.1)
    assert r.execute(np.zeros(10, dtype=np.float32)) == (True, True)
    assert [x['source'] for x in r.collision_contact_samples] == ['physics'] * 5 + ['capture']
    assert r.execute(np.zeros(10, dtype=np.float32)) == (False, False)

def test_invalid_contact_tensor_raises_in_physical_mode():
    import numpy as np
    r = runtime([])
    del r._physical_contact
    r._contacts = SimpleNamespace(get_contact_force_matrix=lambda **kw: np.array([[[float('nan'), 0, 0]]]))
    r._contract = SimpleNamespace(schedule=SimpleNamespace(physics_hz=50))
    with pytest.raises(RuntimeError, match='contact'):
        r._collision_state(0.2)
    assert r.collision_contact_samples[-1]['available'] is False

@pytest.mark.parametrize('protocols', [['ball_pointgoal_v1_1_35s'] * 2, ['ball_pointgoal_v1_2_contact_35s'] * 2, ['ball_pointgoal_v1_1_35s', 'ball_pointgoal_v1_2_contact_35s']])
def test_summary_separates_collision_protocols(tmp_path, monkeypatch, protocols):
    import json
    from scripts import run_pointgoal_benchmark as e
    episodes = []
    for (i, p) in enumerate(protocols):
        ep = SimpleNamespace(episode_id=f'case{i}', scene_uri='scene.usd')
        episodes.append(ep)
        d = tmp_path / f'episode_case{i}'
        d.mkdir()
        (d / 'metrics.json').write_text(json.dumps(dict(evaluation_protocol_id=p, episode_budget_s=35.0, success=0.0, collision=0.0, timeout=1.0, runtime_error=0.0, spl=0.0)))
    monkeypatch.setattr(e, 'summarize_navigation', lambda *a, **kw: {'num_eval': 2})
    if len(set(protocols)) > 1:
        with pytest.raises(ValueError, match='matching'):
            e._write_summary(tmp_path, episodes)
        assert not (tmp_path / 'summary.json').exists()
    else:
        result = json.loads(e._write_summary(tmp_path, episodes).read_text())
        assert result['evaluation_protocol_id'] == protocols[0]

@pytest.mark.parametrize('sample_source', ['capture', 'physics'])
def test_sensor_failure_audit_saved_before_close(tmp_path, monkeypatch, sample_source):
    import json
    import numpy as np
    from scripts import run_pointgoal_benchmark as e
    manifest = tmp_path / 'manifest.json'
    manifest.write_text('{}')
    args = e._build_parser().parse_args(['--manifest', str(manifest), '--split', 'dev', '--policy', 'oracle', '--scene-root', str(tmp_path), '--run-root', str(tmp_path / 'run'), '--headless'])
    episode = SimpleNamespace(episode_id='sensor_failure', seed=1)

    class FailedRuntime:

        def __init__(self, config):
            self.contact_measurement_available = False
            self.collision_contact_samples = [dict(source=sample_source, available=False, contact=False)]

        def close(self, return_code=0):
            audit = args.run_root / 'episode_sensor_failure.partial/collision_audit.json'
            assert audit.is_file()
            raise RuntimeError('native close must not mask original error')
    monkeypatch.setattr(e, 'load_action_protocol', lambda *a: SimpleNamespace(protocol_id='ball_action_v2_aux30'))
    monkeypatch.setattr(e, '_resolve_occupancy_path', lambda *a: tmp_path / 'map.npz')
    monkeypatch.setattr(e, 'IsaacBenchmarkRuntime', FailedRuntime)
    monkeypatch.setattr(e, '_make_policy', lambda *a, **kw: object())

    def failed_episode(*a, **kw):
        if sample_source == 'physics':
            raise RuntimeError('obstacle contact measurement unavailable')
        return SimpleNamespace(positions_world_xy=np.empty((0, 2)), runtime_error='obstacle contact measurement unavailable', termination_reason='runtime_error')
    monkeypatch.setattr(e, 'run_episode', failed_episode)
    with pytest.raises(RuntimeError, match='contact measurement unavailable'):
        e._run_one(args, episode)
    audit = json.loads((args.run_root / 'episode_sensor_failure.partial/collision_audit.json').read_text())
    assert audit['evaluation_protocol_id'] == 'ball_pointgoal_v1_2_contact_35s'
    assert audit['contact_measurement_available'] is False
    assert audit['samples'][0]['source'] == sample_source
    assert not (args.run_root / 'episode_sensor_failure').exists()
