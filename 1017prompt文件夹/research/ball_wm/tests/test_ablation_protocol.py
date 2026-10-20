import importlib
import pytest


def module():
    try:
        return importlib.import_module('training.ablation_protocol')
    except ModuleNotFoundError:
        pytest.fail('ablation protocol not implemented')


def test_split_rejects_leakage_and_unassigned_scenes():
    split = module().scene_indices
    assert split(['a', 'b', 'a', 'c'], {'train':['a'], 'val':['b'], 'test':['c']}) == ([0,2], [1], [3])
    with pytest.raises(ValueError):
        split(['a','b'], {'train':['a'], 'val':['a'], 'test':['b']})
    with pytest.raises(ValueError):
        split(['a','b','c','d'], {'train':['a'], 'val':['b'], 'test':['c']})


def test_budget_prefix_does_not_count_later_success_or_collision():
    clip = module().budget_episode
    episode = {'steps':3, 'termination_reason':'success', 'reference_length_m':1.,
               'initial_distance_m':1., 'initial_clearance_m':1.,
               'trajectory_xy':[[0,0],[.3,0],[.6,0],[.9,0]],
               'planner_trace':[{'step':i+1,'actual_distance_m':d,'actual_clearance_m':.5,
                                 'actual_collision':False} for i,d in enumerate([.7,.4,.1])]}
    early = clip(episode, 2, .2)
    assert early['termination_reason'] == 'timeout'
    assert early['path_length_m'] == pytest.approx(.6)
    assert early['final_distance_m'] == .4
    assert clip(episode,3,.2)['termination_reason'] == 'success'
    episode['planner_trace'][2]['actual_collision'] = True
    episode['termination_reason'] = 'collision'
    assert clip(episode,3,.2)['termination_reason'] == 'collision'
    assert clip(episode,2,.2)['termination_reason'] == 'timeout'
    episode['planner_trace'].pop()
    with pytest.raises(ValueError):
        clip(episode,3,.2)


def test_budget_runner_forwards_initial_clearance_requirement(monkeypatch, tmp_path):
    import json
    from training import run_bev_ablation as runner

    class CommandCaptured(Exception):
        pass

    manifest = {'require_initial_clearance': True, 'goal_offset': 20,
                'min_initial_goal_distance_m': .5, 'min_initial_clearance_m': .2}
    source = tmp_path / 'manifest.json'
    source.write_text(json.dumps(manifest))
    monkeypatch.setattr(runner, 'MANIFEST', source)
    def capture(command, log):
        assert ('--require_initial_clearance' in command) == manifest['require_initial_clearance']
        for flag, field in (('--goal_offset', 'goal_offset'),
                            ('--min_initial_goal_distance', 'min_initial_goal_distance_m'),
                            ('--min_initial_clearance', 'min_initial_clearance_m')):
            assert command[command.index(flag) + 1] == manifest[field]
        raise CommandCaptured

    monkeypatch.setattr(runner, 'child', capture)
    with pytest.raises(CommandCaptured):
        runner.navigation(runner.OLD, tmp_path, ['scene_easy_0'], .1)
