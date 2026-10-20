"""Verify v11 real-run comparisons and preserve the rejected candidate result."""

import hashlib
import json
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / 'oracle_feasible_route_20260929_v9'
PROTOCOL = 'ball_pointgoal_v1_2_contact_35s'


def checksum(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summarize(directory, case):
    episode = directory / f'episode_{case}'
    metrics = json.loads((episode / 'metrics.json').read_text())
    environment = json.loads((episode / 'environment.json').read_text())
    manifest = json.loads((episode / 'manifest.json').read_text())
    entry = next(item for item in manifest['episodes'] if item['episode_id'] == case)
    states = np.load(episode / 'states.npz')
    actions = np.load(episode / 'actions.npz')
    contact = json.loads((episode / 'collision_audit.json').read_text())
    assert metrics['evaluation_protocol_id'] == environment['evaluation_protocol_id'] == PROTOCOL
    assert metrics['episode_budget_s'] == 35.0
    assert environment['scene_sha256'] == entry['scene_sha256']
    assert environment['occupancy_sha256'] == entry['occupancy_sha256']
    assert len(states['position_world_xy']) == len(actions['action_blocks']) + 1 == metrics['action_blocks'] + 1
    assert states['simulation_time_s'][-1] <= 35.0 + 1e-5
    assert np.array_equal(states['physical_contact'], states['collision'])
    assert all(item['available'] for item in contact['samples'])
    assert bool(metrics['collision']) == (metrics['termination_reason'] == 'collision')
    assert bool(metrics['success']) == (metrics['termination_reason'] == 'success')
    reference = states['kinodynamic_reference_pose']
    starts = reference[:-1, :2]
    edges = np.diff(reference[:, :2], axis=0)
    lengths = np.linalg.norm(edges, axis=1)
    cumulative = np.r_[0., np.cumsum(lengths)]
    xy = states['position_world_xy']
    fraction = np.clip(np.sum((xy[:, None, :] - starts) * edges, axis=2)
                       / np.maximum(lengths[None, :] ** 2, 1e-12), 0., 1.)
    feet = starts + fraction[:, :, None] * edges
    distance = np.linalg.norm(xy[:, None, :] - feet, axis=2)
    ix = np.argmin(distance, axis=1)
    row = np.arange(len(xy))
    progress = cumulative[ix] + fraction[row, ix] * lengths[ix]
    error = distance[row, ix]
    return dict(
        termination=metrics['termination_reason'], time_s=float(states['simulation_time_s'][-1]),
        final_distance_m=float(metrics['final_distance_m']), physical_contacts=int(states['physical_contact'].sum()),
        reference_length_m=float(lengths.sum()), reference_progress_m=float(progress[-1]),
        cross_track_rms_m=float(np.sqrt(np.mean(error ** 2))),
        half_speed_blocks=int(np.sum(np.abs(states['policy_scheduled_main_command_rad_s']) < .75)),
        initial_state={key: states[key][0].tolist() for key in (
            'position_world_xy', 'yaw_world_rad', 'joint_position_rad', 'joint_velocity_rad_s',
            'linear_velocity_world_m_s', 'angular_velocity_world_rad_s', 'applied_action')},
        scene_sha256=entry['scene_sha256'], occupancy_sha256=entry['occupancy_sha256'],
        reference_sha256=hashlib.sha256(reference.tobytes()).hexdigest(),
        actions_sha256=hashlib.sha256(actions['action_blocks'].tobytes()).hexdigest(),
    )


def main():
    previous = PREVIOUS / 'hybrid'
    candidate = HERE / 'hybrid'
    for file in previous.iterdir():
        if file.is_file() and file.suffix in ('.py', '.json') and file.name != 'time_planning.py':
            assert checksum(file) == checksum(candidate / file.name), file.name
    core_identity = json.loads((PREVIOUS / 'hybrid_hard_source_identity.json').read_text())
    core = {path: sha for path, sha in core_identity.items() if '/research/ball_wm/' in path}
    assert all(checksum(Path(path)) == sha for path, sha in core.items())
    output = {}
    for suffix in ('0001', '0002', '0004'):
        case = f'hard_0_e1_{suffix}'
        if suffix == '0001':
            status = json.loads((HERE / 'status.json').read_text())
            assert all(json.loads((HERE / 'source_identity_after.json').read_text()).values())
        else:
            status = json.loads((HERE / f'{case}_status.json').read_text())
            assert all(json.loads((HERE / f'{case}_source_identity_after.json').read_text()).values())
        assert status['returncode'] == 0
        old = summarize(PREVIOUS / f'navigation_hybrid_hard_{case}', case)
        new_directory = HERE / f'navigation_runtime_cost_{case}'
        new = summarize(new_directory, case)
        assert old['initial_state'] == new['initial_state']
        assert old['scene_sha256'] == new['scene_sha256']
        assert old['occupancy_sha256'] == new['occupancy_sha256']
        assert old['reference_sha256'] != new['reference_sha256']
        assert old['actions_sha256'] != new['actions_sha256']
        decision = json.loads((new_directory / 'feedback/reference_decision.json').read_text())
        assert decision['selected_source'] == 'time_feasible_curvature_limited'
        output[case] = dict(frozen=old, runtime_cost=new, decision=decision)
    (HERE / 'comparison.json').write_text(json.dumps(output, indent=2) + '\n')
    for case, result in output.items():
        old, new = result['frozen'], result['runtime_cost']
        print(case, old['termination'], round(old['final_distance_m'], 3), '->',
              new['termination'], round(new['final_distance_m'], 3),
              'contacts', new['physical_contacts'])


if __name__ == '__main__':
    main()
