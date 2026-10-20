"""Compare three real cap-0.55 episodes with their frozen cap-0.75 peers."""

import hashlib
import json
from pathlib import Path

import numpy as np

from analyze_regimes import project


HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / 'oracle_feasible_route_20260929_v9'
PROTOCOL = 'ball_pointgoal_v1_2_contact_35s'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(directory, case):
    episode = directory / f'episode_{case}'
    metrics = json.loads((episode / 'metrics.json').read_text())
    environment = json.loads((episode / 'environment.json').read_text())
    manifest = json.loads((episode / 'manifest.json').read_text())
    entry = next(item for item in manifest['episodes'] if item['episode_id'] == case)
    states = np.load(episode / 'states.npz')
    actions = np.load(episode / 'actions.npz')
    audit = json.loads((episode / 'collision_audit.json').read_text())
    assert metrics['evaluation_protocol_id'] == environment['evaluation_protocol_id'] == PROTOCOL
    assert metrics['episode_budget_s'] == 35.0
    assert environment['scene_sha256'] == entry['scene_sha256']
    assert environment['occupancy_sha256'] == entry['occupancy_sha256']
    assert len(states['position_world_xy']) == len(actions['action_blocks']) + 1 == metrics['action_blocks'] + 1
    assert states['simulation_time_s'][-1] <= 35.0 + 1e-5
    assert np.array_equal(states['physical_contact'], states['collision'])
    assert all(item['available'] for item in audit['samples'])
    assert bool(metrics['success']) == (metrics['termination_reason'] == 'success')
    assert bool(metrics['collision']) == (metrics['termination_reason'] == 'collision')
    progress, error = project(states['position_world_xy'], states['kinodynamic_reference_pose'])
    return dict(
        termination=metrics['termination_reason'],
        time_s=float(states['simulation_time_s'][-1]),
        final_distance_m=float(metrics['final_distance_m']),
        physical_contacts=int(states['physical_contact'].sum()),
        actual_travel_m=float(metrics['path_length_m']),
        reference_length_m=float(np.linalg.norm(np.diff(states['kinodynamic_reference_pose'][:, :2], axis=0), axis=1).sum()),
        independent_reference_progress_m=float(progress[-1]),
        cross_track_rms_m=float(np.sqrt(np.mean(error ** 2))),
        cross_track_peak_m=float(error.max()),
        half_speed_blocks=int(np.sum(np.abs(states['policy_scheduled_main_command_rad_s']) < .75)),
        direction_transition_blocks=int(np.sum(states['policy_braking_state_code'] == 2)),
        reference_sha256=hashlib.sha256(states['kinodynamic_reference_pose'].tobytes()).hexdigest(),
        initial_state={key: states[key][0].tolist() for key in (
            'position_world_xy', 'yaw_world_rad', 'joint_position_rad',
            'joint_velocity_rad_s', 'linear_velocity_world_m_s',
            'angular_velocity_world_rad_s', 'applied_action')},
        scene_sha256=entry['scene_sha256'],
        occupancy_sha256=entry['occupancy_sha256'],
        actions_sha256=hashlib.sha256(actions['action_blocks'].tobytes()).hexdigest(),
    )


def main():
    originals = PREVIOUS / 'hybrid'
    candidate = HERE / 'hybrid'
    for file in originals.iterdir():
        if not file.is_file() or file.suffix not in ('.py', '.json'):
            continue
        counterpart = candidate / file.name
        assert counterpart.is_file(), file.name
        if file.name == 'time_planning.py':
            old = file.read_text()
            new = counterpart.read_text()
            assert new == old.replace('max_reference_curvature_1_m = 0.75', 'max_reference_curvature_1_m = 0.55')
        else:
            assert sha(file) == sha(counterpart), file.name
    identity = json.loads((HERE / 'source_identity.json').read_text())
    assert all(sha(Path(path)) == expected for path, expected in identity.items())
    output = {}
    for suffix in ('0001', '0002', '0004'):
        case = f'hard_0_e1_{suffix}'
        previous = read(PREVIOUS / f'navigation_hybrid_hard_{case}', case)
        trial = read(HERE / f'navigation_cap055_{case}_gpu_retry' if suffix == '0004'
                     else HERE / f'navigation_cap055_{case}_gpu', case)
        assert trial['initial_state'] == previous['initial_state']
        assert trial['scene_sha256'] == previous['scene_sha256']
        assert trial['occupancy_sha256'] == previous['occupancy_sha256']
        assert trial['reference_sha256'] != previous['reference_sha256']
        assert trial['actions_sha256'] != previous['actions_sha256']
        output[case] = dict(previous=previous, cap055=trial)
    (HERE / 'bounded_candidate_comparison.json').write_text(json.dumps(output, indent=2) + '\n')
    for case, result in output.items():
        p, t = result['previous'], result['cap055']
        print(case, p['termination'], round(p['final_distance_m'], 3), '->',
              t['termination'], round(t['final_distance_m'], 3),
              'contacts', t['physical_contacts'], 'half-speed',
              p['half_speed_blocks'], '->', t['half_speed_blocks'])


if __name__ == '__main__':
    main()
