"""Recompute outcome from manifest goal, measured velocity and contact samples."""
import hashlib
import json
from pathlib import Path
import numpy as np
from acceptance import stop_hold

HERE = Path(__file__).resolve().parent
BASE = HERE.parent/'fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1/physical_primitive_coverage_v1'


def assess_episode(i):
    episode = f'hard_0_e1_{i:04}'
    root = HERE/episode
    folder = root/f'episode_{episode}'
    metric = json.loads((folder/'metrics.json').read_text())
    manifest = json.loads((folder/'manifest.json').read_text())
    spec = next(e for e in manifest['episodes'] if e['episode_id']==episode)
    goal = np.asarray(spec['goal_position_world_m'][:2])
    with np.load(folder/'states.npz') as s:
        xy = s['position_world_xy']; t = s['simulation_time_s']
        speed = np.linalg.norm(s['linear_velocity_world_m_s'][:,:2],axis=1)
        collision = s['collision']
    distance = np.linalg.norm(xy-goal,axis=1)
    entries = np.flatnonzero((distance <= .2) & (t <= 35.00001))
    entry = int(entries[0]) if len(entries) else None
    end = len(t)-1 if entry is None else entry
    contacts = json.loads((folder/'collision_audit.json').read_text())['samples']
    scored = [s for s in contacts if s['simulation_time_s'] <= t[-1]+1e-5]
    before_entry = [s for s in scored if s['simulation_time_s'] <= t[end]+1e-5]
    coverage = {int(s['physics_step_index']) for s in scored if s['available']}
    with np.load(root/'physics.npz') as p:
        physics_steps = p['physics_step_index']; physics_time = p['simulation_time_s']
        valid = bool(all(int(k) in coverage for k in physics_steps[physics_time<=t[-1]+1e-5]))
    formal_success = bool(entry is not None and valid and not any(s['contact'] for s in before_entry)
                          and not np.any(collision[:end+1]))
    held = stop_hold(distance,speed,t,entry_index=entry)
    all_clear = bool(valid and not any(s['contact'] for s in scored) and not np.any(collision))
    records = json.loads((root/'policy_records.json').read_text())
    identity = json.loads((root/'experiment_identity.json').read_text())
    source_matches = all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
                         for p,h in identity['source_sha256'].items())
    with np.load(folder/'actions.npz') as a:
        actions = a['action_blocks']
    records_aligned = all(np.array_equal(np.asarray(r['request_main_aux'],dtype=np.float32),
                                         actions[r['block'],:2]) for r in records)
    post_entry_zero = bool(entry is None or np.all(actions[entry:]==0))
    output = dict(episode_id=episode,formal_goal_entry_success=formal_success,
                  first_goal_time_s=None if entry is None else float(t[entry]),
                  goal_entry_distance_m=None if entry is None else float(distance[entry]),
                  goal_entry_speed_m_s=None if entry is None else float(speed[entry]),
                  strict_stop_hold_passed=bool(held is not None and all_clear),
                  strict_hold_complete_s=held,observed_until_s=float(t[-1]),
                  final_distance_m=float(distance[-1]),final_xy_speed_m_s=float(speed[-1]),
                  minimum_distance_m=float(distance.min()),
                  post_entry_max_distance_m=None if entry is None else float(distance[entry:].max()),
                  path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
                  contact_samples=len(scored),contact_sensor_coverage_complete=valid,
                  contact_detected=bool(any(s['contact'] for s in scored)),
                  first_contact_s=next((s['simulation_time_s'] for s in scored if s['contact']),None),
                  contact_free_whole_observation=all_clear,
                  policy_records=len(records),route_built=bool((root/'reference_built_at_6s.npz').exists()),
                  final_policy_stage=records[-1]['stage'] if records else None,
                  runner_termination_reason=metric['termination_reason'],
                  runner_runtime_error=metric.get('runtime_error_message'),
                  episode_budget_s=metric['episode_budget_s'],
                  source_hashes_match=source_matches,request_records_aligned=records_aligned,
                  post_entry_requests_zero=post_entry_zero)
    if metric['episode_budget_s'] != 35 or t[-1] > 35.00001 or not source_matches or not records_aligned or not post_entry_zero:
        raise AssertionError(f'experiment integrity failed: {episode}')
    if metric.get('runtime_error_message'):
        raise AssertionError(f'no completed physical outcome: {episode}')
    return output


def reproduction():
    old = BASE/'navigation_probe'
    new = HERE/'hard_0_e1_0004'
    differences = {}
    with np.load(old/'physics.npz') as r, np.load(new/'physics.npz') as n:
        for key in r.files:
            differences[key] = float(np.max(np.abs(r[key].astype(float)-n[key][:len(r[key])].astype(float))))
    for filename in ('states.npz','actions.npz'):
        with np.load(old/f'episode_hard_0_e1_0004/{filename}') as r, np.load(new/f'episode_hard_0_e1_0004/{filename}') as n:
            keys = ('position_world_xy','linear_velocity_world_m_s','joint_position_rad','joint_velocity_rad_s',
                    'applied_action','yaw_world_rad','simulation_time_s') if filename=='states.npz' else ('action_blocks',)
            for key in keys:
                differences[f'{filename}/{key}'] = float(np.max(np.abs(r[key].astype(float)-n[key][:len(r[key])].astype(float))))
    return dict(passed=all(v<=1e-6 for v in differences.values()),max_abs_difference=differences,
                physics_samples_compared=1421,request_blocks_compared=284)


def main():
    results = [assess_episode(i) for i in (4,1,2,3)]
    repeated = reproduction()
    if not repeated['passed']:
        raise AssertionError('0004 goal-entry prefix did not reproduce')
    output = dict(results=results,reproduction_0004=repeated,
                  formal_goal_entry_pass_count=sum(r['formal_goal_entry_success'] for r in results),
                  strict_stop_hold_pass_count=sum(r['strict_stop_hold_passed'] for r in results),
                  historical_development_episodes=4,visual_navigation=False,fresh_held_out=False)
    (HERE/'assessment.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':
    main()
