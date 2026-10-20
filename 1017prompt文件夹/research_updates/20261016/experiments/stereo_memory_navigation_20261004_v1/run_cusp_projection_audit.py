"""Two current-input calls until first divergence; no counterfactual physics labels."""
from pathlib import Path
import datetime
import hashlib
import json
import numpy as np
from cusp_projection import make_cusp_oracle
from oracle_rgb_shadow import make_oracle, reference_from_arrays
from depth_local_curve import install_local_reference, P
from benchmark.policies import PolicyObservation
from training.action_protocol import load_action_protocol

H = Path(__file__).resolve().parent
sha = lambda f: hashlib.sha256(Path(f).read_bytes()).hexdigest()


def arm(factory, protocol, case, ref):
    tracker = factory(protocol)
    length = float(np.linalg.norm(np.diff(ref['pose'][:, :2], axis=0), axis=1).sum())
    plan = reference_from_arrays(ref['pose'], ref['direction'], ref['command'], length, 1)
    install_local_reference(tracker, plan)
    names = dict(reference_index='_path_index', active_drive_direction='_active_drive_direction',
                 pending_drive_direction='_pending_drive_direction', stopped_cycles='_stopped_cycles',
                 off_reference_cycles='_off_reference_cycles', failure_subreason='_failure_subreason',
                 cross_track_error_m='_cross_track_error_m', desired_drive_direction='_desired_drive_direction',
                 braking_state='_braking_state')
    for key, attr in names.items():
        setattr(tracker, attr, case['before_diagnostics'][key])
    tracker.mapping_records = list(case['past_mapping_records'])
    return tracker


def main():
    protocol_file = H / 'cusp_projection_protocol.json'
    p = json.loads(protocol_file.read_text())
    for mapping in [p['source_sha256'], p['input_sha256']]:
        for f, digest in mapping.items():
            assert sha(f) == digest, f
    assert not (H / 'cusp_projection_assessment.json').exists(), 'sole audit already exists'
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    assert p['sealed_utc'] < started
    case = json.loads((H / 'cusp_projection_case_state.json').read_text())
    ref = np.load(H / 'cusp_projection_case_reference.npz')
    protocol = load_action_protocol(P / 'configs/data_contract.yaml')
    old = arm(make_oracle, protocol, case, ref)
    new = arm(make_cusp_oracle, protocol, case, ref)
    assert old.diagnostics == new.diagnostics == case['before_diagnostics']
    assert old.mapping_records == new.mapping_records
    np.testing.assert_equal(old._reference_xy, new._reference_xy)
    rows = []
    for frame in [181, 182]:
        inp = np.load(H / f'cusp_projection_case_input_{frame}.npz')
        # Saved request/expected_after are report-only labels, never arguments to act.
        obs = PolicyObservation(inp['left_rgb'], inp['right_rgb'], inp['proprio_history'],
                                inp['action_history'], inp['position_world_xy'],
                                float(inp['yaw_world_rad']), inp['quaternion_history'])
        results = []
        for name, t in [('archived_tracker', old), ('cusp_candidate', new)]:
            action = t.act(obs, inp['pointgoal_base_xy'])
            if name == 'archived_tracker':
                np.testing.assert_equal(action, inp['request'])
                assert t.diagnostics == case[f'expected_old_after_{frame}']
            results.append(dict(arm=name, protocol_request=action.tolist(),
                                diagnostics=t.diagnostics, request_layers=t.request_diagnostics))
        rows.append(dict(frame=frame, current_position_xy=inp['position_world_xy'].tolist(),
                         world_xy_speed_m_s=float(np.linalg.norm(inp['proprio_history'][-1, :2])),
                         arms=results))
        if frame == 181:
            np.testing.assert_equal(results[0]['protocol_request'], results[1]['protocol_request'])
            assert old.diagnostics == new.diagnostics
    assert old.path_index == 3 and new.path_index == 1
    assert old._pending_drive_direction is None and new._pending_drive_direction == 'reverse'
    assert old._active_drive_direction == new._active_drive_direction == 'forward'
    assert new._stopped_cycles == 0 and new._request_source == 'direction_change_braking'
    assert new.request_diagnostics['policy_raw_main_command_rad_s'] == 0.
    assert old.request_diagnostics['policy_raw_main_command_rad_s'] == .5
    result = dict(started_utc=started, assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  source_checks=len(p['source_sha256']), input_checks=len(p['input_sha256']),
                  independent_arm_calls=4, archived_tracker_requests_exact=2,
                  first_current_request_divergence_frame=182, same_prefix_call_181=True,
                  consumed_initial_policy_gating_state_equal=True, rows=rows,
                  projection_cusp_regression_pass=True, real_reverse_completion_demonstrated=False,
                  new_simulations=0, new_planner_searches=0, navigation_gate_unchanged_rejected=True,
                  future_physics_or_request_labels_used_as_inputs=False,
                  later_archived_trajectory_not_a_candidate_label=True,
                  actual_navigation_candidate_tested=False, whole_research_complete=False,
                  scope='CPU current-action reconstruction stops at first divergence; no physical successor state or success prediction.')
    (H / 'cusp_projection_assessment.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
