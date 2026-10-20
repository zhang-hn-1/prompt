"""One bounded offline causal audit of the archived online E1 run.

No policy.act calls, fitting, future actions, changed controls or new simulation.
Each map is rebuilt only from its current/past sensor packets. Frozen-path windows
include the next replan's physical observation without replacing their path.
"""
import json
import hashlib
from pathlib import Path
import numpy as np
from route_geometry_audit import project, dense_prefix, hazard_clearance, tracking_window
from provenance_grid import ProvenanceGrid
from motion_measurement import xy_curvature
from training.action_protocol import load_action_protocol
from benchmark.policies import geometric_tracker_action
from benchmark.grid_planning import astar_path
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
P = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
R = H/'online_bank_hold_navigation_run'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
rows = read(R/'vision_records.json'); states = read(R/'policy_states.json')
phys = np.load(R/'physics.npz'); xy = phys['position_world_xy'][::5]
time = phys['simulation_time_s'][::5]; vel = phys['linear_velocity_world_m_s'][::5, :2]
yaw = phys['yaw_world_rad'][::5]
ref = np.load(H/'online_provenance_evaluation_reference_e1.npz')['position_world_xy']
protocol = load_action_protocol(P/'configs/data_contract.yaml')
belief = ProvenanceGrid(xy[0]-20., memory_mode='history')
radius = float(.5+.14088407559444638+.1/np.sqrt(2))
details = []; source_files = [R/'physics.npz', R/'vision_records.json', R/'policy_states.json',
    H/'online_provenance_evaluation_reference_e1.npz', H/'reusable_oracle_audit.json',
    H/'route_geometry_audit.py', Path(__file__), P/'benchmark/policies.py', P/'configs/data_contract.yaml']
source_files += [H/n for n in ('provenance_grid.py', 'spatial_memory.py', 'risk_grid.py', 'motion_measurement.py')]
maps_exact = 0; targets_exact = 0; requests_exact = 0
route_compatibility_candidate = []
for i, row in enumerate(rows):
    packet_path = R/'privileged_sensor_packets'/f'frame_{i:03d}.npz'
    input_path = R/'online_inputs'/f'frame_{i:03d}.npz'
    source_files += [packet_path, input_path]
    with np.load(packet_path) as z:
        belief.update_points(z['points_world'], z['root_xyz'], float(z['time_s']))
    belief.seed_body(xy[i])
    snap = R/'belief_snapshots'/f'frame_{i:03d}.npz'
    if snap.exists():
        with np.load(snap) as z:
            for actual, key in [(belief.labels, 'labels'), (belief.memory.floor_time, 'floor_time'),
                                (belief.memory.hazard_time, 'hazard_time'), (belief.provenance(), 'provenance')]:
                np.testing.assert_array_equal(actual, z[key])
        maps_exact += 1; source_files.append(snap)
    with np.load(input_path) as z:
        np.testing.assert_array_equal(xy[i], z['position_world_xy'])
        assert yaw[i] == float(z['yaw_world_rad'])
    path = np.asarray(states[i]['path_xy'], float)
    near = int(np.linalg.norm(path-xy[i], axis=1).argmin())
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[near:], axis=0), axis=1))]
    index = min(len(path)-1, near+int(np.searchsorted(arc, 1.)))
    c, s = np.cos(yaw[i]), np.sin(yaw[i]); rot = np.array([[c, -s], [s, c]])
    target = rot.T@(path[index]-xy[i])
    np.testing.assert_array_equal(target, row['target_base_xy']); targets_exact += 1
    direction = row['direction']
    proposed = direction*geometric_tracker_action(protocol, direction*target) if direction else np.zeros(10, np.float32)
    if time[i] < 6.-1e-5:
        proposed[:] = 0
    else:
        proposed[::2] = np.clip(proposed[::2], -.3, .3)
    np.testing.assert_array_equal(proposed, row['proposed_request']); requests_exact += 1
    own_arc, own_error, tangent = project([xy[i]], path)
    prefix = dense_prefix(path[near:], 1.)
    # Bridge from actual centre to nearest path vertex, rather than hiding cell rounding.
    bridge = prefix if np.array_equal(prefix[0], xy[i]) else np.vstack((xy[i], prefix))
    hazard = belief.origin+np.argwhere(belief.labels == 1)*belief.resolution
    point_clear = hazard_clearance([xy[i]], hazard)
    path_clear = hazard_clearance(prefix, hazard)
    bridge_clear = hazard_clearance(bridge, hazard)
    chord_clear = hazard_clearance([xy[i], path[index]], hazard)
    # Exactly one offline candidate: synchronize planner point clearance with
    # the EXISTING stop-disc radius. All inputs are this frame's rebuilt map.
    # This does not claim actual alternative feedback or trajectories.
    if i in (60, 190, 200, 250, 280, 284, 340):
        start = belief.cells(xy[i:i+1])[0]; goal_cell = belief.cells(path[-1:])[0]
        clear_grid = distance_transform_edt(np.pad(belief.labels != 1, 1))[1:-1, 1:-1]*belief.resolution
        allowed = clear_grid >= radius-1e-8
        candidate = dict(frame=i, proposed_point_clearance_m=radius,
            current_actual_clearance_m=point_clear, start_grid_clearance_m=float(clear_grid[tuple(start)]),
            goal_grid_clearance_m=float(clear_grid[tuple(goal_cell)]),
            role='offline current-map feasibility ONLY; no alternative trajectory truth, no safety certification')
        if not allowed[tuple(start)] or not allowed[tuple(goal_cell)]:
            candidate.update(route_found=False, reason='current_start_or_goal_blocked')
        else:
            new_path = astar_path((~allowed).astype(np.uint8), start=tuple(start), goal=tuple(goal_cell))
            new_world = belief.origin+new_path*belief.resolution
            new_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(new_world, axis=0), axis=1))]
            new_idx = min(len(new_world)-1, int(np.searchsorted(new_arc, 1.)))
            new_prefix = dense_prefix(new_world, 1.)
            new_chord_clear = hazard_clearance([xy[i], new_world[new_idx]], hazard)
            candidate.update(route_found=True, reason='same_map_single_clearance_change',
                path_length_m=float(new_arc[-1]), first_metre_clearance_m=hazard_clearance(new_prefix, hazard),
                unchanged_tracker_chord_clearance_m=new_chord_clear,
                unchanged_tracker_chord_compatible=bool(new_chord_clear is None or new_chord_clear >= radius),
                candidate_path_world_xy=new_world.tolist(), candidate_target_world_xy=new_world[new_idx].tolist())
        route_compatibility_candidate.append(candidate)
    path_to_ref = project(prefix, ref)[1]
    chord = dense_prefix([xy[i], path[index]], float(np.linalg.norm(path[index]-xy[i])))
    shortcut_error = float(project(chord, path)[1].max())
    speed = float(np.linalg.norm(vel[i])); course = float(np.arctan2(vel[i, 1], vel[i, 0])) if speed >= .05 else None
    wrap = lambda v: float(np.arctan2(np.sin(v), np.cos(v)))
    tangent_angle = float(np.arctan2(tangent[0, 1], tangent[0, 0]))
    heading = float(np.arctan2(target[1], target[0]))
    if direction < 0: heading = wrap(heading-np.pi)
    details.append(dict(frame=i, time_s=float(time[i]), last_plan_s=states[i]['last_plan'],
        own_current_path_arc_m=float(own_arc[0]), own_current_path_error_m=float(own_error[0]),
        chosen_local_path_reference_rms_m=float(np.sqrt(np.mean(path_to_ref**2))),
        target_chord_max_deviation_from_own_path_m=shortcut_error,
        target_world_xy=path[index].tolist(), target_index=index, nearest_vertex=near,
        actual_xy_speed_m_s=speed, body_yaw_rad=float(yaw[i]), xy_course_rad=course,
        xy_course_minus_body_rad=None if course is None else wrap(course-yaw[i]),
        xy_course_minus_path_tangent_rad=None if course is None else wrap(course-tangent_angle),
        target_heading_rad=heading, geometric_heading_saturates_aux=bool(abs(heading) > protocol.maximum[1]+1e-7),
        requested_bank_reference_rad=float(row['proposed_request'][1]*.75),
        actual_bank_rad=float(phys['roll_world_rad'][i*5]),
        applied_aux_joint_target_rad=float(phys['applied_action'][i*5, 1]),
        current_hazard_clearance_m=point_clear, local_path_continuous_clearance_m=path_clear,
        path_with_current_bridge_clearance_m=bridge_clear, target_chord_clearance_m=chord_clear,
        local_path_compatible_with_unchanged_stop_radius=bool(path_clear is None or path_clear >= radius),
        target_chord_compatible_with_unchanged_stop_radius=bool(chord_clear is None or chord_clear >= radius),
        stop_veto=row['stopping_consumer']['stop_veto'],
        occupied_cells=row['stopping_consumer']['occupied_cells'],
        unknown_fraction=row['unknown_swept_support_fraction']))

replans = [i for i in range(len(rows)) if i == 0 or states[i]['last_plan'] != states[i-1]['last_plan']]
windows = []
for n, begin in enumerate(replans):
    end = replans[n+1] if n+1 < len(replans) else len(rows)
    path = np.asarray(states[begin]['path_xy'])
    result = tracking_window(xy[begin:end+1], path)
    result.update(begin_frame=begin, end_frame=end, begin_s=float(time[begin]), end_s=float(time[end]),
                  elapsed_s=float(time[end]-time[begin]))
    windows.append(result)

def stats(v):
    a = np.asarray(v, float); a = a[np.isfinite(a)]
    if not len(a): return None
    return dict(n=len(a), mean=float(a.mean()), rms=float(np.sqrt(np.mean(a*a))),
                median=float(np.median(a)), p10=float(np.percentile(a, 10)),
                p90=float(np.percentile(a, 90)), peak=float(np.max(np.abs(a))))

active = [v for v in details if 6.-1e-5 <= v['time_s'] < 28.4-1e-5]
active_windows = [w for w in windows if w['begin_frame'] >= 60 and w['end_frame'] <= 284]
periods = {}
for name, a, b in [('first_motion', 60, 150), ('late_motion', 150, 284), ('stopped', 284, 350)]:
    chosen = details[a:b]
    periods[name] = dict(frames=[a, b-1], n=len(chosen),
        own_current_path_error_m=stats([v['own_current_path_error_m'] for v in chosen]),
        chosen_local_path_reference_rms_m=stats([v['chosen_local_path_reference_rms_m'] for v in chosen]),
        chord_deviation_m=stats([v['target_chord_max_deviation_from_own_path_m'] for v in chosen]),
        speed_m_s=stats([v['actual_xy_speed_m_s'] for v in chosen]),
        course_minus_body_deg=stats([np.degrees(v['xy_course_minus_body_rad']) for v in chosen if v['xy_course_minus_body_rad'] is not None]),
        course_minus_path_tangent_deg=stats([np.degrees(v['xy_course_minus_path_tangent_rad']) for v in chosen if v['xy_course_minus_path_tangent_rad'] is not None]),
        heading_saturation_blocks=sum(v['geometric_heading_saturates_aux'] for v in chosen),
        local_route_stop_incompatibility_blocks=sum(not v['local_path_compatible_with_unchanged_stop_radius'] for v in chosen),
        target_chord_stop_incompatibility_blocks=sum(not v['target_chord_compatible_with_unchanged_stop_radius'] for v in chosen),
        veto_blocks=sum(v['stop_veto'] for v in chosen))

frozen_paths = {}
for begin in (60, 100, 150, 200, 250, 280):
    path = np.asarray(states[begin]['path_xy'])
    frozen_paths[str(begin)] = tracking_window(xy[begin:285], path)
    frozen_paths[str(begin)].update(scope='historical route counterfactual geometry ONLY; never supplied to controller; later observed hazards can invalidate old route')

k, course, good = xy_curvature(phys['simulation_time_s'], phys['position_world_xy'], phys['linear_velocity_world_m_s'][:, :2])
active_phys = (phys['simulation_time_s'] >= 6.-1e-5)&(phys['simulation_time_s'] < 28.4-1e-5)&good
root_proxy = phys['angular_velocity_world_rad_s'][:, 2]/np.maximum(np.linalg.norm(phys['linear_velocity_world_m_s'][:, :2], axis=1), .05)
bank_err = np.array([v['actual_bank_rad']-v['requested_bank_reference_rad'] for v in active])
selected = [details[i] for i in (60, 100, 150, 200, 250, 270, 280, 284, 298, 340, 349)]
old_scores = read(H/'reusable_oracle_audit.json')['rows']
old_e1 = next(v for v in old_scores if v['episode'] == 'episode_golden_e1_0001')
old_dir = Path(old_e1['root'])/old_e1['episode']
old_states = np.load(old_dir/'states.npz')
print('historical_state_fields', old_states.files)
old_xy = old_states['position_world_xy']
old_v = np.diff(old_xy, axis=0)/.1
old_action = np.load(old_dir/'actions.npz')
print('historical_action_fields', old_action.files)
source_files += [old_dir/'states.npz', old_dir/'actions.npz']
result = dict(scope='archived whole E1 privileged depth/true pose diagnostic; no new run/fit/control/gate changes',
    audit=dict(frames=len(rows), saved_maps_exact=maps_exact, target_reconstructions_exact=targets_exact,
               protocol_requests_exact=requests_exact, policy_act_calls=0,
               map_sources='current/past packet prefix only; no offline occupancy or Oracle reference input',
               stop_consumer_radius_m=radius,
               radius_certified=False, course_speed_min_m_s=.05, continuous_segment_clearance=True),
    active_window_summary=dict(n=len(active_windows),
        frozen_half_second_exit_error_m=stats([w['exit_error_m'] for w in active_windows]),
        frozen_half_second_progress_gain_m=stats([w['progress_gain_m'] for w in active_windows]),
        positive_progress_windows=sum(w['progress_gain_m'] > 0 for w in active_windows),
        sum_progress_gain_m=float(sum(w['progress_gain_m'] for w in active_windows)),
        caution='Each window is a different actual route; sum is diagnostic and not coherent total navigation progress. Current-path error resets with replanning.'),
    periods=periods, frozen_historical_route_geometry=frozen_paths, selected_rows=selected,
    motion=dict(actual_xy_curvature_m_inv=stats(k[active_phys]), root_world_z_rate_over_speed_m_inv=stats(root_proxy[active_phys]),
                their_difference_m_inv=stats(k[active_phys]-root_proxy[active_phys]),
                actual_bank_minus_requested_bank_deg=stats(np.degrees(bank_err)),
                curvature_scope='offline central derivative, thresholded neighbours; never an online input or certified steady physical envelope'),
    late_motion=dict(actual_xy_curvature_m_inv=stats(k[(phys['simulation_time_s'] >= 15.-1e-5)&active_phys]),
                     world_z_angular_rate_rad_s=stats(phys['angular_velocity_world_rad_s'][(phys['simulation_time_s'] >= 15.-1e-5)&active_phys, 2]),
                     bank_deg=stats(np.degrees(phys['roll_world_rad'][(phys['simulation_time_s'] >= 15.-1e-5)&active_phys]))),
    single_offline_route_compatibility_candidate=route_compatibility_candidate,
    old_successful_e1=dict(role='historical contextual measurement only; older runtime/rig and different full-map/route/controller',
        elapsed_s=old_e1['elapsed_s'], finite_difference_xy_speed_m_s=stats(np.linalg.norm(old_v, axis=1)),
        actual_xy_speed_m_s=stats(np.linalg.norm(old_states['linear_velocity_world_m_s'][:, :2], axis=1)),
        raw_main_request_rad_s=stats(old_action['controls'][:, 0]),
        sample_intervals_s=stats(np.diff(old_states['simulation_time_s'])),
        path_length_m=float(np.linalg.norm(np.diff(old_xy, axis=0), axis=1).sum()),
        camera_runtime_identity_verified=False),
    source_sha256={str(p):sha(p) for p in source_files})
(H/'online_route_execution_audit.json').write_text(json.dumps(result, indent=2)+'\n')
(H/'online_route_execution_rows.json').write_text(json.dumps(dict(rows=details, windows=windows), indent=2)+'\n')
fig, axes = plt.subplots(2, 2, figsize=(13, 9))
axes[0, 0].plot(ref[:, 0], ref[:, 1], ':', color='gray', label='independent evaluation reference')
for begin in (60, 150, 250, 280):
    path = np.asarray(states[begin]['path_xy'])
    prefix = dense_prefix(path, 1.4)
    axes[0, 0].plot(prefix[:, 0], prefix[:, 1], label=f'actual route {begin/10:g}s')
axes[0, 0].plot(xy[:, 0], xy[:, 1], color='black', label='actual robot')
axes[0, 0].set_aspect('equal'); axes[0, 0].legend(fontsize=8)
axes[0, 0].set_title('Online local routes differ from evaluation reference')
axes[0, 1].plot(time[:-1], [v['own_current_path_error_m'] for v in details], label='current path (replans reset)')
axes[0, 1].plot([w['end_s'] for w in windows], [w['exit_error_m'] for w in windows], '.', label='previous frozen window exit')
axes[0, 1].set_ylabel('cross-track error m'); axes[0, 1].legend(); axes[0, 1].set_title('Resetting paths can hide tracking error')
for key, label in [('current_hazard_clearance_m', 'current centre'), ('local_path_continuous_clearance_m', 'path next 1m'), ('target_chord_clearance_m', 'direct target chord')]:
    axes[1, 0].plot(time[:-1], [v[key] if v[key] is not None else np.nan for v in details], label=label)
axes[1, 0].axhline(radius, ls=':', color='red', label='unchanged stop radius (uncertified)')
axes[1, 0].axhline(.6, ls=':', color='gray', label='A* grid point rule')
axes[1, 0].set_ylim(.3, 2); axes[1, 0].set_ylabel('known hazard clearance m'); axes[1, 0].legend(fontsize=8)
axes[1, 1].plot(time[:-1], [np.degrees(v['target_heading_rad']) for v in details], label='target / body heading')
axes[1, 1].plot(time[:-1], [np.degrees(v['xy_course_minus_path_tangent_rad']) if v['xy_course_minus_path_tangent_rad'] is not None else np.nan for v in details], label='actual course / path tangent')
axes[1, 1].set_ylabel('degrees'); axes[1, 1].legend(fontsize=8)
for ax in axes.flat: ax.grid(alpha=.2)
for ax in (axes[0, 1], axes[1, 0], axes[1, 1]):
    ax.axvline(28.4, color='black', ls='--', alpha=.5); ax.set_xlabel('episode time s')
fig.tight_layout(); fig.savefig(H/'figures/online_route_execution_audit.png', dpi=150); plt.close(fig)
print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256', 'frozen_historical_route_geometry')}, indent=2))
