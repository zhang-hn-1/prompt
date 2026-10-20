"""Read scored v9 trajectories and measure where the 35-second budget goes."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / 'oracle_feasible_route_20260929_v9'
CASES = ('0001', '0002', '0003', '0004')


def project(xy: np.ndarray, reference: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    starts = reference[:-1, :2]
    edges = np.diff(reference[:, :2], axis=0)
    lengths = np.linalg.norm(edges, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    fraction = np.clip(
        np.sum((xy[:, None, :] - starts[None, :, :]) * edges[None, :, :], axis=2)
        / np.maximum(lengths[None, :] ** 2, 1e-12), 0.0, 1.0,
    )
    foot = starts[None, :, :] + fraction[:, :, None] * edges[None, :, :]
    error = np.linalg.norm(xy[:, None, :] - foot, axis=2)
    segment = np.argmin(error, axis=1)
    row = np.arange(len(xy))
    return cumulative[segment] + fraction[row, segment] * lengths[segment], error[row, segment]


def summarize(suffix: str) -> dict:
    case = f'hard_0_e1_{suffix}'
    directory = PREVIOUS / f'navigation_hybrid_hard_{case}' / f'episode_{case}'
    states = np.load(directory / 'states.npz')
    metrics = json.loads((directory / 'metrics.json').read_text())
    xy = states['position_world_xy']
    time_s = states['simulation_time_s']
    dt = np.diff(time_s)
    if not np.allclose(dt, 0.1, atol=1e-5):
        raise AssertionError(f'unexpected state interval in {case}')
    progress, cross_track = project(xy, states['kinodynamic_reference_pose'])
    progress_increment = np.diff(progress)
    distance_increment = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    speed = np.linalg.norm(states['linear_velocity_world_m_s'][:-1, :2], axis=1)
    code = states['policy_braking_state_code']
    main = np.abs(states['policy_scheduled_main_command_rad_s'])
    ref_curvature = np.abs(states['policy_reference_curvature_1_m'])
    desired_curvature = np.abs(states['policy_desired_curvature_1_m'])
    masks = {
        'direction_transition': code == 2,
        'goal_braking': code == 3,
        'tracking_main_0p5': (code == 1) & (np.abs(main - .5) < .1),
        'tracking_main_1p0': (code == 1) & (np.abs(main - 1.) < .1),
        'tracking_main_1p5': (code == 1) & (np.abs(main - 1.5) < .1),
    }
    assert np.sum(np.column_stack(list(masks.values())), axis=1).max() == 1
    assert np.sum(np.column_stack(list(masks.values())), axis=1).min() == 1
    regime = {}
    for name, mask in masks.items():
        regime[name] = dict(
            seconds=float(dt[mask].sum()),
            path_travel_m=float(distance_increment[mask].sum()),
            net_reference_progress_m=float(progress_increment[mask].sum()),
            mean_xy_speed_m_s=float(speed[mask].mean()) if np.any(mask) else None,
        )
    five_second_windows = []
    for t0 in np.arange(0., time_s[-1], 5.):
        left = int(round(t0 / .1))
        right = min(left + 50, len(dt))
        if right <= left:
            continue
        sl = slice(left, right)
        five_second_windows.append(dict(
            start_s=float(t0), end_s=float(time_s[right]),
            net_reference_progress_m=float(progress[right] - progress[left]),
            travel_m=float(distance_increment[sl].sum()),
            mean_speed_m_s=float(speed[sl].mean()),
            cross_track_rms_m=float(np.sqrt(np.mean(cross_track[sl] ** 2))),
        ))
    tracking_half = masks['tracking_main_0p5']
    last_10_start = max(0, len(dt) - 100)
    last_10_progress_m = float(progress[-1] - progress[last_10_start])
    remaining = float(np.linalg.norm(np.diff(states['kinodynamic_reference_pose'][:, :2], axis=0), axis=1).sum() - progress[-1])
    result = dict(
        case=case, termination=metrics['termination_reason'],
        episode_time_s=float(time_s[-1]), final_goal_distance_m=float(metrics['final_distance_m']),
        reference_length_m=float(np.linalg.norm(np.diff(states['kinodynamic_reference_pose'][:, :2], axis=0), axis=1).sum()),
        independent_reference_progress_m=float(progress[-1]),
        remaining_reference_m=remaining,
        actual_travel_m=float(distance_increment.sum()),
        cross_track_rms_m=float(np.sqrt(np.mean(cross_track ** 2))),
        cross_track_peak_m=float(cross_track.max()),
        reference_direction_transitions=int(np.sum(np.diff(states['kinodynamic_reference_drive_direction']) != 0)),
        half_speed_tracking_with_high_reference_curvature_s=float(dt[tracking_half & (ref_curvature >= .55)].sum()),
        half_speed_tracking_with_low_reference_curvature_s=float(dt[tracking_half & (ref_curvature < .55)].sum()),
        half_speed_tracking_with_high_desired_curvature_s=float(dt[tracking_half & (desired_curvature >= .55)].sum()),
        last_10_s_reference_progress_m=last_10_progress_m,
        extrapolated_additional_s_at_last_10_s_rate=(remaining / (last_10_progress_m / (len(dt) - last_10_start) * 10)) if last_10_progress_m > 0 else None,
        regime=regime,
        five_second_windows=five_second_windows,
    )
    return result, (time_s, progress, speed, cross_track, main, code)


def main() -> None:
    results = {}
    series = {}
    for suffix in CASES:
        result, values = summarize(suffix)
        results[result['case']] = result
        series[suffix] = values
    (HERE / 'regime_diagnostics.json').write_text(json.dumps(results, indent=2) + '\n')
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True, constrained_layout=True)
    for column, suffix in enumerate(('0002', '0004')):
        time_s, progress, speed, cross_track, main, code = series[suffix]
        axes[0, column].plot(time_s, progress, label='Measured reference progress')
        axes[0, column].axhline(results[f'hard_0_e1_{suffix}']['reference_length_m'],
                                color='k', linestyle='--', alpha=.5, label='Reference length')
        axes[0, column].set_title(f'E1 {suffix}: {results[f"hard_0_e1_{suffix}"]["termination"]}')
        axes[0, column].set_ylabel('Along-reference progress (m)')
        axes[0, column].legend(fontsize=8)
        axes[1, column].plot(time_s[:-1], speed, label='XY speed (m/s)')
        axes[1, column].plot(time_s[:-1], cross_track[:-1], label='Cross-track error (m)', alpha=.8)
        axes[1, column].plot(time_s[:-1], main / 3., label='Main command / 3', alpha=.65)
        axes[1, column].fill_between(time_s[:-1], 0, .8, where=code == 2,
                                     color='orange', alpha=.15, label='Direction transition')
        axes[1, column].set_xlabel('Simulation time (s)')
        axes[1, column].set_ylabel('Speed / error (m); scaled command')
        axes[1, column].legend(fontsize=7)
        for row in (0, 1):
            axes[row, column].grid(alpha=.25)
    fig.savefig(HERE / 'time_budget_0002_0004.png', dpi=160)
    for suffix in ('0002', '0004'):
        r = results[f'hard_0_e1_{suffix}']
        print(suffix, 'progress', round(r['independent_reference_progress_m'], 2),
              'of', round(r['reference_length_m'], 2), 'transition_s',
              r['regime']['direction_transition']['seconds'], 'half_s',
              r['regime']['tracking_main_0p5']['seconds'], 'remaining_s_at_recent_rate',
              round(r['extrapolated_additional_s_at_last_10_s_rate'], 1))


if __name__ == '__main__':
    main()
