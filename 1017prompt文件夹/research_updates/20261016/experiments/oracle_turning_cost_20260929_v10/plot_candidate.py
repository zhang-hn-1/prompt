"""Plot scored physical trajectories for the bounded curvature A/B."""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / 'oracle_feasible_route_20260929_v9'
fig, axes = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=True)
for ax, suffix in zip(axes, ('0001', '0002', '0004')):
    case = f'hard_0_e1_{suffix}'
    old = PREVIOUS / f'navigation_hybrid_hard_{case}' / f'episode_{case}'
    trial = HERE / f'navigation_cap055_{case}_gpu' if suffix != '0004' else HERE / f'navigation_cap055_{case}_gpu_retry'
    trial = trial / f'episode_{case}'
    for directory, color, label in ((old, '#276da0', 'Frozen 0.75 cap'),
                                    (trial, '#d15d28', 'Trial 0.55 cap')):
        states = np.load(directory / 'states.npz')
        metrics = json.loads((directory / 'metrics.json').read_text())
        xy = states['position_world_xy']
        ref = states['kinodynamic_reference_pose'][:, :2]
        ax.plot(ref[:, 0], ref[:, 1], '--', color=color, alpha=.5)
        ax.plot(xy[:, 0], xy[:, 1], color=color, linewidth=2,
                label=f'{label}: {metrics["termination_reason"]}, {metrics["final_distance_m"]:.2f} m')
    manifest = json.loads((trial / 'manifest.json').read_text())
    episode = next(item for item in manifest['episodes'] if item['episode_id'] == case)
    goal = episode['goal_position_world_m'][:2]
    ax.scatter(*goal, marker='*', s=120, color='black', label='Goal')
    ax.scatter(*np.load(old / 'states.npz')['position_world_xy'][0], s=40, color='#2b8a57', label='Start')
    ax.set_title(case)
    ax.set_xlabel('World X (m)')
    ax.set_ylabel('World Y (m)')
    ax.axis('equal')
    ax.grid(alpha=.25)
    ax.legend(fontsize=7)
fig.savefig(HERE / 'cap055_trajectories.png', dpi=160)
