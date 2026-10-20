"""Saved shared-prefix XY and current-index decision; no successor trajectory."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
a = json.loads((H / 'cusp_projection_assessment.json').read_text())
r = np.load(H / 'cusp_projection_case_reference.npz')
fig, ax = plt.subplots(figsize=(8, 6))
xy = r['pose'][:4, :2]
for i in range(3):
    direction = 'F' if r['direction'][i] > 0 else 'R'
    ax.plot(xy[i:i+2, 0], xy[i:i+2, 1], '-o', lw=2,
            color='royalblue' if direction == 'F' else 'orange', label=f'Known segment {i}: {direction}')
for i, point in enumerate(xy):
    ax.annotate(f'waypoint {i}', point, xytext=(8, -15), textcoords='offset points')
observed = np.array([row['current_position_xy'] for row in a['rows']])
ax.plot(observed[:, 0], observed[:, 1], 'ks--', label='Actual shared prefix 18.1–18.2s')
for idx, label, color, offset in [(3, 'Old selects 3; cancels R', 'crimson', (15, 22)),
                                  (1, 'Candidate holds 1; waits for original stop gate', 'darkgreen', (-70, 48))]:
    ax.annotate(label, xy[idx], xytext=offset, textcoords='offset points', color=color,
                arrowprops=dict(arrowstyle='->', color=color), fontsize=9)
ax.set(title='CPU current-action regression only — no candidate physical trajectory',
       xlabel='World X (m)', ylabel='World Y (m)')
ax.set_aspect('equal'); ax.grid(alpha=.25); ax.legend(fontsize=9, loc='lower left')
fig.tight_layout(); fig.savefig(H / 'cusp_projection_decision.png', dpi=160); plt.close(fig)
