"""Plot saved predictions after sealing. No inference or decision input changes."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

H = Path(__file__).resolve().parent
D = H / 'mvs_hero_temporal_development_v2_api_scale_recovery'
O = H / 'qualified_hazard_navigation_run_v1'
fig, axes = plt.subplots(3, 2, figsize=(12, 14))
for row, record in enumerate(json.loads((D / 'records.json').read_text())):
    f = np.load(D / record['file'])
    errors = []
    for frame, uv in zip([record['frame'], *f['source_frames']],
                         [f['current_uv'], *f['past_uv']]):
        truth = np.load(O / 'offline_truth' / f'frame_{frame:03d}.npz')
        x, y = np.rint(uv).astype(int).T
        z = truth[record['eye'] + '_depth_m'][y, x]
        T = truth[record['eye'] + '_camera_usd_world']
        rays = np.c_[(x - 319.5) / 584.8888888888889,
                     (y - 239.5) / 584.8888888888889, np.ones(len(x))]
        with np.errstate(invalid='ignore'):
            w = T[:3, 3] + (rays * z[:, None]) @ (T[:3, :3] @ np.diag([1, -1, -1])).T
            errors.append(np.linalg.norm(w - f['world'], axis=1))
    error = np.stack(errors, axis=1)
    finite = np.isfinite(error).all(1)
    correct = finite & (error <= .10).all(1)
    colors = np.tile([1., .70, 0., .7], (len(correct), 1))
    colors[finite & ~correct] = [1., .1, .1, .7]
    colors[correct] = [.1, 1., .2, .7]
    ax = axes.flat[row]
    rgb = np.load(O / 'online_inputs' / f"frame_{record['frame']:03d}.npz")[record['eye'] + '_rgb']
    ax.imshow(rgb)
    ax.scatter(*f['current_uv'].T, c=colors, s=.3)
    ax.set_title(f"{record['time_s']:.1f}s {record['eye']}: {correct.mean():.1%} correct / {len(correct):,} queries")
    ax.axis('off')
fig.legend(handles=[Patch(color='lime', label='All 3 endpoints within 0.10m'),
                    Patch(color='red', label='Finite geometry error'),
                    Patch(color='orange', label='Insufficient renderer support')],
           loc='lower center', ncol=3)
fig.suptitle('Rejected Hero depth hypothesis: known poses, closed development data\n'
             'Labels are post-prediction diagnostics; original camera and lighting unchanged')
fig.tight_layout(rect=[0, .025, 1, .955])
path = D / 'three_endpoint_error_overlay_v1.png'
assert not path.exists()
fig.savefig(path, dpi=140)
print(path)
