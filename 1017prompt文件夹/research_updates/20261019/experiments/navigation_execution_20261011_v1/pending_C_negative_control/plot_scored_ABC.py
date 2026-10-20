"""Post-score presentation only: saved arrays, fixed midpoint and fixed map times."""
from pathlib import Path
import json, hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

D = Path(__file__).resolve().parent
read = lambda p: json.loads(Path(p).read_text())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
comparison = read(D / 'ABC_comparison.json')
assert comparison['complete'] and read(D / 'score_recovery_status.json')['child_returncode'] == 0
roots = {arm: Path(data['root']) for arm, data in comparison['arms'].items()}
results = {arm: read(root / 'e1_result.json') for arm, root in roots.items()}
colors = {'A': '#1f77b4', 'B': '#ff7f0e', 'C': '#2ca02c'}
names = ['front_left', 'front_right', 'rear_left', 'rear_right']

# Fixed midpoint of181 exposures, selected by sequence index, not error.
with np.load(D / 'e1_run/frames/frame_090.npz') as raw, np.load(D / 'e1_run/predictions/frame_090.npz') as pred:
    fig, axes = plt.subplots(4, 5, figsize=(16, 10))
    cm = plt.get_cmap('viridis').copy(); cm.set_bad('#999999')
    ec = plt.get_cmap('magma').copy(); ec.set_bad('#999999')
    for row, eye in enumerate(names):
        depth = pred[eye + '_depth'].copy()
        valid = pred[eye + '_valid'] & np.isfinite(depth) & (depth >= .5) & (depth <= 3)
        depth[~valid] = np.nan
        truth = raw[eye + '_depth_label'].copy()
        truth[~np.isfinite(truth) | (truth <= 0)] = np.nan
        error = np.abs(depth - truth)
        for col, (im, title, cmap, lo, hi) in enumerate([
            (raw[eye + '_rgb'], 'RGB', None, None, None),
            (depth, 'pred axial m', cm, 0, 3),
            (truth, 'GT axial m, score only', cm, 0, 3),
            (error, 'valid-overlap |error| m', ec, 0, .5)]):
            ax = axes[row, col]
            kw = {} if cmap is None else dict(cmap=cmap, vmin=lo, vmax=hi)
            ax.imshow(im, **kw); ax.set_title(eye + ': ' + title, fontsize=8); ax.axis('off')
        world = pred[eye + '_world']
        ax = axes[row, 4]; ax.scatter(world[:, 0], world[:, 1], s=.4)
        body = pred['body_estimated'][:3, 3]
        ax.plot(body[0], body[1], 'rx'); ax.set_xlim(-4.5, 4.5); ax.set_ylim(-3, 3)
        ax.set_aspect('equal'); ax.set_title('eye risk proxy XY / m; no semantics', fontsize=8)
    fig.suptitle('Actual C / fixed exposure90 (~45s): four RGB, prediction, sealed GT, error, per-eye BEV\nGray = invalid / unknown; depth display clipped at3m, error at0.5m; no RGB semantic model', fontsize=10)
    fig.tight_layout(); fig.savefig(D / 'C_fixed_midpoint_four_eyes.png', dpi=130); plt.close(fig)

fig, axes = plt.subplots(2, 3, figsize=(15, 8))
for arm, root in roots.items():
    result = results[arm]; item = comparison['arms'][arm]
    with np.load(root / 'e1_run/physics.npz') as physics:
        xy = physics['root_position_world_m'][:, :2]
    axes[0, 0].plot(xy[:, 0], xy[:, 1], color=colors[arm], label=arm)
    t = np.arange(len(result['false_map_cells_by_policy'])) * .1
    axes[0, 1].plot(t, result['false_map_cells_by_policy'], color=colors[arm], label=arm)
    support = read(root / 'supported_memory_result.json')
    st = np.array([r['time_s'] for r in support['rows']])
    den = np.array([r['blind_previously_source_supported_targets'] for r in support['rows']])
    miss = np.array([r['missing'] for r in support['rows']])
    ratio = np.divide(miss, den, out=np.full(len(miss), np.nan), where=den > 0)
    axes[0, 2].plot(st, ratio * 100, color=colors[arm], label=arm)
for ax in axes[0]: ax.legend(); ax.grid(alpha=.2)
axes[0, 0].set_title('Actual trajectories / m; one run each'); axes[0, 0].set_aspect('equal')
axes[0, 1].set_title('False retained risk cells / complete900 actions'); axes[0, 1].set_xlabel('simulation seconds')
axes[0, 2].set_title('Source-supported blind missing / conditional %'); axes[0, 2].set_xlabel('simulation seconds'); axes[0, 2].set_ylim(-2, 102)
for ax, key, title, scale in [
    (axes[1, 0], 'net_goal_progress_m', 'Net goal progress / m (higher better)', 1),
    (axes[1, 1], 'false_map_cell_seconds', 'False retained risk / cell seconds', 1),
    (axes[1, 2], 'source_supported_blind_missing_fraction', 'Blind missing / % (lower better)', 100)]:
    values = [comparison['arms'][a][key] * scale for a in 'ABC']
    ax.bar(list('ABC'), values, color=[colors[a] for a in 'ABC'])
    for i, v in enumerate(values): ax.text(i, v, f'{v:.3g}', ha='center', va='bottom')
    ax.set_title(title); ax.margins(y=.18)
fig.suptitle('A current-only / B existing memory / C original view-support memory\nAll90s timeouts, contact0; divergent RGB/trajectories, development comparison only. C fails progress and blind-loss gates.', fontsize=11)
fig.tight_layout(); fig.savefig(D / 'ABC_scored_development_comparison.png', dpi=135); plt.close(fig)

fig, axes = plt.subplots(3, 3, figsize=(12, 10))
config = read(D / 'protocol.json')
with np.load(config['occupancy']) as occupancy:
    reference = np.argwhere(occupancy['grid'] > 0) * float(occupancy['resolution_m']) + occupancy['global_origin_xy']
for col, arm in enumerate('ABC'):
    for row, frame in enumerate([0, 450, 899]):
        ax = axes[row, col]; ax.set_facecolor('#dddddd')
        ax.scatter(reference[:, 0], reference[:, 1], c='#999999', s=1, label='static GT reference')
        with np.load(roots[arm] / f'e1_run/maps/frame_{frame:03d}.npz') as m:
            xy = np.argwhere(m['labels'] == 1) * float(m['resolution']) + m['origin']
        ax.scatter(xy[:, 0], xy[:, 1], c='#cc3333', s=3, label='retained risk')
        with np.load(roots[arm] / 'e1_run/predictions/frame_090.npz') as p:
            if frame == 450:
                union = np.concatenate([p[n + '_world'][:, :2] for n in names])
                ax.scatter(union[:, 0], union[:, 1], s=.6, c='#0077bb', label='current four-eye union')
        ax.set_xlim(-4.5, 4.5); ax.set_ylim(-3, 3); ax.set_aspect('equal')
        ax.set_title(f'{arm}: fixed action{frame} / {frame * .1:.1f}s', fontsize=9)
axes[1, 2].legend(fontsize=7, loc='upper right')
fig.suptitle('Fixed0 /45 /89.9s joint BEV and retained maps; gray unknown, red risk, blue current union\nStatic GT reference is offline2D body-band proxy; no observed-free or3D safety certificate', fontsize=10)
fig.tight_layout(); fig.savefig(D / 'ABC_fixed_time_joint_and_long_memory.png', dpi=135); plt.close(fig)

denominators = {}
for arm, result in results.items():
    eyes = {}
    for eye in names:
        eyes[eye] = {key: sum(r['per_eye'][eye][key] for r in result['source_rows'])
                     for key in ['target_pixels', 'valid_target_pixels', 'risk_points']}
    denominators[arm] = dict(exposures=len(result['source_rows']), full_RGB_pixels=len(result['source_rows']) * 4 * 640 * 480,
                             target_pixels=sum(r['target_pixels'] for r in result['source_rows']), per_eye=eyes,
                             target_cell_exposures=result['source']['target_cells'], risk_cell_exposures=result['source']['predicted_cells'])
(D / 'presentation_receipt.json').write_text(json.dumps(dict(
    scope='Saved post-score visualization/denominator sums only; no source, model, policy, physics or scientific scorer rerun',
    fixed_C_exposure=90, fixed_policy_frames=[0, 450, 899], denominators=denominators,
    images={p.name: sha(p) for p in sorted(D.glob('*.png'))}, unknown='gray invalid; missing neverfree',
    semantics='N/A, fixed height risk proxy only', repeated_experiment_dispatch=0), indent=2) + '\n')
