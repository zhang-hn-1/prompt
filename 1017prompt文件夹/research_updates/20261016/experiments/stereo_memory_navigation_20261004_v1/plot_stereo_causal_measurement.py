"""Fixed, predeclared frames and pooled diagnostics; no cherry-picked examples."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from stereo_geometry import FOCAL, BASELINE, restore_native
from stereo_causal_measurement import visibility_partition

H = Path(__file__).resolve().parent


def main():
    result = json.loads((H/'stereo_causal_measurement.json').read_text())
    manifest = json.loads((H/'transaction_development_manifest.json').read_text())
    masks = np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
    patches = np.load(H/'stereo_causal_patch_measurements.npz')
    fig, axes = plt.subplots(2, 5, figsize=(17, 7), constrained_layout=True)
    palette = ['#eee', '#222', '#9ea3ad', '#aa77cc', '#e89038', '#eecc44', '#22aa88']
    labels = ['no depth', 'left body', 'outside right', 'right body', 'inconsistent depth', 'missing right depth', 'shared']
    for i, sequence in enumerate(['e1', 'hard_0_e1_0004']):
        row = next(r for r in manifest['samples'] if r['sequence'] == sequence and Path(r['rgb_file']).name == 'frame_010.npz')
        rgb = np.load(row['rgb_file']); truth = np.load(row['truth_file'])
        pred = np.load(H/'predictions_transaction_development/fast'/sequence/'frame_010.npz')
        visibility = visibility_partition(truth['left_depth_m'], truth['right_depth_m'], masks['left'], masks['right'], FOCAL*BASELINE)
        with np.errstate(divide='ignore', invalid='ignore'):
            raw = FOCAL*BASELINE/restore_native(pred['disparity_px'])
            error = abs(raw-truth['left_depth_m'])
        error[~np.isfinite(error)] = 1.
        axes[i, 0].imshow(rgb['left_rgb']); axes[i, 1].imshow(rgb['right_rgb'])
        axes[i, 2].imshow(visibility, cmap=ListedColormap(palette), vmin=0, vmax=6)
        image = axes[i, 3].imshow(error, vmin=0, vmax=.5, cmap='magma')
        axes[i, 4].imshow(pred['confidence'], cmap='gray', vmin=0, vmax=1)
        for j, title in enumerate(['Left RGB', 'Right RGB', 'Offline visibility label', 'Raw axial depth error (m)', 'Frozen final confidence']):
            axes[i, j].set_title(f'{sequence}\n{title}', fontsize=10); axes[i, j].axis('off')
    fig.colorbar(image, ax=list(axes[:, 3]), fraction=.04, pad=.01)
    fig.suptitle('Same archived frame 010; camera, predictions and old gates unchanged', fontsize=14)
    fig.savefig(H/'figures/stereo_causal_fixed_frames.png', dpi=140); plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), constrained_layout=True)
    for i, sequence in enumerate(['e1', 'hard_0_e1_0004']):
        body = result['groups'][sequence]['body']
        counts = body['visibility_counts']; axes[i, 0].bar(range(2, 7), [counts[str(j)]/body['left_external_n']*100 for j in range(2, 7)], color=palette[2:])
        axes[i, 0].set_xticks(range(2, 7), ['outside', 'body mask', 'inconsistent', 'missing', 'shared'], rotation=20)
        axes[i, 0].set_ylabel('% of left eligible body pixels'); axes[i, 0].set_ylim(0, 75)
        prefix = sequence+'__body__frame_010'
        costs = patches[prefix+'__costs']; valid = patches[prefix+'__true_measurable']
        # Midpoint in the fixed stratified sample list; if unsupported use the next measurable.
        sample = len(costs)//2
        candidates = np.flatnonzero(valid & (np.arange(len(costs)) >= sample))
        if len(candidates): sample = int(candidates[0])
        axes[i, 1].plot(costs[sample], lw=1)
        true_d = patches[prefix+'__true_disparity'][sample]
        axes[i, 1].axvline(true_d, color='green', label='offline true disparity')
        xy = patches[prefix+'__coordinates_xy'][sample]
        axes[i, 1].set_title(f'Fixed sample {sample}, rectified pixel {xy.tolist()}')
        axes[i, 1].set_xlabel('Disparity (px)'); axes[i, 1].set_ylabel('5x5 RGB SAD (0-255)'); axes[i, 1].legend(fontsize=8)
        texture = np.concatenate([patches[k] for k in patches.files if k.startswith(sequence+'__body__') and k.endswith('__texture_std')])
        axes[i, 2].hist(texture[np.isfinite(texture)], bins=np.linspace(0, 25, 26), color='#3477aa')
        axes[i, 2].axvline(5., ls='--', color='orange', label='predeclared descriptive bin')
        axes[i, 2].set_xlabel('5x5 grayscale texture standard deviation'); axes[i, 2].set_ylabel('Stratified patch count'); axes[i, 2].legend(fontsize=8)
        axes[i, 0].set_title(sequence+' shared visibility')
    fig.suptitle('Offline diagnostic: visibility loss and local matching ambiguity, not a new predictor')
    fig.savefig(H/'figures/stereo_causal_visibility_and_cost.png', dpi=140); plt.close(fig)
    print('SAVED 2 fixed diagnostics')


if __name__ == '__main__': main()
