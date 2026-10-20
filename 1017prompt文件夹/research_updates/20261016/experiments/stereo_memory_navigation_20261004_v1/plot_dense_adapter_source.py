from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;folder=H/'dense_adapter_v2_predictions';assert (folder/'manifest.json').exists();audit=json.loads((folder/'assessment.json').read_text());trace=[json.loads(x) for x in (H/'dense_adapter_v2_training_trace.jsonl').read_text().splitlines()]
fig,ax=plt.subplots(1,2,figsize=(12,4));ax[0].scatter([r['attempt'] for r in trace],[r['loss'] for r in trace],s=7,c=[r['example'] for r in trace],cmap='tab20');ax[0].set_yscale('log');ax[0].set_xlabel('one fit: optimizer step');ax[0].set_ylabel('old-example loss (not fresh score)')
for arm in ('b0','adapter'):
 rows=[r for r in audit['rows'] if r['arm']==arm];ax[1].plot([r['time_s'] for r in rows],[np.nan if r['groups']['body_overlap']['complete_correct_fraction'] is None else r['groups']['body_overlap']['complete_correct_fraction'] for r in rows],label=arm)
ax[1].set_xlim(0,32);ax[1].axhline(.9,c='k',ls='--',label='frozen 90%');ax[1].set_xlabel('fresh whole sequence time(s)');ax[1].set_ylabel('all-query body overlap correct');ax[1].legend();fig.tight_layout();fig.savefig(H/'dense_adapter_fit_and_whole_validation.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(3,4,figsize=(15,8))
for r,i in enumerate((0,160,320)):
 rgb=np.load(H/'dense_adapter_heldout_rgb'/f'frame_{i:03d}.npz');truth=np.load(H/'dense_adapter_heldout_run/frames'/f'frame_{i:03d}.npz');axs[r,0].imshow(rgb['left_rgb']);axs[r,0].set_title(f'actual left frame{i}')
 for c,(name,z) in enumerate([('offline label',truth['left_depth_m']),('b0',np.load(folder/'b0'/f'frame_{i:03d}.npz')['depth_m']),('adapter',np.load(folder/'adapter'/f'frame_{i:03d}.npz')['depth_m'])],1):
  axs[r,c].imshow(np.ma.masked_invalid(np.where(np.isfinite(z)&(z<=3),z,np.nan)),vmin=.5,vmax=3,cmap='turbo');axs[r,c].set_title(name+': .5..3m, blank=missing')
for ax in axs.flat:ax.axis('off')
fig.tight_layout();fig.savefig(H/'dense_adapter_actual_rgb_depth.png',dpi=130);plt.close(fig)
