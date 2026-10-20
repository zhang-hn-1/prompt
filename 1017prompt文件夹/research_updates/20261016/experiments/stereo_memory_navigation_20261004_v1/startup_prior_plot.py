"""Posthoc figure only; reads saved diagnostic maps and OFFLINE actual sweep."""
from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;out=H/'startup_prior_runs';a=json.loads((out/'assessment.json').read_text());p=json.loads((H/'startup_prior_protocol.json').read_text());origin=np.array(p['origin']);pr=np.load(out/'immutable_prior.npz')['cells'];sweep=np.load(out/'offline_actual_sweep.npz');xy=sweep['xy'];outside=sweep['cells_outside_prior'];fig,axes=plt.subplots(1,3,figsize=(15,5))
for ax,frame in zip(axes,[60,160,320]):
 labels=np.load(out/f'map_{frame:03d}.npz')['labels'];g=np.argwhere(labels==0);h=np.argwhere(labels==1)
 for cells,color,label,size in [(outside,'lightgray','offline required outside prior',8),(pr,'lightblue','fixed initial TASK PRIOR',8),(h,'orange','visible hazard endpoints',18),(g,'green','visible floor endpoints',28)]:
  if len(cells):world=origin+cells*.1;ax.scatter(world[:,0],world[:,1],c=color,s=size,label=label)
 ax.plot(xy[:,0],xy[:,1],c='red',lw=1,label='actual future OFFLINE only');ax.set_title(f't={frame/10:g}s');ax.set_aspect('equal');ax.set_xlim(xy[:,0].min()-1,xy[:,0].max()+1);ax.set_ylim(xy[:,1].min()-1,xy[:,1].max()+1);ax.grid(alpha=.15)
axes[0].legend(fontsize=7);fig.suptitle('Perfect visible depth + fixed initial prior; blank remains unknown; no control/3D certificate');fig.tight_layout();fig.savefig(H/'startup_prior_actual_maps.png',dpi=145);plt.close(fig)
rows=a['rows'];t=[r['time_s'] for r in rows];fig,axes=plt.subplots(1,2,figsize=(12,4));axes[0].plot(t,[r['current_cold']['unknown_cells'] for r in rows],label='cold current footprint');axes[0].plot(t,[np.nan if r['current_prior'] is None else r['current_prior']['unknown_cells'] for r in rows],label='fixed prior current footprint');axes[0].set_ylabel('unknown cells');axes[0].legend();axes[1].plot(t,[r['outside_prior']['unknown_cells'] for r in rows],label='offline required outside prior');axes[1].axhline(313,color='gray',ls='--',label='313 required cells');axes[1].set_ylabel('unknown cells out of 313');axes[1].legend()
for ax in axes:ax.set_xlabel('sequence time (s)');ax.set_xlim(0,32);ax.grid(alpha=.2)
fig.tight_layout();fig.savefig(H/'startup_prior_full_series.png',dpi=145)
