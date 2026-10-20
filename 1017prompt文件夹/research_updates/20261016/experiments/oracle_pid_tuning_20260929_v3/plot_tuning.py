from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from measurement import geometry,bank
h=Path(__file__).resolve().parent;old=h.parent/'oracle_bank_mapping_20260929_v2';result=json.loads((h/'tuning_assessment.json').read_text());tags=[r['tag'] for r in result['candidates']]
fig,axes=plt.subplots(2,2,figsize=(14,8),sharex=True)
for row,main in enumerate([1.,-1.]):
 for tag in ['baseline']+tags:
  p=old/f'validation_{main}' if tag=='baseline' else h/f'{tag}_{main}';z=np.load(p/'physics.npz');t=z['simulation_time_s'];v,k=geometry(z['position_world_xy']);b=np.degrees(bank(z['root_quaternion_world_wxyz']));mask=t>=4.5
  axes[row,0].plot(t[mask],k[mask]*np.sign(main),label=tag,linewidth=1);axes[row,1].plot(t[mask],b[mask],label=tag,linewidth=1)
 axes[row,0].set_ylabel(f'main={main}\nbody curvature [1/m]');axes[row,1].set_ylabel('physical bank [deg]')
 for ax in axes[row]:ax.grid(True);ax.legend(fontsize=8);ax.set_xlabel('simulation time [s]')
fig.tight_layout();fig.savefig(h/'tuning_transients.png',dpi=140)
