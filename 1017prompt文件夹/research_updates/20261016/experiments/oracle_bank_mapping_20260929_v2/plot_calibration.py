from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from measurement import geometry,bank
h=Path(__file__).resolve().parent;rows=json.loads((h/'calibration_assessment.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(11,4))
for main in [.5,1.5,-.5,-1.5]:
 rr=sorted([r for r in rows if r['main']==main],key=lambda x:x['requested_bank_deg'])
 axes[0].plot([r['requested_bank_deg'] for r in rr],[r['actual_bank_deg'] for r in rr],'o-',label=f'main={main}')
 axes[1].plot([r['requested_bank_deg'] for r in rr],[r['curvature_body_1_m'] for r in rr],'o-',label=f'main={main}')
axes[0].plot([-22.5,22.5],[-22.5,22.5],'k--',alpha=.4)
for ax in axes:ax.set_xlabel('requested bank [deg]');ax.grid(True);ax.legend(fontsize=8)
axes[0].set_ylabel('measured bank [deg]');axes[1].set_ylabel('body-forward curvature [1/m]');fig.tight_layout();fig.savefig(h/'calibration_map.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
for ax,p in zip(axes,sorted(h.glob('validation_*/physics.npz'))):
 z=np.load(p);t=z['simulation_time_s'];main=json.loads((p.parent/'config.json').read_text())['main'];v,k=geometry(z['position_world_xy']);ax.plot(t,k*np.sign(main),label='measured body curvature',linewidth=1)
 ax.plot([0,5,5,20,20,35,35,50,50,65],[0,0,.4,.4,-.4,-.4,.7,.7,-.7,-.7],'k--',label='requested');ax.set_ylabel(f'main={main}\ncurvature [1/m]');ax.grid(True);ax.legend()
axes[-1].set_xlabel('simulation time [s]');fig.tight_layout();fig.savefig(h/'validation_curvature.png',dpi=140)
