from pathlib import Path
import json
import numpy as np
from roll_pid import bank_from_quaternion
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent;rows=[]
fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
for p in sorted(h.glob('probe_*/physics.npz')):
 z=np.load(p);t=np.arange(len(z['root_quaternion_world_wxyz']))*.02+.02
 b=np.degrees([bank_from_quaternion(q) for q in z['root_quaternion_world_wxyz']]);desired=np.where(t<=5,0,np.where(t<=15,15,np.where(t<=25,-15,0)))
 c=json.loads((p.parent/'config.json').read_text());windows=[]
 for start,end,target in [(12,15,15),(22,25,-15),(32,35,0)]:
  mask=(t>start)&(t<=end);x=b[mask];windows.append(dict(start_s=start,end_s=end,rms_error_deg=float(np.sqrt(np.mean((x-target)**2))),peak_error_deg=float(np.max(abs(x-target))),span_deg=float(np.ptp(x)),mean_bank_deg=float(x.mean()),rate_rms_deg_s=float(np.sqrt(np.mean(np.diff(x)**2))/.02)))
 row=dict(case=p.parent.name,main=c['main'],mode=c['mode'],gains=c['gains'],max_abs_bank_deg=float(max(abs(b))),windows=windows,tail_rms_deg=float(np.sqrt(np.mean([w['rms_error_deg']**2 for w in windows]))));rows.append(row)
 ax=axes[0 if c['main']==.5 else 1];ax.plot(t,b,label=p.parent.name);ax.set_ylabel('bank [deg]');ax.legend(fontsize=8);ax.grid(True)
for ax in axes:ax.plot([0,5,5,15,15,25,25,35],[0,0,15,15,-15,-15,0,0],'k--',alpha=.5)
axes[-1].set_xlabel('time [s]');fig.tight_layout();fig.savefig(h/'bank_probe.png',dpi=140)
(h/'probe_assessment.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
