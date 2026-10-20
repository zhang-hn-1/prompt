"""Measured frozen-reference comparison, never policy cursor or navigation claim."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
r=json.loads((H/'local_reserve_fixed_reference_assessment_v1.json').read_text())
ref=json.loads((H/'straight_pacing_fixed_reference_v1.json').read_text())
p=np.array([[x['x_m'],x['y_m']] for x in ref['waypoints']])
fig,ax=plt.subplots(1,3,figsize=(15,4.5))
ax[0].plot(p[:,0],p[:,1],'k--',label='Frozen reference')
for arm,color in [('b0','tab:blue'),('paced','tab:orange')]:
 D=H/('local_reserve_fixed_reference_'+arm+'_run_v1')
 ss=json.loads((D/'states.json').read_text());end=r['arms'][arm]['execution_closure']['scored_end_s']
 ss=[s for s in ss if s['time_s']<=end+1e-6]
 t=np.array([s['time_s'] for s in ss]);xy=np.array([s['position'][:2] for s in ss]);v=np.array([np.linalg.norm(s['velocity'][:2]) for s in ss]);q=np.load(D/'requests.npz')['requests']
 ax[0].plot(xy[:,0],xy[:,1],color=color,label=arm)
 ax[1].plot(t,np.linalg.norm(xy-p[-1],axis=1),color=color,label=arm)
 ax[2].plot(t,v,color=color,label=arm+' actual XY speed')
 ax[2].step(np.arange(len(q))*.1,q[:,0],color=color,alpha=.35,label=arm+' main request')
ax[0].set_aspect('equal');ax[0].set(xlabel='World X (m)',ylabel='World Y (m)')
ax[1].axhline(.2,color='k',ls=':');ax[1].set(xlabel='Time (s)',ylabel='Measured endpoint distance (m)')
ax[2].axhline(.05,color='k',ls=':');ax[2].set(xlabel='Time (s)',ylabel='Speed (m/s) / main request')
for a in ax:a.grid(alpha=.2);a.legend(fontsize=8)
fig.suptitle('Local reserve qualification: '+r['classification']+'; fixed reference only, no navigation admission')
fig.tight_layout();out=H/'local_reserve_fixed_reference_trajectory_v1.png';assert not out.exists();fig.savefig(out,dpi=150)
