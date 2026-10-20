"""Actual ordinary E0 states, independently scored, no policy cursor."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).parent
r=json.loads((H/'rolling_endpoint_navigation_e0_assessment_v1.json').read_text());fig,ax=plt.subplots(1,3,figsize=(15,4.5))
for arm,color in [('b0','tab:blue'),('paced','tab:orange')]:
 D=H/('rolling_endpoint_navigation_e0_'+arm+'_run_v1');e=json.loads((D/'task_episode.json').read_text())['actual_episode'];start=np.array(e['start_position_world_m'][:2]);goal=np.array(e['goal_position_world_m'][:2]);ss=json.loads((D/'states.json').read_text());end=r['arms'][arm]['scored_end_s'];ss=[s for s in ss if s['time_s']<=end+1e-6];t=np.array([s['time_s'] for s in ss]);xy=np.array([s['position'][:2] for s in ss]);v=np.array([np.linalg.norm(s['velocity'][:2]) for s in ss]);q=np.load(D/'requests.npz')['requests']
 ax[0].plot(xy[:,0],xy[:,1],color=color,label=arm+' '+r['arms'][arm]['outcome']);ax[1].plot(t,np.linalg.norm(xy-goal,axis=1),color=color,label=arm);ax[2].plot(t,v,color=color,label=arm+' actual XY speed');ax[2].step(np.arange(len(q))*.1,q[:,0],color=color,alpha=.35,label=arm+' main request')
ax[0].plot([start[0],goal[0]],[start[1],goal[1]],'k--',label='Common scoring axis');ax[0].scatter(*goal,marker='*',color='k');ax[0].set_aspect('equal');ax[0].set(xlabel='World X (m)',ylabel='World Y (m)');ax[1].axhline(.2,color='k',ls=':');ax[1].set(xlabel='Time (s)',ylabel='Actual goal distance (m)');ax[2].set(xlabel='Time (s)',ylabel='Speed (m/s) / main request')
for a in ax:a.grid(alpha=.2);a.legend(fontsize=8)
fig.suptitle('Rolling intermediate/final endpoint context: '+r['classification']+'; knownpose/current velocity TaskH');fig.tight_layout();p=H/'rolling_endpoint_navigation_e0_trajectory_v1.png';assert not p.exists();fig.savefig(p,dpi=150)
