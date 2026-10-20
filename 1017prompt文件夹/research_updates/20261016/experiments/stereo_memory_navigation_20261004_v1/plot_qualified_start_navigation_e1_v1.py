"""Post-close evaluation only; never imported by physical actor."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
for name,color in [('b0','tab:blue'),('candidate','tab:orange')]:
 d=H/('qualified_start_navigation_e1_'+name+'_run_v1');s=json.loads((d/'states.json').read_text());r=json.loads((d/'policy_records.json').read_text());t=np.array([v['time_s'] for v in s]);xy=np.array([v['position'][:2] for v in s]);vel=np.array([v['velocity'][:2] for v in s]);q=np.load(d/'requests.npz')['requests'];rt=np.array([v['time_s'] for v in r]);ep=json.loads((d/'task_episode.json').read_text())['actual_episode'];goal=np.array(ep['goal_position_world_m'][:2]);handoff=json.loads((d/'identity.json').read_text())['initial_qualification_time_s']
 axes[0,0].plot(xy[:,0],xy[:,1],color=color,label=name);axes[0,0].scatter(*xy[0],color=color,marker='o');axes[0,0].scatter(*xy[-1],color=color,marker='x');axes[0,0].scatter(*goal,color='green',marker='*',s=100)
 axes[0,1].plot(t,np.linalg.norm(vel,axis=1),color=color,label=name);axes[0,1].axvline(handoff,color=color,ls='--',alpha=.5)
 axes[1,0].step(rt,q[:,0],where='post',color=color,label=name);axes[1,1].plot(t,np.linalg.norm(xy-goal,axis=1),color=color,label=name)
axes[0,0].set(title='Actual world XY (TaskH known pose)',xlabel='world X (m)',ylabel='world Y (m)');axes[0,0].axis('equal')
axes[0,1].set(title='Actual XY speed / initial handoff',xlabel='episode time (s)',ylabel='m/s')
axes[1,0].set(title='Issued raw main request',xlabel='episode time (s)',ylabel='rad/s')
axes[1,1].set(title='True distance to goal',xlabel='episode time (s)',ylabel='m');axes[1,1].axhline(.2,color='green',ls=':')
for ax in axes.flat:ax.grid(alpha=.2);ax.legend()
fig.savefig(H/'qualified_start_navigation_e1_trajectory_v1.png',dpi=170)
