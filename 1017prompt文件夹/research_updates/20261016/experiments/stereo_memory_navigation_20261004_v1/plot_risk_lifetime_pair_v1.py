"""Measured complete trajectories, fixed geometric evaluator; no cursor plot scoring."""
from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
fig,axes=plt.subplots(1,2,figsize=(11,5))
for arm,color in [('history','#3266bb'),('current','#df8734')]:
 D=H/('sgbm_risk_lifetime_e1_'+arm+'_run_v1');states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());ep=json.loads((D/'task_episode.json').read_text())['actual_episode']
 xy=np.array([s['position'][:2] for s in states]);t=np.array([s['time_s'] for s in states]);start=np.array(ep['start_position_world_m'][:2]);goal=np.array(ep['goal_position_world_m'][:2]);u=(goal-start)/np.linalg.norm(goal-start)
 axes[0].plot(xy[:,0],xy[:,1],label=arm,color=color);axes[0].scatter(*xy[-1],color=color,marker='x');axes[1].plot(t,(xy-start)@u,label=arm,color=color)
axes[0].plot([start[0],goal[0]],[start[1],goal[1]],'--',color='gray',label='fixed start-goal axis (evaluation only)');axes[0].scatter(*start,color='black',marker='o');axes[0].scatter(*goal,color='green',marker='*',s=120);axes[0].set_aspect('equal',adjustable='datalim');axes[0].set(xlabel='world x (m)',ylabel='world y (m)',title='Actual complete E1 trajectory')
axes[1].set(xlabel='actual simulation time (s)',ylabel='true signed net progress (m)',title='No path cursor based progress')
for a in axes:a.grid(alpha=.2);a.legend(fontsize=8)
fig.suptitle('SGBM risk lifetime ablation / TaskH known current pose / 35s')
fig.tight_layout();p=H/'sgbm_risk_lifetime_trajectory_v1.png';assert not p.exists();fig.savefig(p,dpi=160);print(p)
