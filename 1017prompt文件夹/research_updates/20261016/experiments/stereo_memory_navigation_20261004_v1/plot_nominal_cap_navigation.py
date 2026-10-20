from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'nominal_cap_history_run';a=json.loads((H/'nominal_cap_navigation_assessment.json').read_text());r=json.loads((O/'policy_records.json').read_text());sh=json.loads((O/'shadow_records.json').read_text());fig,axes=plt.subplots(2,2,figsize=(12,8))
for name,label in [('nominal_cap_history_run','Actual cap1.0'),('masked_memory_history_run','Actual baseline cap0.3')]:
 st=json.loads((H/name/'states.json').read_text());t=np.array([s['time_s'] for s in st]);xy=np.array([s['position'][:2] for s in st]);task=json.loads((H/name/'task_episode.json').read_text())['actual_episode'];goal=np.array(task['goal_position_world_m'][:2]);start=np.array(task['start_position_world_m'][:2]);u=(goal-start)/np.linalg.norm(goal-start);p=(xy-start)@u;cross=(xy-start)@np.array([-u[1],u[0]]);axes[0,0].plot(xy[:,0],xy[:,1],label=label);axes[0,1].plot(t,p,label=label);axes[1,0].plot(t,cross,label=label)
axes[0,0].scatter(*goal,marker='x',c='black');axes[0,0].axis('equal');axes[0,0].set_title('Actual trajectories: same initial state / two restarts');axes[0,1].set_title('Independent goal-line progress (m)');axes[1,0].set_title('Independent lateral error (m)');tt=np.arange(len(r))*.1;axes[1,1].plot(tt,[x['local']['request'][0] for x in r],label='Actual fast-policy main request');axes[1,1].plot(tt,[x['request'][0] for x in sh],ls='--',label='Same-feedback slow shadow: NEVER executed');axes[1,1].set_title('Same live source / parent control states equal');
for ax in axes.flat:ax.legend(fontsize=8);ax.set_xlabel('worldX m' if ax is axes[0,0] else 'simulation s');ax.grid(alpha=.2)
fig.suptitle('Local task reached in12.1s; frozen terminal no-regression FAILED (+2.35mm)');fig.tight_layout();fig.savefig(H/'nominal_cap_navigation_actual_and_shadow.png',dpi=140)
