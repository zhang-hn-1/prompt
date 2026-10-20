from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1');ref=np.load(h/'fixed_pursuit_reference_v1.npz')['pose'][:,:2]
f,ax=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
ax[0].plot(ref[:,0],ref[:,1],'k.--',label='Frozen local reference');circle=plt.Circle(ref[-1],.2,fill=False,color='gray');ax[0].add_patch(circle)
for v,color in [('B0','#2876af'),('velocity','#d17b13')]:
 a=json.loads((h/f'velocity_fixed_reference_{v}_assessment_v1.json').read_text());s=json.loads((h/f'velocity_fixed_reference_{v}_run_v1/states.json').read_text());xy=np.array([x['position'][:2] for x in s]);t=np.array([x['time_s'] for x in s]);k=a['handoff_frame'];ax[0].plot(xy[k:,0],xy[k:,1],color=color,label=v);ax[1].plot(t[k:],np.array(a['reference_error_series'])[k:],color=color,label=v);ax[2].plot(t[k:],np.array(a['reference_progress_series'])[k:],color=color,label=v)
ax[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Actual XY; local tracking only',aspect='equal');ax[1].set(xlabel='Simulation time (s)',ylabel='Nearest-reference distance (m)',title='Lateral distance');ax[2].set(xlabel='Simulation time (s)',ylabel='Reference arc progress (m)',title='Different nominal goal-entry end times')
for a in ax:a.grid(alpha=.25);a.legend()
f.suptitle('V1: original progress gate REJECTED; no post-goal stability data')
f.savefig(h/'velocity_fixed_reference_trajectory_v1.png',dpi=160)
