from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;f,ax=plt.subplots(2,2,figsize=(12,7),constrained_layout=True)
for v,color in [('B0','#2876af'),('velocity','#d17b13')]:
 a=json.loads((H/f'velocity_goal_settling_{v}_assessment_v2.json').read_text());p=np.load(H/f'velocity_goal_settling_{v}_run_v2/physics.npz');k=a['nominal_goal_entry_frame'];t=np.array(a['time_series'])[k:]-a['nominal_goal_entry_time_s'];ax[0,0].plot(t,np.array(a['goal_distance_series'])[k:],color=color,label=v);ax[0,1].plot(t,np.array(a['speed_series'])[k:],color=color,label=v);s=5*k;pt=p['simulation_time_s'][s:]-a['nominal_goal_entry_time_s'];ax[1,0].plot(pt,p['raw_command_target'][s:,0],color=color,ls=':',alpha=.7);ax[1,0].plot(pt,p['applied_action'][s:,0],color=color,label=v+' applied main');ax[1,1].plot([x['time_s']-a['nominal_goal_entry_time_s'] for x in a['low_inside_count_trace']],[x['low_inside_count'] for x in a['low_inside_count_trace']],color=color,label=v)
ax[0,0].axhline(.2,color='gray',ls='--');ax[0,0].set(ylabel='Goal distance (m)',title='Candidate exits radius: peak .202711m');ax[0,1].axhline(.05,color='gray',ls='--');ax[0,1].set(ylabel='Actual XY speed (m/s)',title='Zero requests still have physical transients');ax[1,0].set(ylabel='Main target (rad/s)',title='Dotted raw request; solid applied main');ax[1,1].axhline(5,color='gray',ls='--');ax[1,1].set(ylabel='Consecutive low+inside samples',title='Counter resets on speed rebound or leaving goal')
for a in ax.ravel():a.set_xlabel('Seconds since nominal goal entry');a.legend();a.grid(alpha=.25)
f.suptitle('V2 diagnostic REJECTED: stricter measured stop-and-remain condition failed\nSame local reference/candidate; native settling within original35s; no formal navigation')
f.savefig(H/'velocity_goal_settling_actual_v2.png',dpi=150)
