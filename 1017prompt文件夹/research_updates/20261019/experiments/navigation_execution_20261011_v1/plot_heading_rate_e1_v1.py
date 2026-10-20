from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/navigation_execution_20261011_v1')
assessment=json.loads((H/'heading_rate_e1_host_assessment_v1.json').read_text())
fig,axes=plt.subplots(1,3,figsize=(15,4.7))
for arm,label,color in [('b0','B0 world-z feedback','#4267b2'),('candidate','Projected heading-rate feedback','#d65b2a')]:
 r=H/f'heading_rate_e1_{arm}_run_v2';states=json.loads((r/'states.json').read_text());ep=json.loads((r/'task_episode.json').read_text())['actual_episode'];rows=json.loads((r/'policy_records.json').read_text());p=np.array([s['position'][:2] for s in states]);times=np.array([s['time_s'] for s in states]);times-=times[0]
 start=np.array(ep['start_position_world_m'][:2]);goal=np.array(ep['goal_position_world_m'][:2]);u=(goal-start)/np.linalg.norm(goal-start);v=np.array([-u[1],u[0]])
 axes[0].plot((p-start)@u,(p-start)@v,label=label,color=color);axes[0].scatter(((p-start)@u)[-1],((p-start)@v)[-1],color=color,marker='x')
 axes[1].plot(times,np.linalg.norm(p-goal,axis=1),label=label,color=color)
 valid=[(x['frame']*.1,x['heading_rate_tracking']) for x in rows if x.get('heading_rate_tracking') and x['heading_rate_tracking'].get('valid') and x['heading_rate_tracking']['source']=='tracking' and x['heading_rate_tracking'].get('heading_rate_geometry_supported')]
 axes[2].plot([t for t,d in valid],[d['measured_projected_heading_rate_rad_s']-d['measured_world_yaw_rate_rad_s'] for t,d in valid],color=color,label=label)
axes[0].scatter(0,0,color='black',label='Start',s=20);axes[0].axhline(0,color='grey',ls=':',lw=1);axes[0].set(xlabel='Along goal axis (m)',ylabel='Cross-track position (m)',title='Measured path near start');axes[0].axis('equal')
axes[1].axhline(.2,color='grey',ls='--',label='Arrival radius');axes[1].set(xlabel='Episode time (s)',ylabel='Distance to goal (m)',title='Original 35-second navigation rule')
axes[2].axhline(0,color='grey',lw=1);axes[2].set(xlabel='Episode time (s)',ylabel='Projected heading rate - world z (rad/s)',title='Current tracking inputs; no future labels')
for ax in axes:ax.grid(alpha=.25)
axes[0].legend(fontsize=8);axes[1].legend(fontsize=8)
fig.suptitle('Known-pose TaskH E1 development comparison: '+assessment['classification'],fontsize=12);fig.tight_layout();fig.savefig(H/'heading_rate_E1_comparison_v1.png',dpi=160);plt.close(fig)
print('plot saved')
