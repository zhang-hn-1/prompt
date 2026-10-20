from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
W=Path('/home/an/Documents/ChatGPT/球型机器人导航');H=W/'experiments/stereo_memory_navigation_20261004_v1'
assessment=json.loads((H/'dense_current_live_e1_assessment_v1.json').read_text())
fig,axs=plt.subplots(1,3,figsize=(13,4))
for arm,color in [('b0','#497ba2'),('candidate','#cf7236')]:
 d=H/f'dense_current_live_e1_{arm}_run_v1';ss=json.loads((d/'states.json').read_text());xy=np.array([s['position'][:2] for s in ss]);episode=json.loads((d/'task_episode.json').read_text())['actual_episode'];start=np.array(episode['start_position_world_m'][:2]);goal=np.array(episode['goal_position_world_m'][:2]);u=(goal-start)/np.linalg.norm(goal-start)
 axs[0].plot(xy[:,0],xy[:,1],color=color,label=arm)
 axs[1].plot([s['time_s'] for s in ss],(xy-start)@u,color=color,label=arm)
 counts=assessment['arms'][arm]['source_packet_contract_verified'];axs[2].plot([r['time_s'] for r in counts],[r['partial'] for r in counts],color=color,label=arm)
axs[0].scatter(*start,c='black',marker='o',s=18);axs[0].scatter(*goal,c='green',marker='*',s=75);axs[0].set_aspect('equal',adjustable='datalim');axs[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Actual trajectories')
axs[1].set(xlabel='Simulation time (s)',ylabel='Measured net progress (m)',title='Fixed start-goal axis')
axs[2].set(xlabel='Capture time (s)',ylabel='Actual partial-risk points',title='Existing texture/height consumer')
for ax in axs:ax.grid(alpha=.2);ax.legend()
fig.suptitle('TaskH known-pose E1 development: current sampling only; no 3D safety claim',fontsize=10);fig.tight_layout();out=H/'dense_current_live_e1_comparison_v1.png';fig.savefig(out,dpi=170);plt.close(fig)
print(out)
(H/'dense_current_live_e1_plot_identity_v1.json').write_text(json.dumps({'plot_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'plot_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'assessment_sha256':hashlib.sha256((H/'dense_current_live_e1_assessment_v1.json').read_bytes()).hexdigest()},indent=2)+'\n')
