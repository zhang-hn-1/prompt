from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;a=json.loads((H/'source_event_handoff_assessment_v1.json').read_text());fig,axs=plt.subplots(2,3,figsize=(15,8),constrained_layout=True)
for row in a['arms']:
 arm=row['arm'];d=H/f'source_event_handoff_{arm}_run_v1';states=json.loads((d/'states.json').read_text());xy=np.array([s['position'][:2] for s in states]);t=np.array([s['time_s'] for s in states]);task=json.loads((d/'task_episode.json').read_text())['actual_episode'];goal=np.array(task['goal_position_world_m'][:2]);col='tab:blue' if arm=='B0' else 'tab:orange';ph=np.load(d/'physics.npz');r=json.loads((d/'policy_records.json').read_text());req=np.array([q['local']['request'] for q in r]);seed=np.load(d/'qualified_sensor_packets'/f"source_{row['seed_frame']:03d}.npz");haz=seed['world'];haz=haz[(haz[:,2]>.05)&(haz[:,2]<=1.5)]
 axs[0,0].plot(*xy.T,color=col,label=f'{arm}: {row["termination"]}');axs[0,0].scatter(*xy[-1],color=col,marker='x');axs[0,0].scatter(*haz[:,:2].T,s=2,color=col,alpha=.12)
 axs[0,1].plot(t,np.linalg.norm(xy-goal,axis=1),color=col,label=arm);axs[0,1].axvline(row['handoff_time_s'],color=col,linestyle=':');axs[0,2].plot([q['time_s'] for q in row['all_source_scores']],[q['current']['n'] for q in row['all_source_scores']],color=col,label=arm)
 axs[1,0].plot(t[:-1],req[:,0],color=col,label=arm);axs[1,1].plot(ph['simulation_time_s'],ph['applied_action'][:,0],color=col,label=arm)
 gt=row['geometry_trace'];axs[1,2].plot([q['time_s'] for q in gt],[q['offline_proxy_reference']['projection_distance_m'] for q in gt],color=col,label=arm)
 if arm=='C1':
  proxy=np.load(d/'offline_occupancy_label.npz');occupied=proxy['origin']+np.argwhere(proxy['labels']>0)*float(proxy['resolution']);axs[0,0].scatter(*occupied.T,s=2,c='grey',alpha=.08,label='offline occupancy proxy (not input)');axs[0,0].scatter(*goal,c='green',marker='*',s=80,label='actual local goal')
axs[0,0].set(title='Actual XY trajectories + qualified birth points',xlabel='world X (m)',ylabel='world Y (m)',aspect='equal');axs[0,0].set_xlim(-2.5,1.7);axs[0,0].set_ylim(-1.5,1.2)
for ax,title,y in [(axs[0,1],'Goal distance (whole budget)','distance (m)'),(axs[0,2],'Qualified current hazards (not full coverage)','point count'),(axs[1,0],'Actual final main requests','rad/s'),(axs[1,1],'Actual applied main targets','rad/s'),(axs[1,2],'Independent projection to each live reference','error (m)')]:ax.set(title=title,xlabel='simulation time (s)',ylabel=y)
axs[0,1].axhline(.2,c='grey',ls='--');axs[1,2].axhline(.2,c='grey',ls='--')
for ax in axs.flat:ax.grid(alpha=.15);ax.legend(fontsize=7)
fig.suptitle('Same-version 35s / 2.6m known-pose DEVELOPMENT pair: neither reached goal; C1 collision')
fig.savefig(H/'source_event_handoff_actual_v1.png',dpi=150)
