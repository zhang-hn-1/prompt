from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;W=D.parents[1];C=W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run';R=D/'candidate_e2_run'
fig,axes=plt.subplots(1,2,figsize=(11,4.5));summary={}
for label,run in [('control_height_source',C),('viewpoint_confirmation_candidate',R)]:
 states=json.loads((run/'states.json').read_text());rows=json.loads((run/'policy_records.json').read_text());q=np.load(run/'requests.npz')['requests'];xy=np.array([s['position'][:2] for s in states]);t=np.array([s['time_s'] for s in states]);ep=json.loads((run/'task_episode.json').read_text())['actual_episode'];goal=np.array(ep['goal_position_world_m'][:2]);dist=np.linalg.norm(xy-goal,axis=1)
 axes[0].plot(xy[:,0],xy[:,1],label=label);axes[1].plot(t,dist,label=label)
 main=q.reshape(-1,5,2)[:,:,0]
 summary[label]=dict(remaining_m=float(dist[-1]),closest_m=float(dist.min()),net_progress_m=float(dist[0]-dist[-1]),path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),zero_main_requests=int(np.all(main==0,axis=1).sum()),positive_main_requests=int(np.any(main>0,axis=1).sum()),negative_main_requests=int(np.any(main<0,axis=1).sum()),reference_versions=len(set(r['local_reference_version'] for r in rows)),final_plan=rows[-1]['plan'],termination=json.loads((run/'termination.json').read_text()))
axes[0].scatter(*goal,marker='*',s=100,c='black',label='goal');axes[0].set(xlabel='world X (m)',ylabel='world Y (m)',title='Actual trajectories');axes[0].axis('equal')
axes[1].axhline(.2,color='black',linestyle='--');axes[1].set(xlabel='simulation time (s)',ylabel='remaining distance (m)',title='90s navigation, same scene')
for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(D/'navigation_comparison.png',dpi=160);plt.close(fig)
summary['limits']='Historical control RGB may differ; same development E2, not full causal A/B or independent generalization. Partial surface qualification does not establish free volume or reliable navigation.'
(D/'navigation_comparison.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:{a:b for a,b in v.items() if a not in ['final_plan']} if isinstance(v,dict) else v for k,v in summary.items()}))
