"""Whole executed trajectories and planning decisions; no online evaluation input."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;result={};fig,axs=plt.subplots(1,2,figsize=(11,5))
for ax,arm in zip(axs,['baseline_e1','candidate_e1']):
    R=D/(arm+'_run')
    if not (R/'termination.json').exists():continue
    states=json.loads((R/'states.json').read_text());rows=json.loads((R/'policy_records.json').read_text())
    xy=np.array([s['position'][:2] for s in states]);goal=np.array(json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2])
    versions=sorted({int(r.get('local_reference_version',0)) for r in rows if r.get('local_reference_version',0)>0})
    rebuild=[r for r in rows if r.get('local_plan_created')];snap=[]
    for r in rebuild:
        meta=r.get('plan') or {};d=meta.get('observed_end')
        snap.append(dict(frame=r.get('frame'),time_s=r.get('time_s'),version=r.get('local_reference_version'),status=meta.get('status'),target=meta.get('local_target'),observed_end=d))
    goal_dist=np.linalg.norm(xy-goal,axis=1)
    result[arm]=dict(path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),initial_distance_m=float(goal_dist[0]),remaining_m=float(goal_dist[-1]),net_progress_m=float(goal_dist[0]-goal_dist[-1]),planning_attempt_versions=len(versions),rebuild_events=len(rebuild),successful_installed_paths=sum(r['status']=='success' for r in snap),failed_queries=sum(r['status']!='success' for r in snap),planning_events=snap,scope='Actual trajectory and online planning records; version increments include failed queries, not all installed paths. Same E1 development, not generalization or complete paired causality')
    cfg=json.loads((D/(arm+'_protocol.json')).read_text());m=np.load(cfg['occupancy']);t=m['global_origin_xy']+np.argwhere(m['grid']>0)*float(m['resolution_m'])
    ax.scatter(t[:,0],t[:,1],s=7,c='.85',marker='s',label='Truth (evaluation only)');ax.plot(xy[:,0],xy[:,1],c='#2166ac',label='Executed trajectory');ax.plot(*goal,'k*',ms=11,label='Goal');ax.plot(*xy[0],'ko',ms=4)
    for r in snap:
        if r['observed_end'] and not r['observed_end']['fallback']:
            p=r['target'];ax.plot(*p,'o',c='#ed8b23',ms=5)
    ax.set(xlim=(-5,5),ylim=(-5,5),aspect='equal',xlabel='World X (m)',ylabel='World Y (m)',title=f'{arm} | {states[-1]["time_s"]:.1f}s');ax.legend(fontsize=8,loc='lower left')
(D/'navigation_summary.json').write_text(json.dumps(result,indent=2)+'\n')
fig.suptitle('Actual E1 trajectories; orange = planned exploratory subgoals\nTruth only in this post-run figure, not planner input');fig.tight_layout();fig.savefig(D/'actual_trajectories.png',dpi=140,bbox_inches='tight');plt.close(fig)
print({k:{a:b for a,b in v.items() if a!='planning_events'} for k,v in result.items()})
