"""Post-run actual trace and the online recorded consumer, never a policy input."""
from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
if __name__=='__main__':
 fig,ax=plt.subplots(1,2,figsize=(11,4))
 for arm,color in [('b0','tab:blue'),('candidate','tab:orange')]:
  d=H/f'forward_course_geometry_e1_{arm}_run_v1';states=json.loads((d/'states.json').read_text());rows=json.loads((d/'policy_records.json').read_text());xy=np.asarray([s['position'][:2] for s in states]);ep=json.loads((d/'task_episode.json').read_text())['actual_episode'];goal=np.asarray(ep['goal_position_world_m'][:2]);start=np.asarray(ep['start_position_world_m'][:2]);ax[0].plot(xy[:,0],xy[:,1],color=color,label=arm)
  rr=[r for r in rows if r.get('forward_course_geometry',{}).get('valid')];t=[r['time_s'] for r in rr];k=[r['current_tracking_chain']['requested_curvature_1_m'] for r in rr];ax[1].plot(t,k,color=color,label=arm)
  if arm=='candidate':
   used=[r for r in rr if r['forward_course_geometry']['used']];uxy=np.asarray([states[r['frame']]['position'][:2] for r in used]);
   if len(uxy):ax[0].scatter(uxy[:,0],uxy[:,1],s=8,color=color,alpha=.35,label='qualified course consumed')
 ax[0].plot([start[0],goal[0]],[start[1],goal[1]],'k--',alpha=.3,label='frozen start-goal axis');ax[0].scatter(*start,marker='o',color='black');ax[0].scatter(*goal,marker='*',s=100,color='green');ax[0].set_aspect('equal',adjustable='datalim');ax[0].set(xlabel='world x (m)',ylabel='world y (m)',title='Actual trajectories; TaskH known pose')
 ax[1].set(xlabel='simulation time (s)',ylabel='composed requested curvature (1/m)',title='Recorded current geometry input');
 for a in ax:a.grid(alpha=.2);a.legend(fontsize=8)
 fig.suptitle('Fixed rig/light, original 35 s; no claim of pure-RGB or 3D safety');fig.tight_layout();fig.savefig(H/'forward_course_geometry_e1_actual_trace_v1.png',dpi=160)
