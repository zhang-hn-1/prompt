from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'depth_local_curve_run';a=json.loads((H/'reference_continuity_assessment.json').read_text());xy=np.load(O/'episode_arrays.npz')['positions_world_xy'];fig,axes=plt.subplots(1,2,figsize=(13,6),constrained_layout=True)
for ax,event in zip(axes,[75,160]):
 old=event-5;g=np.load(O/'maps'/f'frame_{event:03d}.npz');prev=np.load(O/'maps'/f'frame_{old:03d}.npz');oldh=prev['labels']==1;newh=(g['labels']==1)&~oldh
 for mask,label,c,size in [(oldh,'Past observed hazards','gray',10),(newh,'New observed hazards','red',45)]:
  p=g['origin']+np.argwhere(mask)*float(g['resolution']);ax.scatter(p[:,0],p[:,1],c=c,s=size,label=label,zorder=5)
 for mf,c in [(old,'tab:blue'),(event,'tab:orange')]:
  q=next(q for q in a['counterfactuals'] if q['event_frame']==event and q['map_frame']==mf and q['position_frame']==event);p=np.asarray(q['route_xy']);ax.plot(p[:,0],p[:,1],c=c,ls='--',label=f'Grid route: map at {mf/10:.1f}s / current XY')
 p=np.load(O/'local_plans'/f'frame_{event-1:03d}.npz')['pose'][:,:2];ax.plot(p[:,0],p[:,1],c='black',lw=3,label='Previously installed local curve');ax.scatter(*xy[event],c='green',marker='*',s=100,label='Current robot position');r=next(r for r in a['rows'] if r['frame']==event)['previous_remaining'];ax.set(title=f'{event/10:.1f}s: local raster pass / continuous {r["suffix_clearance_m"]:.3f}m',xlabel='World x (m)',ylabel='World y (m)',xlim=(-3.5,3.5),ylim=(-3.3,3.3));ax.set_aspect('equal');ax.legend(fontsize=7,loc='lower right')
fig.suptitle('Observed-map changes flip the distant grid route; installed local-prefix validity is separate\nPrivileged depth/known pose, historical diagnostic; dashed alternatives are not executed trajectories',fontsize=11);fig.savefig(H/'reference_continuity_events.png',dpi=150)
