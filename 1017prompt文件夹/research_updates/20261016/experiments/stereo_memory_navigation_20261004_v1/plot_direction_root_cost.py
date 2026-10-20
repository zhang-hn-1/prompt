"""Scientific nominal-reference plot; neither curve is a new actual trajectory."""
from pathlib import Path
import json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
from reference_continuity_audit import reference_geometry_status
s=np.load(H/'direction_root_cost_snapshot.npz');r=np.load(H/'direction_root_cost_request.npz');a={k:np.load(H/f'direction_root_cost_{k}_reference.npz') for k in ['baseline','candidate']}
detail={k:reference_geometry_status(s['position_world_xy'],v['pose'][:,:2],s['labels'],s['origin'],float(s['resolution'])) for k,v in a.items()}
(H/'direction_root_cost_guard_detail.json').write_text(json.dumps(detail,indent=2)+'\n')
colors={'baseline':'#b45b14','candidate':'#1762ad'}
fig,axs=plt.subplots(1,2,figsize=(11.5,4.8),layout='constrained')
haz=s['origin']+np.argwhere(s['labels']==1)*float(s['resolution']);target=r['goal_xy'];start=r['start_xy']
for ax in axs:
 ax.scatter(haz[:,0],haz[:,1],s=25,c='#714769',marker='s',alpha=.45,label='Observed hazard-cell centres')
 for k,v in a.items():
  path=v['pose'][:,:2];ax.plot(path[:,0],path[:,1],color=colors[k],ls='--' if k=='baseline' else '-',lw=2,label=k.capitalize()+' nominal reference')
  for j,sign in enumerate(v['direction'][:-1]):
   d=path[j+1]-path[j];ax.annotate('',xy=path[j]+.65*d,xytext=path[j]+.4*d,arrowprops=dict(arrowstyle='->',color=colors[k],lw=1.4))
 ax.scatter(*start,c='black',s=45,marker='o',zorder=5,label='Current start')
 ax.scatter(*target,c='black',s=90,marker='*',zorder=5,label='Same local target')
 ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.grid(alpha=.18)
axs[0].set(xlim=(-1.7,1.5),ylim=(-3.3,-1.45),title='Frozen 17.5 s problem: nominal curves only')
axs[1].set(xlim=(-1.1,-.57),ylim=(-2.8,-1.97),title='First two primitives: both still reverse')
for k,v in a.items():
 for j in [1,2]:
  xy=v['pose'][j,:2];axs[1].annotate(f'{k[0].upper()}{j}',xy=xy,xytext=(5,4 if k=='candidate' else -13),textcoords='offset points',color=colors[k],fontsize=9)
axs[0].legend(loc='upper left',fontsize=8)
fig.suptitle('Root cost changes choice; no physical candidate run or braking-time prediction',fontsize=12)
fig.savefig(H/'direction_root_cost_comparison.png',dpi=170)
print('Saved nominal-only figure and exact guard details; no planner searches.')
