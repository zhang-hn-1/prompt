"""Saved nominal plan geometry only; no actual candidate trajectory."""
from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;a=json.loads((H/'nominal_support_assessment.json').read_text());mp=np.load(H/'endpoint_navigation_run/maps/frame_175.npz');origin=mp['origin'];res=float(mp['resolution']);haz=np.argwhere(mp['labels']==1)*res+origin
fig,axs=plt.subplots(1,2,figsize=(13,5.5))
ax=axs[0];ax.scatter(haz[:,0],haz[:,1],s=8,color='black',label='past/current measured hazard cells')
for arm,color in zip(a['arms'],['tab:blue','tab:orange']):
 xy=np.array(arm['pose'])[:,:2];ax.plot(xy[:,0],xy[:,1],'-o',color=color,label=arm['name']+' NOMINAL geometry',markersize=4)
 if arm['current_query']:
  q=arm['current_query']['target_world_xy'];ax.scatter(*q,marker='x',color=color,s=70)
cur=a['current'];ax.scatter(*cur['position_world_xy'],marker='s',color='green',label='same actual current pose');ax.scatter(*a['request_metadata']['local_target'],marker='*',s=160,color='red',label='same local goal')
allxy=np.concatenate([np.array(x['pose'])[:,:2] for x in a['arms']]);ax.set_xlim(allxy[:,0].min()-.8,allxy[:,0].max()+.8);ax.set_ylim(allxy[:,1].min()-.8,allxy[:,1].max()+.8);ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_title('Frame175 known-map CPU comparison\nNo candidate physical successor labels');ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)')
ax=axs[1]
for arm,color in zip(a['arms'],['tab:blue','tab:orange']):
 rows=arm['primitive_checks'];x=np.arange(len(rows));k=[r['nominal_curvature_1_m'] for r in rows];ax.plot(x,k,'-o',color=color,label=arm['name']);
 for r in rows:ax.vlines(r['index'],r['common_domain'][0],r['common_domain'][1],color=color,alpha=.25)
ax.set_xlabel('outgoing primitive index (different plans)');ax.set_ylabel('nominal body curvature (1/m)');ax.grid(alpha=.2);ax.legend();ax.set_title('Common table domain passes; overall eligibility REJECT\nLength3.0 ->3.6m; objective4.075 ->4.7')
fig.tight_layout();fig.savefig(H/'nominal_support_comparison.png',dpi=160);plt.close(fig)
