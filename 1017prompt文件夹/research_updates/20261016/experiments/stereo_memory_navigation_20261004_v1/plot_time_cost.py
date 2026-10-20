from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
r=json.loads((H/'time_cost_assessment.json').read_text());fig,ax=plt.subplots(figsize=(8,6))
m=np.load(H/'endpoint_navigation_run/maps/frame_175.npz');occ=m['labels']==1;o=m['origin'];res=float(m['resolution']);cells=np.argwhere(occ);ax.scatter(o[0]+cells[:,0]*res,o[1]+cells[:,1]*res,s=7,c='gray',alpha=.35,label='Observed hazards (past/current)')
for a,col in zip(r['arms'],['#1f77b4','#d62728']):
 if a['pose']:
  p=np.asarray(a['pose']);ax.plot(p[:,0],p[:,1],'o-',color=col,label=a['name']+' / '+a['status'])
 else:ax.plot([],[],color=col,label=a['name']+' / '+a['status'])
start=np.asarray(r['current']['position_world_xy']);goal=np.asarray(r['request_metadata']['local_target']);ax.scatter(*start,c='black',s=65,marker='s',label='Same measured current state');ax.scatter(*goal,c='green',s=100,marker='*',label='Same local target');ax.set_xlim(start[0]-2.5,start[0]+2.5);ax.set_ylim(start[1]-2.5,start[1]+2.5);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('World X / m');ax.set_ylabel('World Y / m');ax.legend(fontsize=8);ax.set_title('Current 175: static nominal cost comparison / no physical rollout');fig.tight_layout();fig.savefig(H/'time_cost_comparison.png',dpi=150)
