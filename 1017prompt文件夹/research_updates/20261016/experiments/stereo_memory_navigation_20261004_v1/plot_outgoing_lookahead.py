"""Known current reference/current-state geometry; no hypothetical future motion."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
r=json.loads((H/'outgoing_lookahead_rows.json').read_text());actual=json.loads((H/'cusp_navigation_run/policy_records.json').read_text())
f,axes=plt.subplots(1,3,figsize=(15,4.5))
for ax,index in zip(axes[:2],[197,297]):
 x=next(z for z in r if z['frame']==index);ref=np.load(H/'cusp_navigation_run/local_plans'/f'frame_{index:03d}.npz')['pose'];idx=x['reference_index'];t=actual[index]['tracking'];xy=np.array([t['position_world_x_m'],t['position_world_y_m']]);ax.plot(ref[:4,0],ref[:4,1],'k.-',label='CURRENT known local reference');ax.plot(ref[idx:idx+2,0],ref[idx:idx+2,1],'g-',lw=4,alpha=.45,label='Outgoing reverse edge');ax.scatter(*xy,marker='*',s=100,c='black',label='CURRENT measured position')
 for key,c,label in [('old','tab:red','Original target'),('candidate','tab:blue','Corrected CURRENT target')]:
  target=x[key]['target_world_xy'];ax.scatter(*target,c=c,marker='x',s=80,label=label);ax.plot([xy[0],target[0]],[xy[1],target[1]],color=c,ls=':')
 ax.set(title=f'frame{index} / t={index/10}s, no future trajectory',xlabel='World X(m)',ylabel='World Y(m)');ax.axis('equal');ax.legend(fontsize=7);ax.grid(alpha=.25)
for key,c in [('old','tab:red'),('candidate','tab:blue')]:
 z=[x for x in r if x['direction']=='reverse' and x['reference_version'] in [3,6]];axes[2].scatter([x['time_s'] for x in z],[x[key]['curvature_body_1_m'] for x in z],color=c,s=14,label=key+' current algebra')
axes[2].set(xlabel='Actual archived current time(s)',ylabel='Signed-body PP curvature(1/m)',title='Current query; candidate movement UNTESTED');axes[2].grid(alpha=.25);axes[2].legend(fontsize=8)
f.tight_layout();f.savefig(H/'outgoing_lookahead_current_geometry.png',dpi=150);plt.close(f)
