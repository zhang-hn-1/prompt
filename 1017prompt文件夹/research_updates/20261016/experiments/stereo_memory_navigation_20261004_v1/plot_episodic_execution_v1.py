from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
H=Path(__file__).resolve().parent
if __name__=='__main__':
 a=json.loads((H/'episodic_execution_memory_run_v1/assessment.json').read_text());p={(v['episode'],v['frame']):v for v in json.loads((H/'episodic_execution_memory_run_v1/predictions.json').read_text())}
 rows=[r for r in a['rows'] if r['score_status']=='scored'];fig,axes=plt.subplots(2,3,figsize=(12,7))
 for ax,r in zip(axes.flat,rows):
  pred=p[r['episode'],r['frame']]
  for key,label in [('target','teacher'),('trajectory','episodic'),('constant_velocity','constant velocity')]:
   t=np.array(r[key] if key=='target' else pred[key]);t=np.r_[np.zeros((1,3)),t];ax.plot(t[:,0],t[:,1],'.-',label=label)
  ax.set_title(('E1' if '_e1_' in r['episode'] else 'E2')+f" frame {r['frame']}");ax.set_xlabel('relative X (m)');ax.set_ylabel('relative Y (m)');ax.grid(alpha=.25);ax.axis('equal')
 axes.flat[0].legend(fontsize=8);fig.suptitle('Offline privileged-state diagnostic: 6 / 30 scored\nCoverage gate failed; selected trajectories are not navigation admission')
 fig.tight_layout();fig.savefig(H/'episodic_execution_trajectories_v1.png',dpi=160)
