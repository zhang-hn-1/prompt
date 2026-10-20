from pathlib import Path
import numpy as np,json,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'hypothesis_learning_model_v1';fig,axes=plt.subplots(1,3,figsize=(14,4));names=['hypothesis_learning_H1_run_v1','hypothesis_learning_H2_run_v1']
for case,col in zip(names,['tab:blue','tab:orange']):
 d=np.load(O/(case+'.npz'));axes[0].hist(d['probabilities'],bins=np.linspace(0,1,21),alpha=.5,label=case.split('_')[2]);s=json.loads((H/case/'states.json').read_text());p=np.array([r['position'] for r in s]);axes[1].plot([r['time_s'] for r in s],np.linalg.norm(p[:,:2]-p[0,:2],axis=1)*100,label=case.split('_')[2]);f=np.load(H/'hypothesis_learning_features_v1'/(case+'.npz'));axes[2].scatter(f['baseline_possible_xy'][:,0],f['baseline_possible_xy'][:,1],s=6,alpha=.5,label=case.split('_')[2])
t=np.load(H/names[0]/'offline_occupancy_label.npz');xy=t['origin']+np.argwhere(t['labels']>0)*float(t['resolution']);axes[2].scatter(xy[:,0],xy[:,1],s=3,c='black',label='offline2D occupied proxy')
axes[0].axvline(.5,c='red',ls='--');axes[0].set(xlabel='Frozen model probability',ylabel='Hypothesis voxel count',title='No fresh voxel reaches fixed0.5');axes[1].set(xlabel='Simulation time(s)',ylabel='Root XY drift(cm)',title='Zero request is not zero physical drift');axes[2].set(xlabel='World X(m)',ylabel='World Y(m)',title='Original conservative MODEL hypotheses');axes[2].axis('equal')
for a in axes:a.legend(fontsize=7);a.grid(alpha=.2)
fig.tight_layout();fig.savefig(O/'evidence_overview.png',dpi=160)
