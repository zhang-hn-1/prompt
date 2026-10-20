from pathlib import Path
import json,hashlib,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from scipy.ndimage import maximum_filter
D=Path(__file__).resolve().parent
# Reuse frozen scoring function without re-running its top-level scoring driver.
s=(D/'score.py').read_text();ns={};exec('import numpy as np\nfrom scipy.ndimage import maximum_filter,distance_transform_edt\n'+s[s.index('def metrics('):s.index('\nrows=[]')],ns);metrics=ns['metrics'];seal=json.loads((D/'sgbm_prediction_seal.json').read_text())
for p,h in seal['outputs'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
r=json.loads((D/'result.json').read_text());comparisons=[]
cm=ListedColormap(['white','#83c5e8','#fdae61','#d7191c','#762a83']);norm=BoundaryNorm(np.arange(-.5,5.5),5)
for name in ['H','E0','E1']:
 a=np.load(D/'data'/f'{name}.npz');y=a['labels'];p=np.load(D/'predictions'/f'{name}.npz');b=np.load(D/'sgbm_predictions'/f'{name}.npz')['labels'];row=dict(group=name,sgbm_current_surface=metrics(y,b));comparisons.append(row)
 if name=='H':continue
 i=90;truth=y[i];pred=p['labels'][i];prob=p['prob'][i];bad=(pred>=2)&~maximum_filter(truth>=2,size=3,mode='constant');miss=(truth>=2)&~maximum_filter(pred>=2,size=3,mode='constant');err=np.zeros(truth.shape,int);err[(pred>=2)&~bad]=1;err[bad]=2;err[miss]=3
 fig,axs=plt.subplots(2,3,figsize=(13,8));axs=axs.ravel();x=a['rgb'][i];axs[0].imshow(np.concatenate([x[:3].transpose(1,2,0),x[3:].transpose(1,2,0)],axis=1));axs[0].set_title(name+' current left/right RGB at45s');axs[0].axis('off')
 def grid(ax,z,title,cmap=cm,n=norm,vmin=None,vmax=None):
  im=ax.imshow(z,origin='lower',extent=(-3,3,0,4),aspect='equal',interpolation='nearest',cmap=cmap,norm=n,vmin=vmin,vmax=vmax);ax.set(title=title,xlabel='Lateral (m)',ylabel='Forward (m)');return im
 grid(axs[1],truth,'Visible true surfaces (labels only)');grid(axs[2],pred,'Learned current local map');grid(axs[3],b[i],'SGBM current risk baseline')
 grid(axs[4],err,'Green supported; red unmatched; blue missed',ListedColormap(['white','#208b48','#d73027','#2166ac']),BoundaryNorm(np.arange(-.5,4.5),4))
 entropy=-(prob*np.log(np.maximum(prob,1e-8))).sum(0)/np.log(5);im=grid(axs[5],entropy,'Predictive entropy (not calibrated)','magma',None,0,1);fig.colorbar(im,ax=axs[5],shrink=.7)
 fig.suptitle(name+(' TRAINING scene' if name=='E0' else ' EXCLUDED from fit; previously seen development')+' — fixed sequence midpoint (45s)\nWhite unknown; light blue ground NOT free; orange/red/purple obstacle height bins');fig.tight_layout();fig.savefig(D/f'{name}_local_map.png',dpi=150,bbox_inches='tight');plt.close(fig)
(D/'sgbm_comparison.json').write_text(json.dumps(comparisons,indent=2)+'\n')
# Learning graph reports training loss only: no test-driven checkpoint selection.
trace=json.loads((D/'train_trace.json').read_text());fig,ax=plt.subplots(figsize=(6,4));ax.plot([v['epoch'] for v in trace],[v['train_loss'] for v in trace]);ax.set(xlabel='Epoch',ylabel='Training weighted cross entropy',title='One frozen fit; final epoch only');fig.tight_layout();fig.savefig(D/'training_loss.png',dpi=140);plt.close(fig)
for v in comparisons:print(v['group'],v['sgbm_current_surface']['precision'],v['sgbm_current_surface']['recall'],v['sgbm_current_surface']['localization_p90_m'])
