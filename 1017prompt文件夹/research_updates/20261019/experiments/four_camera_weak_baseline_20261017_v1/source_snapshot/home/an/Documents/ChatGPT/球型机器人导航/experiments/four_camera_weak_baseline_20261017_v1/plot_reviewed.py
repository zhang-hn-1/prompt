from paths import *
import numpy as np,json
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from source import NAMES
cfg=json.loads((D/'protocol.json').read_text());res=json.loads((D/'e0_result.json').read_text())
# Preselected fixed middle exposure, not selected by performance.
f='frame_055.npz';a=np.load(Path(cfg['data_root'])/'frames'/f);p=np.load(D/'e0_predictions'/f)
fig,axs=plt.subplots(4,5,figsize=(18,11))
for i,n in enumerate(NAMES):
    z=a[n+'_depth_label'];d=p[n+'_depth'].copy();valid=p[n+'_valid']&(d>=.5)&(d<=3);d[~valid]=np.nan
    e=np.abs(d-z);world=p[n+'_world'];cad=p[n+'_CAD']
    for j,(im,title,cm,lim) in enumerate([(a[n+'_rgb'],'RGB',None,None),(d,'SGBM axial metres','viridis',(0,3)),(z,'GT axial metres (score only)','viridis',(0,3)),(e,'absolute depth error metres','magma',(0,.5))]):
        ax=axs[i,j];kw={} if lim is None else dict(vmin=lim[0],vmax=lim[1]);cmap=plt.get_cmap(cm).copy() if cm else None; cmap.set_bad('0.6') if cmap else None; ax.imshow(im,cmap=cmap,**kw);ax.set_title(n+' / '+title,fontsize=8);ax.axis('off');ax.set_facecolor('.6')
    ax=axs[i,4];ax.scatter(world[:,0],world[:,1],s=.2);root=p['body_estimated'][:3,3];ax.plot(root[0],root[1],'rx');ax.set_aspect('equal');ax.set_xlim(root[0]-3.5,root[0]+3.5);ax.set_ylim(root[1]-3.5,root[1]+3.5);ax.set_facecolor('0.9');ax.set_title('risk proxy XY; missing = unknown',fontsize=8)
fig.suptitle('Current four cameras E0 fixed middle 27.5s; real RGB SGBM, no learned semantics; gray invalid. No historical map/navigation in this figure.')
fig.tight_layout();fig.savefig(D/'e0_fixed_middle.png',dpi=120);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(12,3));ts=[r['time_s'] for r in res['rows']]
axs[0].plot(ts,[r['union']['recall'] for r in res['rows']],label='union target recall');axs[0].plot(ts,[r['union']['precision'] for r in res['rows']],label='union risk precision');axs[0].set_ylim(0,1);axs[0].legend();axs[0].set_xlabel('seconds, all110frames')
axs[1].plot(ts,[r['generation_s'] for r in res['rows']]);axs[1].set_ylabel('4RGB generation seconds (not full latency)');axs[1].set_xlabel('seconds');fig.tight_layout();fig.savefig(D/'e0_complete_curves.png',dpi=120)
