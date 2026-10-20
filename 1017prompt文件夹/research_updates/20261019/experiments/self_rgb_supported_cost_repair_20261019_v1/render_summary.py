"""Post-seal presentation only, fixed E1A90 view; no model/scorer rerun."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
cfg=json.loads((D/'protocol.json').read_text());result=json.loads((D/'result.json').read_text())
assert result['complete'] and json.loads((D/'score_status.json').read_text())['child_returncode']==0
row=next(r for r in cfg['rows'] if r['sequence']=='E1A' and r['frame']==90)
names=['front_left','front_right','rear_left','rear_right']
cm=plt.get_cmap('viridis').copy();cm.set_bad('.6');ec=plt.get_cmap('magma').copy();ec.set_bad('.6')
with np.load(row['input_rgb_frame']) as raw,np.load(row['reference_baseline']) as base,np.load(D/'E1A_090_self_supported.npz') as new:
    fig,axes=plt.subplots(4,5,figsize=(17,10))
    for i,n in enumerate(names):
        z=base[n+'_depth'].copy();z[~base[n+'_valid']]=np.nan
        zz=new[n+'_depth'].copy();zz[~new[n+'_valid']]=np.nan
        truth=raw[n+'_depth_label'].copy();truth[~np.isfinite(truth)|(truth<=0)]=np.nan
        err=np.abs(zz-truth)
        for j,(im,title,cmap,hi) in enumerate([(raw[n+'_rgb'],'actual RGB',None,None),(z,'old reference depth m',cm,3),(zz,'CAD-support depth m',cm,3),(truth,'sealed GT depth m',cm,3),(err,'valid-overlap |error| m',ec,.5)]):
            axes[i,j].imshow(im,**({} if cmap is None else dict(cmap=cmap,vmin=0,vmax=hi)));axes[i,j].axis('off');axes[i,j].set_title(n+' / '+title,fontsize=8)
    fig.suptitle('Fixed E1A exposure90 (~45s): no frame selection by repair outcome\nGray invalid/unknown; depth clipped0-3m, error0-.5m. GT only post-seal. Reference is not native OpenCV equivalence.');fig.tight_layout();fig.savefig(D/'fixed_repair_four_eyes.png',dpi=120);plt.close(fig)
    fig,axes=plt.subplots(1,5,figsize=(17,4))
    for i,n in enumerate(names):
        axes[i].scatter(base[n+'_world'][:,0],base[n+'_world'][:,1],s=1,c='#dc7a27',label='old reference')
        axes[i].scatter(new[n+'_world'][:,0],new[n+'_world'][:,1],s=.7,c='#1878b0',label='CAD support')
        axes[i].set_title(n,fontsize=9)
    for data,col,label in [(base,'#dc7a27','old reference union'),(new,'#1878b0','CAD support union')]:
        points=np.concatenate([data[n+'_world'] for n in names]);axes[4].scatter(points[:,0],points[:,1],s=1,c=col,label=label)
    axes[4].set_title('four-eye joint BEV, height proxy',fontsize=9)
    for ax in axes:ax.set_xlim(-4.5,4.5);ax.set_ylim(-3,3);ax.set_aspect('equal');ax.set_facecolor('.85')
    axes[4].legend(fontsize=7);fig.suptitle('Fixed E1A90 per-eye/joint risk points / metres; absence is unknown. Long-term memory/navigation N/A.');fig.tight_layout();fig.savefig(D/'fixed_repair_per_eye_joint_BEV.png',dpi=120);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4));labels=[r['sequence']+'/'+str(r['frame']) for r in result['rows']];x=np.arange(4)
for arm,offset,col in [('baseline',-.18,'#dc7a27'),('self_supported',.18,'#1878b0')]:
    false=[r['arms'][arm]['union']['false_visible_cells'] for r in result['rows']]
    recall=[r['arms'][arm]['union']['recall']*100 for r in result['rows']]
    axes[0].bar(x+offset,false,width=.36,color=col,label=arm);axes[1].bar(x+offset,recall,width=.36,color=col,label=arm)
for ax in axes:ax.set_xticks(x,labels);ax.legend(fontsize=8)
axes[0].set_title('Cells >.15m from visible target');axes[1].set_title('Full target-cell coverage / %');axes[1].set_ylim(0,105)
fig.suptitle('Same four complete development frames; frozen gates, no threshold tuning');fig.tight_layout();fig.savefig(D/'repair_complete_frame_metrics.png',dpi=130)
