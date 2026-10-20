"""Presentation of already saved GT attribution; no matcher/scorer rerun."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
r=json.loads((D/'result.json').read_text());cfg=json.loads((D/'protocol.json').read_text())
assert r['complete'] and json.loads((D/'status.json').read_text())['child_returncode']==0
g=r['groups']['affected_protected_target']
fig,axes=plt.subplots(1,2,figsize=(10,4))
axes[0].bar(['original <=15cm','original >15cm','GT unscored'],[g['old_good'],g['old_bad'],866-g['scorable']],color=['#3388aa','#aa3377','#ddbb33'])
for i,n in enumerate([g['old_good'],g['old_bad'],866-g['scorable']]):axes[0].text(i,n,str(n),ha='center',va='bottom')
axes[0].set_title('866 protected self-RGB-sensitive risk queries')
axes[1].bar(['corrected','harmed','still bad'],[g['corrected'],g['harmed'],g['still_bad']],color=['#228833','#bb3355','#777777'])
for i,n in enumerate([g['corrected'],g['harmed'],g['still_bad']]):axes[1].text(i,n,str(n),ha='center',va='bottom')
axes[1].set_title('Same854 GT-target queries after intervention')
for ax in axes:ax.set_ylim(0,850);ax.set_ylabel('query instances')
fig.suptitle('Dependence is not a depth-error label:753/854 were originally within axial15cm\nFrozen RGB inversion invalidated all854; not an online method or3D truth.');fig.tight_layout();fig.savefig(D/'dependency_GT_transition_counts.png',dpi=130);plt.close(fig)
row=next(x for x in cfg['rows'] if x['sequence']=='E1A' and x['frame']==90)
with np.load(row['input_rgb_frame']) as raw,np.load(row['original_path']) as old:
    fig,axes=plt.subplots(4,5,figsize=(17,10));cm=plt.get_cmap('viridis').copy();cm.set_bad('.6');ec=plt.get_cmap('magma').copy();ec.set_bad('.6')
    for i,eye in enumerate(cfg['eyes']):
        z=old[eye+'_depth'].copy();z[~old[eye+'_valid']]=np.nan
        gt=raw[eye+'_depth_label'].copy();gt[~np.isfinite(gt)|(gt<=0)]=np.nan;error=np.abs(z-gt)
        with np.load(D/(row['sequence']+'_'+str(row['frame']).zfill(3)+'_'+eye+'.npz')) as a:
            m=a['affected']&a['protected'];q=a['query_pixel'];good=m&a['GT_finite_in_range']&a['original_good_within15cm'];bad=m&a['GT_finite_in_range']&~a['original_good_within15cm'];unknown=m&~a['GT_finite_in_range']
        for j,(image,title,cmap,hi) in enumerate([(raw[eye+'_rgb'],'actual RGB',None,None),(z,'original native axial m',cm,3),(gt,'sealed GT axial m',cm,3),(error,'valid-overlap |error| m',ec,.5)]):
            axes[i,j].imshow(image,**({} if cmap is None else dict(cmap=cmap,vmin=0,vmax=hi)));axes[i,j].axis('off');axes[i,j].set_title(eye+' / '+title,fontsize=8)
        ax=axes[i,4];ax.imshow(raw[eye+'_rgb'])
        for mask,color,label in [(good,'#ff3333','original <=15cm'),(bad,'#cc33ff','original >15cm'),(unknown,'#ffee33','GT unscored')]:ax.scatter(q[mask]%640,q[mask]//640,s=4,c=color,label=label)
        ax.axis('off');ax.set_title('affected protected original queries',fontsize=8)
    axes[0,4].legend(fontsize=6,loc='lower left');fig.suptitle('Fixed E1A exposure90 (~45s): RGB / native prediction / GT / error / dependency locations\nDepth display0-3m, error0-.5m; gray invalid. Conditional axial agreement only; editedRGB never deployed.');fig.tight_layout();fig.savefig(D/'fixed_GT_attribution_four_eyes.png',dpi=120)
