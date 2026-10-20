from common import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
# Chosen by first blocking event's earliest saved erroneous input, not minimum/maximum error.
r=load(D/'birth_result.json');i=min(x['frame'] for x in r['first_event_all_false_cell_source_lineage']);name=f'frame_{i:03d}.npz'
fig,axs=plt.subplots(4,4,figsize=(13,10))
with np.load(RUN/'frames'/name) as a,np.load(RUN/'predictions'/name) as p:
    for row,n in enumerate(NAMES):
        pred=p[n+'_depth'].astype(float);valid=p[n+'_valid']&np.isfinite(pred)&(pred>=.5)&(pred<=3)&~p[n+'_CAD'];pred[~valid]=np.nan
        gt=a[n+'_depth_label'].astype(float);gt[~np.isfinite(gt)|(gt<.5)|(gt>3)|p[n+'_CAD']]=np.nan
        err=abs(pred-gt);finite=np.isfinite(err);q90=float(np.quantile(err[finite],.9)) if finite.any() else None
        for col,(im,title,cmap,limits) in enumerate([(a[n+'_rgb'],'saved RGB',None,None),(pred,'sealed SGBM depth(m)','viridis',(.5,3)),(gt,'offline GT depth(m)','viridis',(.5,3)),(err,'finite valid error(m)','magma',(0,1.5))]):
            ax=axs[row,col];cm=plt.get_cmap(cmap).copy() if cmap else None
            if cm:cm.set_bad('.6')
            ax.imshow(im,cmap=cm,**({} if limits is None else dict(vmin=limits[0],vmax=limits[1])));ax.axis('off');ax.set_title(n+' '+title,fontsize=8)
            if col==3:ax.text(.01,.02,f'n={finite.sum()}; P90={q90:.3f}m' if q90 is not None else 'N/A',transform=ax.transAxes,color='white',fontsize=8,bbox=dict(facecolor='black',alpha=.6,pad=2))
    time=float(a['source_time_s'])
fig.suptitle(f'First no-route event: earliest contributing source={i}, actual t={time:.2f}s\nExisting RGB/predictions, postseal teacher only. Gray=CAD/missing/outside .5-3m; error is conditional, not full coverage.\nNo new source inference, RGB semantics or memory output.',fontsize=11)
fig.tight_layout(rect=[0,0,1,.91]);fig.savefig(D/'birth_first_source.png',dpi=120);plt.close(fig)
print('PLOT_BIRTH',i)
