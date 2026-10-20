from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;cfg=json.loads((D/'protocol.json').read_text());row=cfg['rows'][3];N=['front_left','front_right','rear_left','rear_right']
assert row['sequence']=='E1A' and row['frame']==90
with np.load(row['input_rgb_frame']) as a:rgb={n:a[n+'_rgb'] for n in N}
with np.load(D/'E1A_090_constant.npz') as a:aa={k:a[k] for k in a.files}
with np.load(D/'E1A_090_replicate.npz') as a:bb={k:a[k] for k in a.files}
fig,axs=plt.subplots(5,5,figsize=(20,17));fig.suptitle('Fixed E1A frame90, t=45s: OFF-IMAGE padding intervention only\nGT depth / true error / RGB semantics / historical BEV / navigation = N/A',fontsize=16)
for i,n in enumerate(N):
 axs[i,0].imshow(rgb[n]);axs[i,0].set_title(n+' RGB (same bytes)')
 for j,(label,a) in enumerate([('black padding',aa),('edge padding',bb)],1):
  z=a[n+'_depth'];v=a[n+'_valid']&(z>=.5)&(z<=3);cmap=plt.cm.viridis.copy();cmap.set_bad('.65');im=axs[i,j].imshow(np.where(v,z,np.nan),vmin=.5,vmax=3,cmap=cmap);axs[i,j].set_title(label+' depth [m], source window');fig.colorbar(im,ax=axs[i,j],fraction=.035)
 both=aa[n+'_valid']&bb[n+'_valid'];dif=np.where(both,abs(aa[n+'_depth']-bb[n+'_depth']),np.nan);cmap=plt.cm.magma.copy();cmap.set_bad('.65');im=axs[i,3].imshow(dif,vmin=0,vmax=.3,cmap=cmap);axs[i,3].set_title('depth CHANGE, not GT error [m]');fig.colorbar(im,ax=axs[i,3],fraction=.035)
 body=aa['body_estimated'][:2,3]
 for a,c,sz,lab in [(aa,'.25',4,'black'),(bb,'magenta',1,'edge')]:
  w=a[n+'_world'][:,:2]-body;axs[i,4].scatter(w[:,0],w[:,1],s=sz,c=c,label=lab,alpha=.6)
 axs[i,4].set_title('current XY point cloud, not map');axs[i,4].set_xlim(-3,3);axs[i,4].set_ylim(-3,3);axs[i,4].set_aspect('equal');axs[i,4].set_facecolor('.85');axs[i,4].legend(fontsize=7)
 for j in range(4):axs[i,j].set_xticks([]);axs[i,j].set_yticks([])
for ax in axs[4]:ax.axis('off')
ax=axs[4,0];ax.axis('on')
for a,c,sz,lab in [(aa,'.25',3,'black'),(bb,'magenta',1,'edge')]:
 w=np.concatenate([a[n+'_world'][:,:2] for n in N])-body;ax.scatter(w[:,0],w[:,1],s=sz,c=c,label=lab,alpha=.5)
ax.set_xlim(-3,3);ax.set_ylim(-3,3);ax.set_aspect('equal');ax.set_title('joint current cloud XY [m]');ax.legend(fontsize=7);ax.set_facecolor('.85')
axs[4,1].text(0,1,'Gray pixels: unknown / outside shown source window\nBlack/edge colors are interventions, not correctness\nMagenta points are not an approved measurement\nCloud XY uses original causal estimated pose',va='top',fontsize=12)
axs[4,3].text(0,1,'Selection fixed before this diagnostic:\n4th of previous v113 timing frames.\nNo GT arrays read for rendering.\nLocal changes do not identify false risks.',va='top',fontsize=12)
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(D/'fixed_padding_dependency.png',dpi=125);plt.close(fig)
r=json.loads((D/'result.json').read_text());fig,ax=plt.subplots(figsize=(11,4));labels=[];tot=[];affected=[]
for row in r['rows']:
 labels.append(row['sequence']+' frame'+str(row['frame']));tot.append(sum(a['old_risk_point_denominator'] for a in row['eyes']));affected.append(sum(a['old_risk_points_lost_or_shift_gt15cm'] for a in row['eyes']))
x=np.arange(4);ax.bar(x,tot,color='.7',label='all original risk points');ax.bar(x,affected,color='magenta',label='lost or depth change >.15m')
for i,(a,b) in enumerate(zip(affected,tot)):ax.text(i,b+100,f'{a}/{b}',ha='center')
ax.set_xticks(x,labels);ax.set_ylim(0,max(tot)*1.18);ax.legend();ax.set_ylabel('point instances');ax.set_title('Complete four-frame denominator; not true-error or navigation score');fig.tight_layout();fig.savefig(D/'complete_four_frame_dependency.png',dpi=140)
print('RENDERED_NO_GT')
