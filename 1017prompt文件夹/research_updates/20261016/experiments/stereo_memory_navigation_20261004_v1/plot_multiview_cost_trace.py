"""Every sealed anchor curve and every spatial selection, not success cherry-picking."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
H=Path(__file__).resolve().parent;r=json.loads((H/'multiview_cost_trace_audit.json').read_text());p=json.loads((H/'multiview_cost_trace_protocol.json').read_text())
fig,axes=plt.subplots(4,8,figsize=(24,12),constrained_layout=True)
for ri,row in enumerate(p['rows']):
 f=row['frame'];d=np.load(H/'multiview_cost_traces'/f'frame_{f:03d}.npz');depth=d['depths_m']
 for j,a in enumerate(row['anchors']):
  label=next(x for x in r['rows'] if x['frame']==f and x['query_index']==a['query_index']);ax=axes[ri,j]
  for vi in range(len(row['sources'])):ax.plot(depth,d['source_cost'][j,:,vi],lw=.5,alpha=.35)
  curve=d['cost'][j];ax.plot(depth,curve,color='#2b5f91',lw=1.2)
  ax.scatter(depth[d['competitive'][j]],curve[d['competitive'][j]],c='#df993b',s=8,zorder=3)
  if label['raw_depth_m'] is not None:ax.scatter(label['raw_depth_m'],label['best_cost'],c='#ce5047',s=20,zorder=4)
  ax.axvline(label['true_axis_depth_m'],color='#319565',ls='--',lw=1)
  changes=np.r_[False,np.diff(d['view_bits'][j].astype(int))!=0];ax.scatter(depth[changes],np.full(changes.sum(),1.9),c='#8c61b1',s=5,marker='|')
  err='unknown' if label['raw_ray_error_m'] is None else f'err={label["raw_ray_error_m"]:.3f}m'
  ax.set_title(f'{f:03d} q{a["query_index"]} {a["stratum"]}\nconf={int(label["confidence"])} {err}',fontsize=8)
  ax.set_xlim(.5,3);ax.set_ylim(-.05,2.05);ax.tick_params(labelsize=7)
  if ri==3:ax.set_xlabel('optical axial plane depth (m)',fontsize=8)
  if j==0:ax.set_ylabel('1-ZNCC cost',fontsize=8)
handles=[Line2D([0],[0],color=c,ls=ls,marker=m,label=l) for c,ls,m,l in [('#2b5f91','-',None,'valid median cost (gaps invalid)'),('#df993b','None','o','competitive bins'),('#ce5047','None','o','raw winner'),('#319565','--',None,'independent true depth (label only)'),('#8c61b1','None','|','source bitmap changes')]]
fig.legend(handles=handles,loc='outside lower center',ncol=5,fontsize=9);fig.suptitle('All 32 preregistered anchors, frozen V1; curves saved before labels, no candidate tuning')
fig.savefig(H/'figures/multiview_cost_trace_all_anchors.png',dpi=120);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
for ax,row in zip(axes.flat,p['rows']):
 packet=np.load(H/'multiview_rgb_inputs'/f'frame_{row["frame"]:03d}.npz');ax.imshow(packet['left_rgb'])
 for j,a in enumerate(row['anchors']):
  y,x=a['pixel_yx'];c='#2f76c0' if a['stratum']=='spatial' else '#e95848';ax.scatter(x,y,s=30,facecolors='none',edgecolors=c,lw=1.5)
  ax.annotate(str(j+1),(x,y),xytext=(3,3),textcoords='offset points',fontsize=9,color=c,fontweight='bold',bbox=dict(facecolor='white',alpha=.6,pad=.5,edgecolor='none'))
 ax.set_title(f'{row["frame"]:03d}: 1-4 spatial, 5-8 retained-output stratum');ax.axis('off')
fig.suptitle('Deterministic sampling from frozen full source queries; retained stratum uses confidence, never GT accuracy')
fig.savefig(H/'figures/multiview_cost_trace_anchor_rgb.png',dpi=140);plt.close(fig)
print('SAVED all32 cost curves and all4 actual RGB anchor locations')
