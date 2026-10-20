from pathlib import Path
import sys,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;S=D.parent/'dual_monocular_semantic_bev_20261015_v1';sys.path.insert(0,str(S));from projection import per_eye,cells
R=D.parent/'replanning_reserve_20261013_v1/baseline_e1_run';cfg=json.loads((R.parent/'baseline_e1_protocol.json').read_text());f=np.load(R/'frames/frame_450.npz');p=np.load(S/'predictions/B65/frame_450.npz');m=np.load(S/'predictions/B65/memory_450.npz');masks=dict(np.load(cfg['static_masks']));origin=p['origin'];ts=[cells(per_eye(f[e+'_depth_m'],f[e+'_camera_usd_world'],masks[e],f['root_position_world_m'],stride=1)[0],origin) for e in ['left','right']];t=np.unique(np.concatenate(ts),axis=0);ref=t*.1+origin;g=np.load(cfg['occupancy']);body=np.argwhere(g['grid']>0)*float(g['resolution_m'])+g['global_origin_xy'];trees=[cKDTree(ref),cKDTree(body)];fig,ax=plt.subplots(1,4,figsize=(16,4));ax[0].scatter(body[:,0],body[:,1],s=3,c='gray');ax[0].scatter(ref[:,0],ref[:,1],s=15,c='green');ax[0].set_title(f'Full bodyband ref gray / original visible green\n{len(t)} current visible cells at 45s')
stats=[]
for j,k in enumerate(['left_cells','right_cells','fused_cells']):
 w=p[k]*.1+origin;dist=trees[0].query(w)[0];good=dist<=.15+1e-10;stats.append(dict(arm=k,predicted_cells=len(w),within_visible_target_015=int(good.sum()),visible_targets=len(t),covered_visible=int((cKDTree(w).query(ref)[0]<=.15+1e-10).sum())))
 if j==2:ax[1].scatter(w[:,0],w[:,1],c=np.where(good,'green','red'),s=8);ax[1].set_title(f'Closed v77 fused prior, original lens\n{good.sum()}/{len(w)} within .15m visible target')
ix=np.argwhere(m['labels']==1);w=ix*.1+m['origin'];dist=trees[1].query(w)[0];wrong=dist>.15+1e-10;age=float(m['time_s'])-m['clock'][tuple(ix.T)];ax[2].scatter(w[:,0],w[:,1],c=np.where(wrong,'red','green'),s=4);ax[2].scatter(w[age>10,0],w[age>10,1],c='purple',s=2);ax[2].set_title(f'Closed v77 memory only\n{len(w)} risk / {wrong.sum()} far / {(age>10).sum()} old10s')
ax[3].scatter(ref[:,0],ref[:,1],s=15,c='green');ax[3].set_title(f'Wide frustum potential for ORIGINAL targets\n{len(t)}/{len(t)} only, no new union/map/source')
for a in ax:a.set_aspect('equal',adjustable='box');a.set_xlim(-5,5);a.set_ylim(-5,5)
fig.suptitle('Same existing B65 fixed midpoint: source context, NOT wide-optic predictions or memory\nGreen within reference .15m / red far / purple old10s; blank UNKNOWN, not free');fig.tight_layout();fig.savefig(D/'original_context_bev.png',dpi=150);plt.close(fig)
(D/'original_context_bev_stats.json').write_text(json.dumps(dict(arms=stats,old_memory_risk=len(w),old_memory_far=int(wrong.sum()),old_memory_old10=int((age>10).sum()),scope='Old v77 current/long memory context; current uses visible GT, memory uses full bodyband reference. Not new optic model or map.'),indent=2)+'\n')
