from paths import *
from source import NAMES,RAYS,AX
import numpy as np,json
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
out=D/'e1_run';r=json.loads((D/'e1_result.json').read_text());cfg=json.loads((D/'protocol.json').read_text());cap=json.loads((out/'capture_seal.json').read_text())
physical=np.load(out/'physics.npz')['root_position_world_m'];imu=json.loads((out/'native_imu.json').read_text());estimated=np.array([v['estimated_base_world'] for v in imu])[:,:3,3]
ep=json.loads((out/'episode.json').read_text());goal=np.array(ep['goal_position_world_m'])
occ=np.load(cfg['occupancy']);reference=np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy'];tree=cKDTree(reference)
m=np.load(out/'maps'/f"frame_{r['actions']-1:03d}.npz");ids=np.argwhere(m['labels']==1);xy=ids*float(m['resolution'])+m['origin'];false=tree.query(xy)[0]>.15 if len(xy) else np.zeros(0,bool)
fig,axs=plt.subplots(1,3,figsize=(15,4))
for ax in axs[:2]:
 ax.scatter(reference[:,0],reference[:,1],s=2,c='.75',label='GT bodyband (offline)');ax.plot(physical[:,0],physical[:,1],c='b',lw=1,label='actual');ax.plot(estimated[:,0],estimated[:,1],c='orange',lw=1,label='native estimate');ax.plot(goal[0],goal[1],'k*',ms=10);ax.set_aspect('equal');ax.set_xlim(min(physical[:,0].min(),goal[0])-1,max(physical[:,0].max(),goal[0])+1);ax.set_ylim(physical[:,1].min()-2,physical[:,1].max()+2)
axs[0].set_title('Actual/estimated trajectory, development E1');axs[0].legend(fontsize=7)
if len(xy):axs[1].scatter(xy[:,0],xy[:,1],s=10,c=np.where(false,'red','green'),label='B retained risk: red >.15m fromGT')
axs[1].set_title('Last decision B memory; absence=unknown');axs[1].legend(fontsize=6)
ts=[v['time_s'] for v in r['source_rows']];axs[2].plot(ts,[v['covered_cells']/v['target_cells'] if v['target_cells'] else np.nan for v in r['source_rows']],c='blue',label='visible target recall');axs[2].set_ylim(0,1);axs[2].set_ylabel('target recall',color='blue');right=axs[2].twinx();right.plot([x*.1 for x in range(len(r['false_map_cells_by_policy']))],r['false_map_cells_by_policy'],c='orange',label='false map cells');right.set_ylabel('false memory cells',color='orange');axs[2].set_xlabel('simulation seconds')
fig.suptitle('Fixed4camera SGBM + native IMU/encoder + existing B memory. Offline GT only. '+r['termination']);fig.tight_layout();fig.savefig(D/'e1_trajectory_memory.png',dpi=130);plt.close(fig)
# Fixed source exposure at45s; if absent show first frame and state why.
f='frame_090.npz' if (out/'frames/frame_090.npz').exists() else 'frame_000.npz';a=np.load(out/'frames'/f);p=np.load(out/'predictions'/f);fig,axs=plt.subplots(4,5,figsize=(17,10))
for i,n in enumerate(NAMES):
 d=p[n+'_depth'].copy();valid=p[n+'_valid']&(d>=.5)&(d<=3);d[~valid]=np.nan;gt=a[n+'_depth_label']
 for j,(im,title,cmap,lim) in enumerate([(a[n+'_rgb'],'actualRGB',None,None),(d,'SGBM axialm','viridis',(0,3)),(gt,'offline GTm','viridis',(0,3)),(abs(d-gt),'absolute errorm','magma',(0,.5))]):
  ax=axs[i,j];cm=plt.get_cmap(cmap).copy() if cmap else None; cm.set_bad('0.6') if cm else None; ax.imshow(im,cmap=cm,**({} if lim is None else dict(vmin=lim[0],vmax=lim[1])));ax.set_facecolor('.6');ax.set_title(n+' '+title,fontsize=8);ax.axis('off')
 T=a[n+'_camera_usd_world_label'];gtworld=(RAYS*gt[...,None])@(T[:3,:3]@AX).T+T[:3,3]
 k=np.isfinite(gt)&(gt>=.5)&(gt<=3)&~p[n+'_CAD']&(np.linalg.norm(gtworld-a['root_position_physics_label'],axis=2)>=.55)&(gtworld[...,2]>.05)&(gtworld[...,2]<=1.5)
 truthxy=gtworld[k][::8];risk=p[n+'_world'];root=p['body_estimated'][:3,3];ax=axs[i,4]
 ax.scatter(truthxy[:,0],truthxy[:,1],s=.2,c='.6');ax.scatter(risk[:,0],risk[:,1],s=.3,c='blue');ax.plot(root[0],root[1],'rx');ax.set_xlim(root[0]-3.5,root[0]+3.5);ax.set_ylim(root[1]-3.5,root[1]+3.5);ax.set_aspect('equal');ax.set_facecolor('.95');ax.set_title(n+' XY:grayGT bluecurrent rednative',fontsize=8)
fig.suptitle('E1 predetermined45s source or first exposure if45s unavailable; actual t='+str(float(a['source_time_s']))+'s; gray invalid/unknown, no RGB semantics.');fig.tight_layout();fig.savefig(D/'e1_fixed_source.png',dpi=110);plt.close(fig)
