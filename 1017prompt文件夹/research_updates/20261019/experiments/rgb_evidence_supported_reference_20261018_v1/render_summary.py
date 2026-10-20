"""Post-seal visualization only. Fixed fourth protocol row; no source rerun."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/four_camera_weak_baseline_20261017_v1'))
from source import NAMES,RAYS,AX
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 cfg=json.loads((D/'protocol.json').read_text());seal=json.loads((D/'prediction_seal.json').read_text());result=json.loads((D/'result.json').read_text())
 assert seal['complete'] and result['complete'] and seal['protocol_sha256']==sha(D/'protocol.json') and result['prediction_seal_sha256']==sha(D/'prediction_seal.json')
 for r in seal['rows']:assert sha(r['path'])==r['sha256']
 row=cfg['rows'][3];assert row['sequence']=='E1A' and row['frame']==90
 selected=next(r for r in result['rows'] if r['sequence']=='E1A' and r['frame']==90)
 paths={arm:next(x['path'] for x in seal['rows'] if x['sequence']=='E1A' and x['frame']==90 and x['arm']==arm) for arm in ['unmasked','masked']};paths['original']=row['prediction_path']
 arrays={}
 for arm,p in paths.items():
  with np.load(p) as a:arrays[arm]={k:a[k] for k in a.files}
 origin=arrays['masked']['origin'];body=arrays['masked']['body_estimated']
 targets={};depth={};mask={};rgb={}
 with np.load(row['input_rgb_frame']) as a:
  for n in NAMES:
   rgb[n]=a[n+'_rgb'];z=a[n+'_depth_label'];T=a[n+'_camera_usd_world_label'];root=a['root_position_physics_label']
   with np.errstate(invalid='ignore'):world=(RAYS*z[...,None])@(T[:3,:3]@AX).T+T[:3,3]
   k=np.isfinite(z)&(z>=.5)&(z<=3)&~arrays['original'][n+'_CAD']&(np.linalg.norm(world-root,axis=2)>=.55)&(world[...,2]>.05)&(world[...,2]<=1.5)
   depth[n]=z;mask[n]=k;targets[n]=np.unique(np.rint((world[k,:2]-origin)/.1).astype(int),axis=0)*.1+origin
 def predicted(arm,n):return np.unique(np.rint((arrays[arm][n+'_world'][:,:2]-origin)/.1).astype(int),axis=0)*.1+origin
 allxy=np.concatenate(list(targets.values())+[predicted(a,n) for a in arrays for n in NAMES]);lo=np.min(allxy,axis=0)-.3;hi=np.max(allxy,axis=0)+.3
 def bev(ax,p,t,title):
  if len(t):ax.scatter(t[:,0],t[:,1],s=5,c='#b0b0b0',label='visible GT target')
  err=cKDTree(t).query(p)[0] if len(p) and len(t) else np.full(len(p),np.inf)
  if len(p):ax.scatter(p[:,0],p[:,1],s=12,c=np.where(err<=.15,'#16834b','#cc253d'),marker='x',linewidths=.8,label='prediction near/far')
  ax.scatter([body[0,3]],[body[1,3]],c='black',s=16,marker='^');ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),title=title,xlabel='world X (m)',ylabel='world Y (m)');ax.set_aspect('equal');ax.grid(alpha=.15)
 cmap=plt.colormaps['viridis'].copy();cmap.set_bad('#888888');ecmap=plt.colormaps['magma'].copy();ecmap.set_bad('#888888')
 fig,axs=plt.subplots(4,6,figsize=(24,13.4));d_im=e_im=None
 for i,n in enumerate(NAMES):
  m=selected['arms']['masked']['per_eye'][n];axs[i,0].imshow(rgb[n]);axs[i,0].set_title(n+' | actual RGB')
  d_im=axs[i,1].imshow(np.where(mask[n],depth[n],np.nan),vmin=.5,vmax=3,cmap=cmap);axs[i,1].set_title('GT target depth (m) | N='+str(m['target_pixels']))
  for j,arm in enumerate(['unmasked','masked'],2):
   a=arrays[arm];v=a[n+'_valid']&np.isfinite(a[n+'_depth'])&(a[n+'_depth']>=.5)&(a[n+'_depth']<=3)
   axs[i,j].imshow(np.where(v,a[n+'_depth'],np.nan),vmin=.5,vmax=3,cmap=cmap);axs[i,j].set_title(arm+' depth | valid '+str(int(v.sum())))
  a=arrays['masked'];v=a[n+'_valid']&np.isfinite(a[n+'_depth'])&(a[n+'_depth']>=.5)&(a[n+'_depth']<=3);err=np.where(mask[n]&v,abs(a[n+'_depth']-depth[n]),np.nan)
  e_im=axs[i,4].imshow(err,vmin=0,vmax=.5,cmap=ecmap);q=m['conditional_axial_error_m'];axs[i,4].set_title('masked |error|; K/N='+str(m['valid_target_pixels'])+'/'+str(m['target_pixels'])+'\nP90='+('N/A' if q is None else f"{q['P90']:.3f} m"))
  bev(axs[i,5],predicted('masked',n),targets[n],'masked per-eye BEV (current only)')
  for j in range(5):axs[i,j].axis('off')
 fig.suptitle('Fixed selection: fourth registered frame E1A / 090 / 45.0s | actual four cameras\nGray = UNKNOWN / outside the displayed GT target; no RGB semantics, historical map or navigation output',fontsize=14)
 fig.subplots_adjust(top=.91,bottom=.07,left=.015,right=.99,hspace=.25,wspace=.06)
 cb=fig.add_axes([.25,.035,.25,.012]);fig.colorbar(d_im,cax=cb,orientation='horizontal',label='axial depth (m)')
 cb=fig.add_axes([.65,.035,.25,.012]);fig.colorbar(e_im,cax=cb,orientation='horizontal',label='absolute axial error (m); clipped display at .5')
 fig.savefig(D/'E1A_090_four_eye_summary.png',dpi=130);plt.close(fig)
 fig,axs=plt.subplots(1,3,figsize=(17,6));target=np.unique(np.concatenate(list(targets.values())),axis=0)
 for ax,arm in zip(axs,['original','unmasked','masked']):
  p=np.unique(np.concatenate([predicted(arm,n) for n in NAMES]),axis=0);m=selected['arms'][arm]['union'];bev(ax,p,target,arm+f" | cells {m['predicted_cells']}/{m['target_cells']}\nP={m['precision']:.3f}, R={m['recall']:.3f}")
 fig.suptitle('Current four-eye joint BEV | gray: visible target; green: within .15m; red: farther from visible target\nRed is not a static free-space verdict. Historical memory / actual navigation: N/A.',fontsize=12);fig.tight_layout(rect=[0,0,1,.88]);fig.savefig(D/'E1A_090_joint_BEV.png',dpi=140);plt.close(fig)
 (D/'visualization_metadata.json').write_text(json.dumps(dict(fixed_selection='protocol.rows[3], E1A090',prediction_seal_sha256=sha(D/'prediction_seal.json'),result_sha256=sha(D/'result.json'),GT_read_after_complete_seal_and_score=True,scope='Presentation only; no source/scoring rerun',colors='depthviridis .5..3; errormagma0...5; unknown gray; BEVvisibleGTgray, near green, far red',unknown_not_free=True,historical_map=None,navigation=None),indent=2)+'\n')
if __name__=='__main__':main()
