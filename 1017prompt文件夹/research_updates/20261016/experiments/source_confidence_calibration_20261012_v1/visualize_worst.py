from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
for ep in ['e0','e1']:
 R=D/f'data_{ep}_run';rows=[]
 for p in sorted((R/'source_predictions').glob('*.npz')):
  a=np.load(p);f=np.load(R/'frames'/p.name);z=a['depth_m'];gt=f['left_depth_m'];v=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(gt)&(gt>0)
  if v.sum()>=1000:rows.append(dict(frame=p.stem,time_s=float(a['time_s']),pixels=int(v.sum()),p90_m=float(np.quantile(np.abs(z[v]-gt[v]),.9))))
 row=max(rows,key=lambda r:r['p90_m']);name=row['frame']+'.npz';a=np.load(R/'source_predictions'/name);f=np.load(R/'frames'/name);z=a['depth_m'];gt=f['left_depth_m'];v=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(gt)&(gt>0)
 fig,axs=plt.subplots(1,4,figsize=(16,4));axs[0].imshow(f['left_rgb']);axs[0].set_title('Left RGB');axs[1].imshow(np.where(v,z,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[1].set_title('Original stereo depth (m)');axs[2].imshow(np.where(np.isfinite(gt)&(gt>0),gt,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[2].set_title('True depth (color capped at3m)');im=axs[3].imshow(np.where(v,np.abs(z-gt),np.nan),vmin=0,vmax=.3,cmap='magma');axs[3].set_title('Error (color capped at0.3m)');fig.colorbar(im,ax=axs[3],shrink=.65)
 for ax in axs:ax.axis('off')
 fig.suptitle(f'{ep.upper()} worst conditional depth p90 epoch: {row["time_s"]:.1f}s, p90={row["p90_m"]:.3f}m, n={row["pixels"]} (not whole-run average)');fig.tight_layout();fig.savefig(D/f'{ep}_worst_depth.png',dpi=140);plt.close(fig);(D/f'{ep}_depth_epoch_metrics.json').write_text(json.dumps(dict(worst=row,eligible_epoch_rows=rows,selection='maximum per-epoch conditional p90 among >=1000 valid confident0.5-3m pixels; reporting only'),indent=2)+'\n');print(ep,row)
