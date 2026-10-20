"""Show every preregistered frame; no successful-frame selection."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
H=Path(__file__).resolve().parent
r=json.loads((H/'multiview_rgb_audit.json').read_text());data=np.load(H/'multiview_rgb_measurements.npz')
fig,axes=plt.subplots(4,3,figsize=(13,13),constrained_layout=True)
cmap=plt.get_cmap('viridis').copy();cmap.set_bad('#333333')
for i,row in enumerate(r['rows']):
 f=row['frame'];p=f'frame_{f:03d}__';pack=np.load(H/'multiview_rgb_inputs'/f'frame_{f:03d}.npz');pred=np.load(H/'multiview_rgb_predictions'/f'frame_{f:03d}.npz')
 axes[i,0].imshow(pack['left_rgb']);axes[i,0].set_title(f'{f:03d}: actual left RGB, t={row["time_s"]:.1f}s',fontsize=10)
 im=axes[i,1].imshow(pred['depth_m'],cmap=cmap,vmin=.5,vmax=3);axes[i,1].set_title(f'Final axial depth: {row["confident_image_pixels"]} image pixels\n{row["elapsed_cpu_wall_s"]:.1f}s offline CPU',fontsize=10)
 yy,xx=data[p+'source_pixels_yx'].T;v=np.zeros((480,640),np.uint8)
 v[yy,xx]=np.where(data[p+'confident'],np.where(data[p+'correct'],3,2),1)
 axes[i,2].imshow(v,cmap=ListedColormap(['#eeeeee','#aaaaaa','#dc614f','#39ad80']),vmin=0,vmax=3)
 axes[i,2].set_title(f'Full source queries: {row["metrics"]["n"]}\ncorrect {row["metrics"]["correct_n"]}, retained {row["metrics"]["confident_n"]}',fontsize=10)
 for a in axes[i]:a.axis('off')
fig.colorbar(im,ax=axes[:,1],shrink=.7,label='Final optical axial depth (m); dark = unknown')
fig.legend(handles=[Patch(color=c,label=l) for c,l in [('#eeeeee','not queried'),('#aaaaaa','query rejected / unknown'),('#dc614f','retained, ray error > .10m'),('#39ad80','retained, ray error <= .10m')]],loc='outside lower center',ncol=2,fontsize=9)
fig.suptitle('ONE frozen causal RGB+known-pose plane-sweep candidate: REJECTED, no navigation runs')
fig.savefig(H/'figures/multiview_rgb_all_frames.png',dpi=130);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
x=np.arange(4);n=np.array([a['metrics']['n'] for a in r['rows']])
for offset,key,label,color in [(-.3,'fast_confident_n','Fast retained','#888'),(-.1,'fast_correct_n','Fast correct','#7777bb'),(.1,'confident_n','V1 retained','#df9751'),(.3,'correct_n','V1 correct','#39ad80')]:
 y=np.array([a['metrics'][key] for a in r['rows']]);axes[0].bar(x+offset,y/n*100,width=.2,label=label,color=color)
axes[0].set_xticks(x,['000','010','020','023']);axes[0].set_ylabel('% of complete independent query denominator');axes[0].legend(fontsize=8)
axes[0].set_title('Matched-centre comparison: rejections remain in denominator')
for offset,key,label,color in [(-.15,'fast_ray_error_p90_m','Fast','#888'),(.15,'candidate_ray_error_p90_m','V1','#df9751')]:
 y=[a['metrics'][key] if a['metrics'][key] is not None else np.nan for a in r['rows']];axes[1].bar(x+offset,y,width=.3,label=label,color=color)
axes[1].axhline(.1,color='#39ad80',ls='--',label='frozen .10m gate');axes[1].set_xticks(x,['000','010','020','023']);axes[1].set_ylabel('Retained centre ray error p90 (m)');axes[1].legend(fontsize=8)
axes[1].set_title('Fast 020/023 absent: no retained query predictions')
fig.suptitle('Source-sphere scope only; confident score is not a correctness certificate')
fig.savefig(H/'figures/multiview_rgb_comparison.png',dpi=140);plt.close(fig)
print('SAVED all4 actual frame/complete-query figures')
