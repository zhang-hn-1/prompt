"""Measured scientific figures; predictions are already persisted."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Circle
H=Path(__file__).resolve().parent;F=H/'figures';F.mkdir(exist_ok=True)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']=font.get_name();plt.rcParams['axes.unicode_minus']=False
assessment=json.loads((H/'run_assessment.json').read_text())
fig,axes=plt.subplots(2,2,figsize=(12,11))
for ax,label,title in zip(axes.ravel(),('e0','e2','h3','h4'),('E0 简单回归','E2 窄通道','E1 困难起点 0003','E1 困难起点 0004')):
    maproot=H/'runs/map'/label;known=np.load(maproot/'truth_map.npz');ref=np.load(H/f'common_reference_{label}.npz')['position_world_xy']
    origin=known['origin'];res=float(known['resolution']);grid=known['labels']
    ax.imshow(grid.T,origin='lower',extent=[origin[0]-.5*res,origin[0]+(grid.shape[0]-.5)*res,origin[1]-.5*res,origin[1]+(grid.shape[1]-.5)*res],cmap=ListedColormap(['#f4f4f4','#253547','#aaa']),vmin=0,vmax=2)
    ax.plot(ref[:,0],ref[:,1],'--',color='#999',lw=1.2,label='固定几何评估参考')
    allpts=[ref]
    for mode,color,name in [('map','#147d48','完整地图'),('visible','#d88813','可见真值深度')]:
        root=H/'runs'/mode/label;ep=next(root.glob('episode_*'));s=np.load(ep/'states.npz');xy=s['position_world_xy'];allpts.append(xy)
        r=assessment['runs'][mode+'_'+label]
        ax.plot(xy[:,0],xy[:,1],color=color,lw=2,label=name)
        ax.scatter(*xy[-1],color=color,marker='x' if r['termination']=='collision' else 'o',s=70,zorder=5)
    ax.scatter(*ref[0],marker='s',color='#2366ad',s=45,label='起点');ax.scatter(*ref[-1],marker='*',color='#754ead',s=100,label='目标')
    pts=np.concatenate(allpts);ax.set_xlim(pts[:,0].min()-1.3,pts[:,0].max()+1.3);ax.set_ylim(pts[:,1].min()-1.3,pts[:,1].max()+1.3)
    ax.set_aspect('equal');ax.set_title(title+'\n完整地图'+('到达' if label in ('e0','e2') else '碰撞')+' / 可见真值深度超时');ax.set_xlabel('世界 X / m');ax.set_ylabel('世界 Y / m')
fig.suptitle('8 条实际运行：相同控制器、初始状态与 35 秒规则',fontsize=15);fig.legend(*axes[0,0].get_legend_handles_labels(),loc='lower center',ncol=5,fontsize=9);fig.tight_layout(rect=(0,.06,1,.95));fig.subplots_adjust(hspace=.40);fig.savefig(F/'trajectories.png',dpi=160);plt.close(fig)

from analyze_runs import initial_belief
fig,axes=plt.subplots(1,4,figsize=(16,4.5))
for ax,label in zip(axes,('e0','e2','h3','h4')):
    b=initial_belief(H/'runs/visible'/label);f=np.load(H/'runs/visible'/label/'online_inputs/frame_000.npz');xy=f['position_world_xy'];ref=np.load(H/f'common_reference_{label}.npz')['position_world_xy']
    origin=b.origin-xy;ax.imshow(b.labels.T,origin='lower',extent=[origin[0]-.05,origin[0]+39.95,origin[1]-.05,origin[1]+39.95],cmap=ListedColormap(['#aaa','#eef6e8','#e4554c']),vmin=-1,vmax=1)
    ax.plot(ref[:,0]-xy[0],ref[:,1]-xy[1],color='#2366ad',lw=1.5)
    ax.add_patch(Circle((0,0),.4,fill=False,color='#111',lw=2));ax.add_patch(Circle((0,0),.6,fill=False,color='#7d49a1',ls='--'))
    c=assessment['initial_nominal_tube_coverage'][label]
    ax.set_title(f'{label.upper()}：首米管道已知空闲 {100*c["known_free_fraction"]:.1f}%')
    ax.set_xlim(-2,2);ax.set_ylim(-2,2);ax.set_aspect('equal');ax.set_xlabel('相对起点 X / m');ax.set_ylabel('相对起点 Y / m')
fig.suptitle('首帧可见深度地图：灰色未知、浅绿色地面点与机体占地、红色障碍点\n实线圆为球体半径 0.4 m；虚线圆为所设净空 0.6 m',fontsize=13);fig.tight_layout();fig.savefig(F/'initial_observability.png',dpi=160);plt.close(fig)

depth=json.loads((H/'depth_assessment.json').read_text());seqs=['e0','e1','e2','hard_0_e1_0003','hard_0_e1_0004','direct_e1'];short=['E0','E1','E2','H3','H4','Direct E1']
fig,axes=plt.subplots(1,2,figsize=(12,4.3));xs=np.arange(len(seqs))
for i,model in enumerate(('da3_metric','unidepth_v2')):
    vals=[depth[model]['sequences'].get(s,{}).get('obstacle',{}) for s in seqs]
    p=[v.get('p90_m') or np.nan for v in vals];q=[(v.get('within_0p3m') or 0)*100 for v in vals]
    axes[0].bar(xs+(i-.5)*.35,p,.35,label=model);axes[1].bar(xs+(i-.5)*.35,q,.35,label=model)
for ax in axes:ax.set_xticks(xs,short);ax.legend(fontsize=9);ax.grid(axis='y',alpha=.2)
axes[0].axhline(.3,color='red',ls='--');axes[0].set_ylabel('障碍点三维误差 p90 / m')
axes[1].axhline(90,color='red',ls='--');axes[1].set_ylabel('三维误差 ≤0.30 m 的比例 / %');axes[1].set_ylim(0,103)
fig.suptitle('历史开发序列离线评测：每个模型 66 对同步图像，无真值缩放',fontsize=13);fig.tight_layout();fig.savefig(F/'depth_errors.png',dpi=160);plt.close(fig)

manifest=json.loads((H/'offline_manifest.json').read_text());row=next(r for r in manifest['samples'] if r['sequence']=='hard_0_e1_0004')
rgb=np.load(row['rgb_file']);truth=np.load(row['truth_file']);models={m:np.load(H/'predictions'/m/row['sequence']/Path(row['rgb_file']).name) for m in ('da3_metric','unidepth_v2') if (H/'predictions'/m/row['sequence']/Path(row['rgb_file']).name).exists()}
fig,axes=plt.subplots(2,4,figsize=(10,10))
for i,eye in enumerate(('left','right')):
    axes[i,0].imshow(np.rot90(rgb[eye+'_rgb']));axes[i,0].set_title(('左' if i==0 else '右')+'相机 RGB（软件转正）')
    d=truth[eye+'_depth_m'];d=np.where(np.isfinite(d)&(d>0),d,np.nan)
    images=[np.rot90(d)]+[np.rot90(models[m][eye+'_depth_m']) if m in models else np.zeros((640,480)) for m in ('da3_metric','unidepth_v2')]
    for j,(im,title) in enumerate(zip(images,('渲染真值 z 深度','DA3 米制深度','UniDepthV2 米制深度')),1):
        chart=axes[i,j].imshow(im,vmin=.5,vmax=3,cmap='viridis');axes[i,j].set_title(title)
for ax in axes.ravel():ax.axis('off')
fig.suptitle('困难起点 0004 首帧：相机物理位置保持不变；色标单位 m',fontsize=13);fig.tight_layout(rect=(0,0,.95,.96));cax=fig.add_axes([.96,.13,.015,.7]);fig.colorbar(chart,cax=cax)
fig.savefig(F/'depth_views_h4.png',dpi=150,bbox_inches='tight');plt.close(fig)
