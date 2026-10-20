"""Measured scientific figures, fixed geometry and current camera frames."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle
H=Path(__file__).resolve().parent;OLD=H.parent/'visual_information_new_rig_20261004_v2';F=H/'figures';F.mkdir(exist_ok=True)
plt.rcParams['font.family']=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name();plt.rcParams['axes.unicode_minus']=False
A=json.loads((H/'initial_support_audit.json').read_text());names={'e0':'E0','e2':'E2','h3':'Hard 0003','h4':'Hard 0004'}
fig,axes=plt.subplots(4,2,figsize=(11,15))
for row,case in enumerate(names):
    inp=np.load(OLD/'runs/visible'/case/'online_inputs/frame_000.npz');root=inp['position_world_xy'];yaw=float(inp['yaw_world_rad']);r=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    x,y=np.meshgrid(np.arange(-1.2,2.81,.1),np.arange(-1.5,1.51,.1),indexing='xy');world=np.stack((x,y),axis=-1)@r.T+root
    for col,arm in enumerate(('points','surface')):
        d=np.load(H/f'initial_belief_{case}_{arm}.npz');cells=np.rint((world-d['origin'])/.1).astype(int);labels=d['labels'][cells[...,0],cells[...,1]];ax=axes[row,col]
        ax.pcolormesh(x,y,labels+1,cmap=ListedColormap(['#c7cbd0','#fcfcfc','#d64a48']),vmin=0,vmax=2,shading='nearest')
        ax.add_patch(Circle((0,0),.4,fill=False,color='#2070b4',linewidth=2,label='已知机体占地 0.4m'))
        ax.add_patch(Circle((0,0),.6,fill=False,color='#d98915',linestyle='--',linewidth=2,label='要求安全半径 0.6m'))
        ax.plot(.5,0,'x',color='black',ms=9,mew=2,label='前方0.5m未知证据点')
        ax.axvline(A[case]['nearest_observed_floor_forward_in_1p2m_band_m'],color='#44945a',linestyle=':',label='最邻近有效前方地面')
        ax.set_aspect('equal');ax.set_xlim(-1.2,2.8);ax.set_ylim(-1.5,1.5)
        ax.set_title(names[case]+('：地面落点' if arm=='points' else '：连续地面补全')+f"，自由格 {A[case]['representations'][arm]['free_cells']}")
        ax.set_xlabel('相对机体前方 / m');ax.set_ylabel('相对机体左方 / m')
        if row==0 and col==0:ax.legend(fontsize=8,loc='lower right')
fig.suptitle('相同当前观测：白色为已有自由空间，灰色为未知，红色为占据\n补全观测内空洞，没有填补机体前方盲区；两组起点安全净空均为0.412m',fontsize=14)
fig.tight_layout(rect=(0,0,1,.94));fig.savefig(F/'initial_map_support.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,4))
x=np.arange(4)
for i,(key,label_,color) in enumerate([('points','地面落点','#477cb7'),('surface','连续地面','#dc9435'),('dense_points','全部原始像素','#619c66')]):
 axes[0].bar(x+(i-1)*.25,[A[k]['representations'][key]['free_cells'] for k in names],.25,label=label_,color=color)
axes[0].set_xticks(x,list(names.values()));axes[0].set_ylabel('初始已知自由格数');axes[0].legend(fontsize=9)
axes[1].bar(x,[A[k]['representations']['surface']['start_clearance_m'] for k in names],color='#dc9435');axes[1].axhline(.6,c='red',ls='--',label='要求0.6m');axes[1].set_ylim(0,.75);axes[1].set_xticks(x,list(names.values()));axes[1].set_ylabel('起点观测自由区净空 / m');axes[1].legend()
fig.suptitle('更多观测自由格 ≠ 已有安全起步通路');fig.tight_layout();fig.savefig(F/'support_counts.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(10,4.5))
for i,z in enumerate(('0.05','0.4','0.8')):
 ax.bar(x+(i-1)*.25,[A[k]['visibility_probes'][z]['los_fraction']*100 for k in names],.25,label='世界高度 '+z+'m')
ax.set_xticks(x,list(names.values()));ax.set_ylim(0,105);ax.set_ylabel('双目至少一眼有当前视线证据 / %');ax.legend();ax.grid(axis='y',alpha=.2)
ax.set_title('前方0.4–1.1m、左右±0.6m：每层104个空间探针\n视线比例只是观测诊断，不能等同整个机器人碰撞体积已清空');fig.tight_layout();fig.savefig(F/'near_field_visibility.png',dpi=160);plt.close(fig)
if (H/'run_assessment.json').exists():
 R=json.loads((H/'run_assessment.json').read_text())['runs'];fig,axes=plt.subplots(2,2,figsize=(11,9))
 for ax,case in zip(axes.ravel(),names):
  ref=np.load(OLD/f'common_reference_{case}.npz')['position_world_xy'];origin=ref[0]
  ax.plot(*(ref-origin).T,c='#777777',ls='--',label='冻结参考，仅离线评估')
  for arm,color,ls in [('points','#467bbc','-'),('surface','#e59528','--')]:
   ep=next((H/'runs'/arm/case).glob('episode_*'));s=np.load(ep/'states.npz');xy=s['position_world_xy']-origin;ax.plot(*xy.T,c=color,ls=ls,linewidth=2,label='落点基线' if arm=='points' else '连续地面候选')
  lo=np.minimum((ref-origin).min(0),[-.8,-.8])-.3;hi=np.maximum((ref-origin).max(0),[.8,.8])+.3
  center=(lo+hi)/2;span=max(hi-lo)
  ax.set_xlim(center[0]-span/2,center[0]+span/2);ax.set_ylim(center[1]-span/2,center[1]+span/2)
  ax.set_title(names[case]);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('世界X位移 / m');ax.set_ylabel('世界Y位移 / m');ax.legend(fontsize=8)
 fig.suptitle('八条重新运行的35秒闭环对照：真实机体轨迹\n参考来自固定初始全图，仅用于相同离线评价',fontsize=13);fig.tight_layout(rect=(0,0,1,.93));fig.savefig(F/'trajectories.png',dpi=160);plt.close(fig)
 fig,axes=plt.subplots(2,2,figsize=(12,8))
 for ax,case in zip(axes.ravel(),names):
  for arm,color in [('points','#467bbc'),('surface','#e59528')]:
   rows=R[arm+'_'+case]['support_over_time'];ax.plot([v['time_s'] for v in rows],[v['known_free'] for v in rows],c=color,label=arm)
  ax.set_title(names[case]);ax.set_xlabel('实际仿真时间 / s');ax.set_ylabel('累计已知自由格数');ax.grid(alpha=.2);ax.legend()
 fig.suptitle('连续帧记忆下的地图变化（动作是否改变需看独立指令记录）');fig.tight_layout();fig.savefig(F/'temporal_map_support.png',dpi=160);plt.close(fig)
print('Figures saved',F)
