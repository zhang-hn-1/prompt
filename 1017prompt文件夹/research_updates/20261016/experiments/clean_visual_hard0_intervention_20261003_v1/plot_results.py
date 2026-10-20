from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).parent;E=H.parent
old=json.loads((H/'acquisition_pair/original/camera_clocks.json').read_text());new=json.loads((H/'synchronized_v2/hard_0_e1_0003/camera_clocks.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(12,4))
for name,records,style in [('Original',old,'r.-'),('Synchronized',new,'b.-')]:
 t=np.array([v['episode_time_s'] for v in records]);age=np.array([v['eyes']['left']['age_s'] for v in records]);axes[0].plot(t[t<=2],age[t<=2]*1000,style,label=name)
 elapsed=np.array([v['eyes']['left']['rendering_time_s'] for v in records]);axes[1].step(t,elapsed-elapsed[0],where='post',label=name)
axes[0].set(xlabel='Episode time (s)',ylabel='RGB age relative to measured state (ms)',title='Actual SDK camera clock');axes[0].legend();axes[0].grid(alpha=.3)
axes[1].set(xlabel='Episode time (s)',ylabel='Acquired image time since first image (s)',title='Old 5 Hz effective acquisition / corrected 10 Hz');axes[1].legend();axes[1].grid(alpha=.3)
fig.tight_layout();fig.savefig(H/'camera_time_alignment.png',dpi=160);plt.close(fig)
fig,axes=plt.subplots(1,5,figsize=(16,4.6))
for ax,label in zip(axes,['e0','e1','e2','hard_0_e1_0003','hard_0_e1_0004']):
 episode=label if label.startswith('hard') else 'golden_'+label+'_0001';root=H/'synchronized_v2'/label;prev=(H/'baseline'/label if label.startswith('hard') else E/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean'/label)
 f=np.load(root/'online_inputs/frame_000.npz');yaw=float(f['yaw_world_rad']);c,s=np.cos(yaw),np.sin(yaw);rot=np.array([[c,-s],[s,c]]);origin=f['position_world_xy'];goal=f['pointgoal_base_xy'];m=json.loads((root/f'episode_{episode}/metrics.json').read_text())
 for directory,style,color,name in [(prev,'--','gray','Original B0'),(root,'-','royalblue','Synchronized B0')]:
  state=np.load(directory/f'episode_{episode}/states.npz');xy=(state['position_world_xy']-origin)@rot;ax.plot(xy[:,0],xy[:,1],style,color=color,lw=2,label=name)
 ax.scatter([0],[0],s=24,c='black');ax.scatter([goal[0]],[goal[1]],marker='*',s=90,c='orange');ax.scatter(xy[-1,0],xy[-1,1],marker='o' if m['success'] else 'x',s=65,c='green' if m['success'] else 'red')
 ax.set_title(label.replace('hard_0_e1_','Hard E1 #')+'\n'+m['termination_reason']+f" {m['elapsed_s']:.1f}s");ax.set_xlabel('Forward from start (m)');ax.set_ylabel('Left from start (m)');ax.axis('equal');ax.grid(alpha=.25)
axes[0].legend(fontsize=8);fig.tight_layout();fig.savefig(H/'actual_navigation_regression.png',dpi=150)
