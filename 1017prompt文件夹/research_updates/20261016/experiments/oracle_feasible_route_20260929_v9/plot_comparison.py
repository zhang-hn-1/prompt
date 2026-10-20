from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent
fig,axes=plt.subplots(2,2,figsize=(12,11),constrained_layout=True)
for ax,suffix in zip(axes.flat,['0001','0002','0003','0004']):
 case=f'hard_0_e1_{suffix}'
 old=h/f'navigation_physical_baseline_{case}'/f'episode_{case}'
 new=h/f'navigation_hybrid_hard_{case}'/f'episode_{case}'
 a=np.load(old/'states.npz');b=np.load(new/'states.npz');ma=json.loads((old/'metrics.json').read_text());mb=json.loads((new/'metrics.json').read_text());manifest=json.loads((new/'manifest.json').read_text());episode=next(x for x in manifest['episodes'] if x['episode_id']==case);goal=np.asarray(episode['goal_position_world_m'][:2])
 for z,c,label in [(a,'#bb4b3a','Original PID Oracle'),(b,'#1d6faf','Measured-time Oracle')]:
  ref=z['kinodynamic_reference_pose'][:,:2];xy=z['position_world_xy'];ax.plot(ref[:,0],ref[:,1],'--',color=c,alpha=.45,linewidth=1.4)
  ax.plot(xy[:,0],xy[:,1],'-',color=c,linewidth=2,label=label)
 ax.scatter(*a['position_world_xy'][0],s=45,color='#318b4c',label='Start',zorder=5)
 ax.scatter(*goal,s=80,color='black',marker='*',label='Goal',zorder=5)
 ax.set_title(f'{case}: {ma["termination_reason"]} {ma["final_distance_m"]:.2f} m → {mb["termination_reason"]} {mb["final_distance_m"]:.2f} m')
 ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.axis('equal');ax.grid(alpha=.2)
handles,labels=axes.flat[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=4,bbox_to_anchor=(.5,-.025));fig.suptitle('Hard-0 E1: scored physical trajectories and corresponding references',fontsize=14)
fig.savefig(h/'hard0_e1_trajectories.png',dpi=160,bbox_inches='tight')
