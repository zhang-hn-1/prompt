from paths import *
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=json.loads((D/'comparison.json').read_text());B=W/'experiments/four_camera_weak_baseline_20261017_v1';cfg=json.loads((D/'protocol.json').read_text());occ=np.load(cfg['occupancy']);xy=np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy']
fig,axs=plt.subplots(2,3,figsize=(14,8));colors={'A':'royalblue','B':'darkorange'}
for name,root in [('A',D),('B',B)]:
    v=json.loads((root/'e1_result.json').read_text());ph=np.load(root/'e1_run/physics.npz');pos=ph['root_position_world_m'];ep=json.loads((root/'e1_run/episode.json').read_text());goal=ep['goal_position_world_m']
    axs[0,0].plot(pos[:,0],pos[:,1],color=colors[name],label=name+' '+v['termination']);axs[0,0].scatter(goal[0],goal[1],marker='*',c='black')
    ts=[x['time_s'] for x in v['source_rows']];axs[0,1].plot(ts,[x['covered_cells']/x['target_cells'] if x['target_cells'] else np.nan for x in v['source_rows']],color=colors[name],label=name)
    axs[0,2].plot(np.arange(len(v['false_map_cells_by_policy']))*.1,v['false_map_cells_by_policy'],color=colors[name],label=name)
axs[0,0].scatter(xy[:,0],xy[:,1],s=.5,c='.8');axs[0,0].axis('equal');axs[0,0].set(xlim=(-5,5),ylim=(-4,4),title='Actual development trajectories',xlabel='worldX(m)',ylabel='worldY(m)');axs[0,0].legend(fontsize=8)
axs[0,1].set(title='Visible target recall; domains diverge',xlabel='Simulationtime(s)',ylabel='Coverage',ylim=(0,1));axs[0,1].legend()
axs[0,2].set(title='GT-false risk cells retained',xlabel='Simulationtime(s)',ylabel='Cells');axs[0,2].legend()
for ax,key,title,unit in [(axs[1,0],'net_goal_progress_m','Net goalprogress','m'),(axs[1,1],'source_supported_blind_missing_fraction','Source-supported blind target missing','fraction'),(axs[1,2],'false_map_cell_seconds','False risk persistence','cell-seconds')]:
    values=[r['arms'][n][key] for n in ['A','B']];ax.bar(['A current only','B existing memory'],values,color=[colors[n] for n in ['A','B']]);ax.set(title=title,ylabel=unit)
    for j,value in enumerate(values):ax.text(j,value,f'{value:.4g}',ha='center',va='bottom',fontsize=9)
fig.suptitle('One prospective A vs known prior B; fixed4camera/source/pose/planner/control\nDevelopment only: action/RGB paths diverge; C rejected offline, no full3arm or learning claim. Missing=unknown.',fontsize=12)
fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(D/'A_B_development_comparison.png',dpi=140);plt.close(fig)
