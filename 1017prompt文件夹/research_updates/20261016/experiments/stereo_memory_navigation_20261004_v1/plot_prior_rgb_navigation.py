from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
s=json.loads((H/'prior_rgb_navigation_assessment_series.json').read_text());a=json.loads((H/'prior_rgb_navigation_assessment.json').read_text())
fig,ax=plt.subplots(1,2,figsize=(11,4.5));colors={'current':'tab:blue','history':'tab:orange'}
for arm,v in s.items():
 xy=np.array([r['position'][:2] for r in v['states']]);t=np.array([r['time_s'] for r in v['states']]);g=np.array(v['task']['actual_episode']['goal_position_world_m'][:2]);start=np.array(v['task']['actual_episode']['start_position_world_m'][:2]);u=(g-start)/np.linalg.norm(g-start)
 ax[0].plot(xy[:,0],xy[:,1],color=colors[arm],label=f"{arm}: {a['arms'][arm]['termination']}");ax[1].plot(t,(xy-start)@u,color=colors[arm],label=arm)
ax[0].scatter(*start,c='k',marker='o',label='start');ax[0].scatter(*g,c='green',marker='*',s=100,label='local goal');ax[0].add_patch(plt.Circle(g,.2,fill=False,color='green'));ax[0].set_aspect('equal');ax[0].set(xlabel='world X (m)',ylabel='world Y (m)',title='Actual robot trajectories');ax[1].axhline(2,color='green',ls='--');ax[1].set(xlabel='simulation seconds',ylabel='actual progress along goal line (m)',title='2 m task; known pose; explicit unknown exploration')
for x in ax:x.legend();x.grid(alpha=.2)
fig.tight_layout();fig.savefig(H/'prior_rgb_navigation_trajectories.png',dpi=150);plt.close(fig)
fig,ax=plt.subplots(2,3,figsize=(12,7))
for j,arm in enumerate(('current','history')):
 O=H/('prior_rgb_'+arm+'_run');r=json.loads((O/'policy_records.json').read_text());i=len(r)-1;mp=np.load(O/'maps'/f'frame_{i:03d}.npz');src=r[i]['local']['source_frame'];rgb=np.load(O/'online_inputs'/f'frame_{src:03d}.npz')
 ax[j,0].imshow(rgb['left_rgb']);ax[j,0].set_title(f'{arm}: source {src} left RGB');ax[j,1].imshow(rgb['right_rgb']);ax[j,1].set_title('right RGB')
 origin=mp['origin'];res=float(mp['resolution']);xy=np.array([q['position'][:2] for q in s[arm]['states']]);c=np.rint((xy[-1]-origin)/res).astype(int);lo=np.maximum(c-35,0);hi=np.minimum(c+36,mp['labels'].shape);data=mp['provenance'][lo[0]:hi[0],lo[1]:hi[1]];extent=[origin[0]+lo[0]*res,origin[0]+hi[0]*res,origin[1]+lo[1]*res,origin[1]+hi[1]*res]
 from matplotlib.colors import ListedColormap,BoundaryNorm
 cmap=ListedColormap(['#dddddd','#a2e0b1','#427d49','#fd8c70','#943530']);ax[j,2].imshow(data.T,origin='lower',extent=extent,cmap=cmap,norm=BoundaryNorm(np.arange(-.5,5.5),5));ax[j,2].plot(xy[:,0],xy[:,1],c='blue');ax[j,2].set_title('Unknown/current/history source labels\npriors are not observed free');ax[j,2].set_aspect('equal')
 for x in ax[j,:2]:x.axis('off')
fig.tight_layout();fig.savefig(H/'prior_rgb_navigation_sources_and_maps.png',dpi=150);plt.close(fig)
print('two actual-data plots saved')
