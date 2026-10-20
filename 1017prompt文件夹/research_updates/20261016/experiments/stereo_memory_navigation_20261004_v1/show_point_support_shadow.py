"""Offline presentation only, after shadow maps and label assessment."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'point_support_runs'
a=json.loads((O/'assessment.json').read_text());p=json.loads((H/'point_support_protocol.json').read_text());ref=np.load(O/'reference_and_sweep.npz');origin=np.array(p['grid']['origin']);xy=origin+ref['whole_cells']*.1
end=json.loads((O/'frame_180.json').read_text());fig,axes=plt.subplots(1,2,figsize=(14,5),constrained_layout=True)
for ax,arm in zip(axes,['current','history']):
 ax.scatter(xy[:,0],xy[:,1],marker='s',s=18,c='#dadde0',label='333 required cells: UNKNOWN')
 r=ref['world_xy'];ax.plot(r[:,0],r[:,1],c='#2f5b91',lw=2,label='frozen turn + own stop reference')
 records=end['arms'][arm]['records'];pts=np.array([r['world'] for r in records]);ax.scatter(pts[:,0],pts[:,1],s=15,c='#dc7426',alpha=.5,label=f'{len(records)} endpoint records (not unique objects)')
 ax.set_title(arm+' map at18s: zero floor cells');ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='lower left',fontsize=8)
fig.suptitle('Same causal point inputs; memory retains surfaces away from the required sweep')
(h:=H/'figures').mkdir(exist_ok=True);fig.savefig(h/'point_support_spatial.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
for arm,c in [('current','#327cb2'),('history','#dd7626')]:
 rr=[r for r in a['rows'] if r['arm']==arm];t=[r['time_s'] for r in rr]
 axes[0].plot(t,[r['support']['whole']['unknown_cells'] for r in rr],color=c,label=arm)
 axes[1].plot(t,[r['records'] for r in rr],color=c,label=arm)
 axes[2].plot(t,[r['hazard_cells'] for r in rr],color=c,label=arm+' hazard proxy')
axes[0].set_ylim(0,350);axes[0].set_title('Required cells remain unknown');axes[0].set_ylabel('cells /333; no admission')
axes[1].set_title('Retained records; not covered area');axes[1].set_ylabel('records')
axes[2].plot(t,[0]*len(t),color='black',ls='--',label='floor cells (both)=0');axes[2].set_title('No floor support appears');axes[2].set_ylabel('endpoint proxy cells')
for ax in axes:ax.set_xlabel('original episode seconds');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('181 historical captures, readonly shadow; no new navigation run')
fig.savefig(h/'point_support_time.png',dpi=150);plt.close(fig)
# Pure descriptive metadata, no extra prediction or mapping pass.
packets=[json.loads((O/f'packet_{i:03d}.json').read_text()) for i in range(181)];pts=[v for r in packets for v in r['points']];z=np.array([v['world'][2] for v in pts])
r=dict(all_points=len(pts),nominal_floor_points=int((np.abs(z)<=.05).sum()),min_z_m=float(z.min()),max_z_m=float(z.max()),floor_gate_filtered_nominal_points=sum(abs(v['world'][2])<=.05 and abs(v['world'][2])+v['sigma_m']>.05 for v in pts),scene_body_id_unavailable=True)
(O/'height_descriptive.json').write_text(json.dumps(r,indent=2)+'\n');print(r)
