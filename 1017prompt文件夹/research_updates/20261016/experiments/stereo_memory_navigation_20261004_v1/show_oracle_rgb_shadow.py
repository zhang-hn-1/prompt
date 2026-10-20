from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'oracle_rgb_shadow_run';a=json.loads((H/'oracle_rgb_shadow_assessment.json').read_text());n=np.load(O/'episode_arrays.npz');r=np.load(H/'oracle_rgb_shadow_reference.npz');occ=np.load(O/'offline_occupancy_label.npz');x=n['positions_world_xy'];origin=occ['origin'];res=float(occ['resolution']);grid=occ['labels'];ii,jj=np.nonzero(grid>0);xy=origin+np.c_[ii,jj]*res
(H/'figures').mkdir(exist_ok=True)
f,ax=plt.subplots(1,2,figsize=(12,5),dpi=150)
ax[0].scatter(xy[:,0],xy[:,1],s=5,c='0.85',label='Offline known-map obstacles');ax[0].plot(r['pose'][:,0],r['pose'][:,1],'--',c='tab:orange',label='Frozen Oracle reference');ax[0].plot(x[:,0],x[:,1],c='tab:blue',label='Actual current-rig Oracle');ax[0].scatter(*x[0],c='green',marker='o',s=60,label='Start');ax[0].scatter(*x[-1],c='blue',marker='o',s=45,label='End');goal=np.array([3.3,0.]);ax[0].scatter(*goal,c='red',marker='*',s=110,label='Goal');ax[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Oracle success: 30.7s, no contact');ax[0].axis('equal');ax[0].set_xlim(-4,4);ax[0].set_ylim(-3.1,3.1);ax[0].legend(fontsize=7)
t=[v['time_s'] for v in a['source_rows']];ax[1].plot(t,[v['records'] for v in a['memory_rows']],label='Retained query records');ax[1].plot(t,[v['correct'] for v in a['memory_rows']],label='Offline-labelled accurate records');ax[1].plot(t,[v['floor_cells'] for v in a['map_rows']],label='Provisional floor cells');ax[1].set(xlabel='Episode time (s)',ylabel='Count (different quantities)',title='Read-only RGB memory; no control admission');ax[1].legend(fontsize=8)
f.tight_layout();f.savefig(H/'figures/oracle_online_rgb_shadow.png');plt.close(f)
