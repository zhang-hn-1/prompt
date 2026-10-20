from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
STAGE=Path(__file__).resolve().parent;R=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/wm_aux30_independent_calibration_20260926_v1');jobs=json.loads((R/'collection_manifest.json').read_text())['jobs']
fig,axes=plt.subplots(2,2,figsize=(9,8),layout='constrained')
for ax,job in zip(axes.flat,jobs):
 with np.load(Path(job['output'])/'ep0000/states.npz') as z:
  pos=z['pos'][:,:2];q=z['quat'][0];coll=z['collision'].astype(bool);frame_ids=np.searchsorted(z['physics_step'],z['stereo_physics_step']);assert np.array_equal(z['physics_step'][frame_ids],z['stereo_physics_step'])
 yaw=np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2));rot=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]]);xy=(pos-pos[0])@rot.T
 ax.plot(xy[:,0],xy[:,1],color='#2178ad',lw=1.6,label='Measured trajectory');ax.scatter(xy[0,0],xy[0,1],c='#16866b',s=50,label='First measured point');ax.scatter(xy[-1,0],xy[-1,1],c='black',marker='x',s=45,label='End (not a navigation goal)')
 if coll.any():ax.scatter(xy[frame_ids[coll],0],xy[frame_ids[coll],1],c='#ba4e4e',s=12,label='Geometric contact label')
 ax.set(title=job['id'].replace('_cal_',' / '),xlabel='Initial-heading x (m)',ylabel='Initial-heading y (m)');ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.legend(fontsize=7)
fig.suptitle('Four new 24 s calibration sequences | actual motion',fontsize=14);fig.savefig(STAGE/'calibration_trajectories.png',dpi=160);plt.close(fig)
