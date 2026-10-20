from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;W=D.parents[1];C=W/'experiments/source_confidence_calibration_20261012_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';sys.path[:0]=[str(V),str(W/'experiments/stereo_memory_navigation_20261004_v1'),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm'];from viewpoint_confirmation import ViewpointConfirmationGrid
summaries=[]
for ep,name in [('e0','E0'),('e1','E1')]:
 R=C/f'data_{ep}_run';ev=np.load(D/f'{name}_evaluation.npz');m=np.load(R/'maps/frame_000.npz');g=ViewpointConfirmationGrid(m['origin'],10,static_scene_assumption=True);views={int(p.stem.split('_')[1]):dict(time_s=float(np.load(p)['time_s']),position_world=np.load(p)['left_camera_usd_world'][:3,3]) for p in (R/'known_poses').glob('*.npz')};native=json.loads((R/'native_predictions.json').read_text());poses={round(a['time_s'],7):a['position_world'] for a in native if a.get('available')};accepted={}
 for i in np.unique(ev['frame']):
  ix=ev['frame']==i;accepted[int(i)]=set(ev['query'][ix&ev['keep']].tolist())
 for i in range(900):
  p=R/'maps'/f'frame_{i:03d}.npz';a=np.load(p);t=float(a['time_s']);rs=json.loads((R/'raw_point_memory_snapshots'/f'frame_{i:03d}.json').read_text())['records'];filtered=[]
  for r in rs:
   k=np.array([int(q) in accepted[int(r['frame'])] for q in r['query_pixel_id']],bool);rr=dict(r)
   for field in ['world','query_pixel_id','partner_pixel_id']:rr[field]=np.asarray(r[field])[k].tolist()
   filtered.append(rr)
  g.consume_records(filtered,t,np.array(poses[round(t,7)]),views)
 actual=np.load(R/'maps/frame_899.npz');cfg=json.loads((C/f'data_{ep}_protocol.json').read_text());occ=np.load(cfg['occupancy']);truth=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(truth);oldxy=g.origin+np.argwhere(actual['labels']==1)*g.resolution;newxy=g.origin+np.argwhere(g.labels==1)*g.resolution;oldbad=tree.query(oldxy)[0]>.15;newbad=tree.query(newxy)[0]>.15;track=np.array([a['position'][:2] for a in json.loads((R/'states.json').read_text())]);domain=np.vstack([track,oldxy,newxy]);lo=domain.min(0)-2;hi=domain.max(0)+2
 fig,axs=plt.subplots(1,2,figsize=(12,5))
 for ax,xy,bad,title in zip(axs,[oldxy,newxy],[oldbad,newbad],['Actual saved map','Filtered-source replay, same trajectory']):
  ax.scatter(truth[:,0],truth[:,1],s=8,c='0.8',marker='s');ax.scatter(xy[:,0],xy[:,1],s=15,c=np.where(bad,'#d73027','#208b48'));ax.plot(track[:,0],track[:,1],lw=1,c='#2166ac');ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),aspect='equal',xlabel='X (m)',ylabel='Y (m)',title=title)
 fig.suptitle(f'{name}: counterfactual maps only — no new navigation, blank unknown');fig.tight_layout();fig.savefig(D/f'{ep}_map_replay.png',dpi=140);plt.close(fig);row=dict(sequence=name,actual_cells=len(oldxy),replay_cells=len(newxy),actual_far015m_cells=int(oldbad.sum()),replay_far015m_cells=int(newbad.sum()),scope='same saved physical trajectory/source; mapcounterfactual, not actual navigation; bodybandreference tolerance.15; no fullrecall');summaries.append(row);print(row,flush=True)
(D/'map_replay_results.json').write_text(json.dumps(summaries,indent=2)+'\n')
