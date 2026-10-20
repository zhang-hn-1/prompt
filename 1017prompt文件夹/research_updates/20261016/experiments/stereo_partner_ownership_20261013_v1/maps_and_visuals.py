from pathlib import Path
import sys,json,hashlib,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;W=D.parents[1];C=W/'experiments/source_confidence_calibration_20261012_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';sys.path[:0]=[str(V),str(W/'experiments/stereo_memory_navigation_20261004_v1'),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from viewpoint_confirmation import ViewpointConfirmationGrid
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs={};outs={};replays=[]
for ep,name in [('e0','E0'),('e1','E1')]:
 R=C/f'data_{ep}_run';m=np.load(R/'maps/frame_000.npz');grids=[ViewpointConfirmationGrid(m['origin'],10,static_scene_assumption=True) for _ in range(2)];views={}
 for p in sorted((R/'known_poses').glob('*.npz')):
  a=np.load(p);views[int(p.stem.split('_')[1])]=dict(time_s=float(a['time_s']),position_world=a['left_camera_usd_world'][:3,3]);inputs[str(p)]=sha(p)
 nativepath=R/'native_predictions.json';poses={round(a['time_s'],7):a['position_world'] for a in json.loads(nativepath.read_text()) if a.get('available')};inputs[str(nativepath)]=sha(nativepath);accepted={}
 for p in sorted((D/'predictions').glob(f'{name}_*.npz')):
  a=np.load(p);accepted[int(a['frame'])]=set(a['query'][a['keep']].tolist())
 for i in range(900):
  p=R/'maps'/f'frame_{i:03d}.npz';a=np.load(p);t=float(a['time_s']);rp=R/'raw_point_memory_snapshots'/f'frame_{i:03d}.json';rs=json.loads(rp.read_text())['records'];filtered=[]
  for r in rs:
   k=np.array([int(q) in accepted[int(r['frame'])] for q in r['query_pixel_id']],bool);rr=dict(r)
   for field in ['world','query_pixel_id','partner_pixel_id']:rr[field]=np.asarray(r[field])[k].tolist()
   filtered.append(rr)
  for g,records in zip(grids,[rs,filtered]):g.consume_records(records,t,np.asarray(poses[round(t,7)]),views)
  assert np.array_equal(grids[0].labels,a['labels']), (name,i,'baseline replay mismatch')
  inputs[str(p)]=sha(p);inputs[str(rp)]=sha(rp)
 for arm,g in zip(['before','after'],grids):
  p=D/f'{name}_{arm}_map.npz';np.savez_compressed(p,origin=g.origin,resolution=g.resolution,labels=g.labels,clock=g.hazard_acquisition_time,time_s=g.time);outs[str(p)]=sha(p)
 replays.append(dict(sequence=name,exact_baseline_maps=900));print(name,'900 exact baseline and filtered maps',flush=True)
(D/'map_prediction_seal.json').write_text(json.dumps(dict(inputs=inputs,outputs=outs,source={str(p):sha(p) for p in [Path(__file__),V/'viewpoint_confirmation.py',W/'experiments/stereo_memory_navigation_20261004_v1/repeated_exposure_memory_v1.py',W/'experiments/stereo_memory_navigation_20261004_v1/partial_point_risk_grid.py']},ownership_seal_sha=sha(D/'prediction_seal.json'),ownership_outputs=json.loads((D/'prediction_seal.json').read_text())['output_sha256'],truth_read=False,replays=replays),indent=2)+'\n')
# All counterfactual maps sealed before reference and GT scoring.
summaries=[]
for ep,name in [('e0','E0'),('e1','E1')]:
 R=C/f'data_{ep}_run';cfg=json.loads((C/f'data_{ep}_protocol.json').read_text());occ=np.load(cfg['occupancy']);truth=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(truth);track=np.array([a['position'][:2] for a in json.loads((R/'states.json').read_text())]);goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2];rows=[];fig,axs=plt.subplots(1,3,figsize=(15,5))
 for ax,arm in zip(axs[:2],['before','after']):
  m=np.load(D/f'{name}_{arm}_map.npz');cells=np.argwhere(m['labels']==1);xy=m['origin']+cells*float(m['resolution']);old=float(m['time_s'])-m['clock'][tuple(cells.T)]>10;bad=tree.query(xy)[0]>.15
  ax.scatter(truth[:,0],truth[:,1],s=7,c='0.8',marker='s');ax.scatter(xy[:,0],xy[:,1],s=15,c=np.where(bad,'#d73027','#208b48'));ax.scatter(xy[old,0],xy[old,1],s=38,facecolors='none',edgecolors='#7028a0');ax.set_title(arm+' risk vs reference')
  rows.append(dict(arm=arm,risk_cells=len(xy),older10s=int(old.sum()),far015=int(bad.sum()),older_far015=int((old&bad).sum())))
 axs[2].scatter(truth[:,0],truth[:,1],s=7,c='0.6',marker='s');axs[2].set_title('True body-band reference')
 for ax in axs:
  ax.plot(track[:,0],track[:,1],c='#2166ac',lw=1);ax.plot(*goal,'*',c='black',ms=12);ax.set(xlim=(-5,5.5),ylim=(-5,5.5),aspect='equal',xlabel='World X (m)',ylabel='World Y (m)')
 fig.suptitle(name+' final map at 89.9s: same-trajectory map replay, blank prediction=unknown; purple age>10s; red >.15m');fig.tight_layout();fig.savefig(D/f'{name}_map_comparison.png',dpi=140,bbox_inches='tight');plt.close(fig)
 counts=np.zeros((2,3),dtype=int);point_errors=[[],[]]
 for p in sorted((R/'partial_observations').glob('*.npz')):
  a=np.load(p);f=np.load(R/'frames'/p.name);v=np.load(D/'predictions'/f'{name}_{p.name}');q=a['query_pixel_id'];k=np.isin(q,v['query'][v['keep']]);x=q%640;y=q//640;gt=f['left_depth_m'][y,x];ok=np.isfinite(gt)&(gt>0);rays=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1);T=f['left_camera_usd_world'];gtworld=T[:3,3]+(rays*np.where(ok,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;floor=ok&(np.abs(gtworld[:,2])<=.03)
  for j,mask in enumerate([np.ones(len(q),bool),k]):
   counts[j]+=np.array([mask.sum(),(mask&ok).sum(),(mask&floor).sum()]);point_errors[j].extend(np.linalg.norm(a['world'][mask&ok]-gtworld[mask&ok],axis=1))
 for j,row in enumerate(rows):
  row.update(accepted_points=int(counts[j,0]),finite_gt_points=int(counts[j,1]),gt_floor_false=int(counts[j,2]),point_error_p50_p90=np.quantile(point_errors[j],[.5,.9]).tolist())
 summaries.append(dict(sequence=name,rows=rows,map_time_s=float(m['time_s']),point_stats_scope='all181 source captures0..90s; last capture90s is after final map89.9s; point stats not map cell membership'));print(summaries[-1],flush=True)
 if name=='E0':
  p=R/'source_predictions/frame_450.npz';a=np.load(p);f=np.load(R/'frames'/p.name);v=np.load(D/'predictions'/f'{name}_{p.name}');z=a['depth_m'];gt=f['left_depth_m'];base=np.full(z.shape,np.nan);cand=base.copy();base.ravel()[v['query']]=z.ravel()[v['query']];cand.ravel()[v['query'][v['keep']]]=z.ravel()[v['query'][v['keep']]];valid=np.isfinite(cand)&np.isfinite(gt)&(gt>0)
  fig,axs=plt.subplots(1,5,figsize=(19,4));axs[0].imshow(f['left_rgb']);axs[0].set_title('Left RGB')
  for ax,arr,title in zip(axs[1:4],[base,cand,np.where(np.isfinite(gt)&(gt>0)&(gt<=3),gt,np.nan)],['Source-packet stereo before','Source-packet stereo after','Rendered truth depth']):
   im=ax.imshow(arr,cmap='viridis',vmin=.5,vmax=3);ax.set_title(title);fig.colorbar(im,ax=ax,shrink=.5,label='m')
  im=axs[4].imshow(np.where(valid,np.abs(cand-gt),np.nan),cmap='magma',vmin=0,vmax=.6);axs[4].set_title('After absolute error');fig.colorbar(im,ax=axs[4],shrink=.5,label='m')
  for ax in axs:ax.axis('off')
  fig.suptitle('E0 at preselected 45s — sampled packet pixels only; white unknown / no score');fig.tight_layout();fig.savefig(D/'E0_depth_comparison.png',dpi=150,bbox_inches='tight');plt.close(fig)
(D/'map_results.json').write_text(json.dumps(summaries,indent=2)+'\n')
