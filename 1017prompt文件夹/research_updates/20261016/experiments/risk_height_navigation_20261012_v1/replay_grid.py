from pathlib import Path
import sys,json,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path[:0]=[str(H),str(P)]
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from execution_domain_four_point_consumer_v2 import planning_clearance
from scipy.ndimage import distance_transform_edt
R=W/'experiments/nearfield_rig_navigation_20261012_v1/candidate_e2_run';native=json.loads((R/'native_predictions.json').read_text());poses={round(n['time_s'],7):np.asarray(n['position_world']) for n in native if n.get('available')};maps=np.load(R/'maps/frame_000.npz');ep=json.loads((R/'task_episode.json').read_text())['actual_episode'];goal=np.array(ep['goal_position_world_m'][:2]);old=RepeatedExposureMemoryGrid(maps['origin'],10,static_scene_assumption=True);new=RepeatedExposureMemoryGrid(maps['origin'],10,static_scene_assumption=True);keep_ids={};rows=[]
for p in sorted((R/'partial_observations').glob('*.npz')):
 a=np.load(p);keep=a['world'][:,2]-a['sigma_m']>.05;keep_ids[int(a['frame'])]=set(map(int,a['query_pixel_id'][keep]))
for i in range(0,900,5):
 sm=np.load(R/'maps'/f'frame_{i:03d}.npz');t=float(sm['time_s']);root=poses[round(t,7)];snap=json.loads((R/'raw_point_memory_snapshots'/f'frame_{i:03d}.json').read_text());records=[];filtered=[]
 for r in snap['records']:
  a=dict(r);a['world']=np.asarray(r['world']).reshape(-1,3);a['query_pixel_id']=np.asarray(r['query_pixel_id']);a['partner_pixel_id']=np.asarray(r['partner_pixel_id']);records.append(a)
  keep=np.array([int(q) in keep_ids[int(r['frame'])] for q in r['query_pixel_id']],bool);b=dict(a)
  for k in ['world','query_pixel_id','partner_pixel_id']:b[k]=a[k][keep]
  filtered.append(b)
 for grid,rs in [(old,records),(new,filtered)]:grid.consume_records(rs,t,root)
 assert np.array_equal(old.labels,sm['labels']),(i,'baseline replay mismatch')
 if i%50==0 or i==895:
  result=dict(frame=i,time_s=t)
  for label,g in [('old',old),('filtered',new)]:
   clearance=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;c=g.cells(root[:2][None])[0];end=g.cells(goal[None])[0]
   result[label]=dict(hazard_cells=int((g.labels==1).sum()),root_clearance_m=float(clearance[tuple(c)]),goal_clearance_m=float(clearance[tuple(end)]),root_pass=bool(clearance[tuple(c)]>=planning_clearance(g)-1e-8),goal_pass=bool(clearance[tuple(end)]>=planning_clearance(g)-1e-8))
  rows.append(result)
out=dict(baseline_replay_exact_frames=180,scope='Same saved trajectory map replay; interval-ambiguous points remain unknown, no observed-free claim or navigation counterfactual claim',rows=rows)
(D/'grid_replay.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
