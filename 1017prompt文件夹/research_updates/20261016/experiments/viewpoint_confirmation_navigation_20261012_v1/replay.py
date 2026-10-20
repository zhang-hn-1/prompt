from pathlib import Path
import sys,json,numpy as np
from scipy.ndimage import distance_transform_edt
D=Path(__file__).resolve().parent;W=D.parents[1];sys.path[:0]=[str(D),str(W/'experiments/stereo_memory_navigation_20261004_v1'),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from viewpoint_confirmation import ViewpointConfirmationGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from execution_domain_four_point_consumer_v2 import planning_clearance
R=W/'experiments/nearfield_rig_navigation_20261012_v1/candidate_e2_run';sm=np.load(R/'maps/frame_000.npz');old=RepeatedExposureMemoryGrid(sm['origin'],10,static_scene_assumption=True);new=ViewpointConfirmationGrid(sm['origin'],10,static_scene_assumption=True);views={int(p.stem.split('_')[1]):dict(time_s=float(np.load(p)['time_s']),position_world=np.load(p)['left_camera_usd_world'][:3,3]) for p in (R/'known_poses').glob('*.npz')};native=json.loads((R/'native_predictions.json').read_text());poses={round(x['time_s'],7):x['position_world'] for x in native if x.get('available')};rows=[];changed=0
for i in range(0,900,5):
 m=np.load(R/'maps'/f'frame_{i:03d}.npz');t=float(m['time_s']);root=np.array(poses[round(t,7)]);rs=json.loads((R/'raw_point_memory_snapshots'/f'frame_{i:03d}.json').read_text())['records']
 old.consume_records(rs,t,root);new.consume_records(rs,t,root,views)
 assert np.array_equal(old.labels,m['labels']);changed+=int(not np.array_equal(new.labels,old.labels))
 if i%50==0 or i==895:
  row=dict(frame=i,time_s=t)
  for k,g in [('old',old),('new',new)]:
   c=g.cells(root[:2][None])[0];cl=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution
   row[k]=dict(hazards=int((g.labels==1).sum()),confirmed=int(np.isfinite(g._confirmed_clock).sum()),root_clearance_m=float(cl[tuple(c)]),root_pass=bool(cl[tuple(c)]>=planning_clearance(g)-1e-8))
  rows.append(row)
x=dict(baseline_map_replay_exact_frames=180,changed_map_frames=changed,rows=rows,scope='Saved trajectory counterfactual memory only; unchanged recent10s. Camera translation screen necessary heuristic, not correctness or independence certificate. No truth consulted, no navigation benefit claim.')
(D/'replay_assessment.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps(x))
