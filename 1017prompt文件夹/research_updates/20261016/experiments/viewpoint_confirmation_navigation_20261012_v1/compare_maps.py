"""Original promotion on actual candidate input; only map effects counterfactual."""
from pathlib import Path
import sys,json,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];sys.path[:0]=[str(W/'experiments/stereo_memory_navigation_20261004_v1'),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
R=D/'candidate_e2_run';maps=sorted((R/'maps').glob('*.npz'));m=np.load(maps[0]);g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);native=json.loads((R/'native_predictions.json').read_text());poses={round(x['time_s'],7):x['position_world'] for x in native if x.get('available')};changed=0;first=None;peak=0
for p in maps:
 a=np.load(p);t=float(a['time_s']);rs=json.loads((R/'raw_point_memory_snapshots'/(p.stem+'.json')).read_text())['records'];g.consume_records(rs,t,np.asarray(poses[round(t,7)]));difference=int((a['labels']!=g.labels).sum());peak=max(peak,difference)
 if difference:
  changed+=1
  if first is None:first=dict(frame=p.stem,time_s=t,changed_cells=difference)
control=W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run';q=np.load(R/'requests.npz')['requests'];b=np.load(control/'requests.npz')['requests'];n=min(len(q),len(b));dif=np.flatnonzero(np.any(q[:n]!=b[:n],axis=1))
x=dict(same_input_map_changed_frames=changed,total_map_frames=len(maps),first_map_difference=first,peak_changed_cells=peak,final_original_persistent_cells=int(np.isfinite(g._confirmed_clock).sum()),final_candidate_hazard_cells=int((a['labels']==1).sum()),historical_request_differences=int(len(dif)),first_historical_request_difference=int(dif[0]) if len(dif) else None,scope='Same actual current/past points and native roots for map-only oldpromotion shadow. Historical request comparison may have different RGB, no alternate-physics/fullcausal/generalization claim.')
(D/'map_intervention.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps(x))
