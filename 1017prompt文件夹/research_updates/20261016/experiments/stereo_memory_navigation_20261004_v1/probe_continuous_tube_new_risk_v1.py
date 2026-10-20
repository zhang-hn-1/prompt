from pathlib import Path
import sys,json,numpy as np,hashlib
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from continuous_cell_tube_v1 import continuous_cell_tube_screen
rows=json.loads((D/'policy_records.json').read_text());m=np.load(D/'maps/frame_506.npz');old=np.load(D/'maps/frame_501.npz');g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'].copy();guard=rows[506]['reference_decision']['guard'];current=continuous_cell_tube_screen(g,guard);assert current==rows[506]['reference_decision']['tube_viability_screen'];g.labels=old['labels'].copy();prior=continuous_cell_tube_screen(g,guard);added=np.argwhere((m['labels']==1)&(old['labels']!=1));removed=np.argwhere((m['labels']!=1)&(old['labels']==1));trace=[]
for c in added:
 support=[]
 for i in range(0,506,5):
  path=D/'partial_observations'/f'frame_{i:03d}.npz';p=np.load(path);g.labels=m['labels'];cells=g.cells(p['world'][:,:2]);mask=(cells==c).all(axis=1)&(p['world'][:,2]>.05)&(p['world'][:,2]<=1.5)
  for j in np.flatnonzero(mask):support.append(dict(frame=i,time_s=float(p['acquisition_time_s']),world=p['world'][j].tolist(),query_pixel_id=int(p['query_pixel_id'][j]),partner_pixel_id=int(p['partner_pixel_id'][j])))
 trace.append(dict(cell=c.tolist(),world_center=(m['origin']+c*float(m['resolution'])).tolist(),code=int(m['codes'][tuple(c)]),hazard_acquisition_time_s=float(m['hazard_acquisition_time'][tuple(c)]),all_accepted_support_through_frame505=support))
out=dict(scope='ONE POSTCLOSE DEVELOPMENT frame506 fixed geometry risk-label map-swap; raw accepted supports through505 only. No teacher/actor recall/next control/model, no safety admission. Historical map swap alone cannot isolate upstream correspondence or certify safety.',current_screen=current,prior501_labels_same506_geometry_screen=prior,added=trace,removed_cells=removed.tolist(),old_gate_unchanged=True,formal35_NOTRUN=True);(D/'new_risk_trigger_diagnostic_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
