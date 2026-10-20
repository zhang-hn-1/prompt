"""Postsealed labels for two fixed cells; not a producer or filter."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;O=H/'E1_guard_cell_trace_v1';seal=json.loads((O/'seal_pretruth.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();assert sha(O/'trace_pretruth.json')==seal['output_sha256'];assert all(sha(p)==s for p,s in seal['inputs'].items());v=json.loads((O/'trace_pretruth.json').read_text());rows=[];directory={'old':'minimum_prefix_transition_navigation_run_v1','new_raw':'E1_current_pixel_navigation_run_v1','new_filtered':'E1_current_pixel_navigation_run_v1'}
for name,cells in v['datasets'].items():
 D=H/directory[name]
 for cell in cells:
  rr=[]
  for r in cell['records']:
   f=r['frame'];p=D/'frames'/f'frame_{f:03d}.npz';g=np.load(p);T=np.load(D/'known_poses'/p.name)['left_camera_usd_world'];q=r['query_pixel_id'];x,y=q%640,q//640;z=float(g['left_depth_m'][y,x]);valid=bool(np.isfinite(z) and z>0);truth=None;err=None;truthcell=None
   if valid:
    truth=T[:3,3]+(np.array([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,1])*z)@(T[:3,:3]@np.diag([1,-1,-1])).T;err=float(np.linalg.norm(truth-np.asarray(r['world'])));m=np.load(D/'maps/frame_331.npz');truthcell=np.rint((truth[:2]-m['origin'])/float(m['resolution'])).astype(int).tolist()
   rr.append(dict(**r,GT_valid=valid,GT_world=None if truth is None else truth.tolist(),error_m=err,within10cm=valid and err<=.1,GT_same_risk_cell=truthcell==cell['cell'],GT_cell=truthcell))
  rows.append({'dataset':name,'target':cell['target_xy'],'points':rr,'n':len(rr),'valid':sum(r['GT_valid'] for r in rr),'correct10cm':sum(r['within10cm'] for r in rr),'same_cell_truth':sum(r['GT_same_risk_cell'] for r in rr)})
result={'scope':'POSTSEAL_TWO_GUARD_CELL_LABELS_NOT_SOURCE_CERTIFICATION','rows':rows,'no_source_selection_change':True,'no_gate_change':True,'history_not_fresh':True};(O/'postseal_point_labels_v1.json').write_text(json.dumps(result,indent=2)+'\n');print([{k:v for k,v in r.items() if k!='points'} for r in rows]);print('old critical point',rows[0]['points'])
