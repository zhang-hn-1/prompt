"""Same current reference; separate estimated/true root and MODEL/raster centres."""
from pathlib import Path
import json,hashlib
import numpy as np
from remaining_reference_v1 import project_remaining
from route_geometry_audit import hazard_clearance
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';O=H/'persistent_context_D3_historical_v1'
states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());birth=np.load(D/'immutable_possible_set_birth.npz');out=[]
for f in [100,200,300,400]:
 plan=np.load(D/'local_plans'/f'frame_{f:03d}.npz')['pose'][:,:2];m=np.load(D/'maps'/f'frame_{f:03d}.npz');est=np.load(D/'estimated_poses'/f'frame_{f:03d}.npz')['root_position_world_m'][:2];raster=m['origin']+np.argwhere(m['labels']==1)*float(m['resolution']);values={}
 for name,root in [('estimated',est),('true',np.asarray(states[f]['position'])[:2])]:
  line=project_remaining(plan,root)['remaining_xy']
  for pn,pts in [('birth_MODEL_centres',birth['possible_xy']),('current_raster_centres',raster)]:values[name+'_'+pn]=hazard_clearance(np.asarray(line),pts)
 logged=rows[f]['local']['reference_decision']['guard']['suffix_clearance_m']
 assert abs(values['estimated_current_raster_centres']-logged)<1e-10
 out.append(dict(frame=f,distances_m=values,original_logged_suffix_clearance_m=logged,exact_current_raster_reconstruction=True))
p=O/'reference_distance_coordinate_audit_v1.json';p.write_text(json.dumps(dict(scope='HISTORICAL_COORDINATE_DIAGNOSIS_NOT_CONTROL_AB',rows=out,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),no_policy_recall=True),indent=2)+'\n');print(json.dumps(out,indent=2))
