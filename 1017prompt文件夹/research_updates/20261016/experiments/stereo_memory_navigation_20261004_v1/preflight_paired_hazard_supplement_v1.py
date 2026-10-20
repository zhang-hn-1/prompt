"""ONE map union necessary preflight. Historical B0 states are not C1 outcomes."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.ndimage import distance_transform_edt
from paired_current_evidence_v1 import qualify_packet
from paired_prior_memory_v1 import PairedPriorMemory
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';O=H/'paired_hazard_supplement_preflight_historical_v1';O.mkdir();m=PairedPriorMemory();rows=[]
for f in range(0,401,5):
 p=dict(np.load(D/'sensor_packets'/f'source_{f:03d}.npz'));rgb=np.load(D/'online_inputs'/f'frame_{f:03d}.npz');mask=np.load(D/'masks'/f'frame_{f:03d}.npz');t=float(p['time_s']);q=qualify_packet(p,rgb['left_rgb'],rgb['right_rgb'],mask['left'],mask['right'],t,t,f);a=q['eligible_packet'];b=q['unknown_packet'];hashes={str(D/'sensor_packets'/f'source_{f:03d}.npz'):hashlib.sha256((D/'sensor_packets'/f'source_{f:03d}.npz').read_bytes()).hexdigest()};a['input_sha256']=b['input_sha256']=hashes;m.update(a,b,t)
 if f not in [20,60,100,200,300,400]:continue
 z=np.load(D/'maps'/f'frame_{f:03d}.npz');snap=m.at(t);pts=np.concatenate([r['world'] for r in snap['provisional']['records']],axis=0);pts=pts[(pts[:,2]>.05)&(pts[:,2]<=1.5)];cells=np.rint((pts[:,:2]-z['origin'])/float(z['resolution'])).astype(int);cells=cells[((cells>=0)&(cells<np.asarray(z['labels'].shape))).all(1)];union=z['labels'].copy();union[tuple(cells.T)]=1;field=distance_transform_edt(np.pad(union!=1,1))[1:-1,1:-1]*float(z['resolution']);pose=np.load(D/'estimated_poses'/f'frame_{f:03d}.npz');start=np.rint((pose['root_position_world_m'][:2]-z['origin'])/float(z['resolution'])).astype(int);goal=np.rint((np.array([2.2,0.])-z['origin'])/float(z['resolution'])).astype(int);s=float(field[tuple(start)]);g=float(field[tuple(goal)]);rows.append(dict(frame=f,base_MODEL_cells=int((z['labels']==1).sum()),added_cells=int(((union==1)&(z['labels']!=1)).sum()),current_start_raster_clearance_m=s,fixed_goal_raster_clearance_m=g,start_goal_necessary_pass=min(s,g)>=.8707106781186548-1e-8,role='B0 actual state only; no C1 future outcome'))
 np.savez_compressed(O/f'map_{f:03d}.npz',labels=union,original_MODEL_labels=z['labels'],time_s=t,frame=f,observed_free=False,source_admitted=False)
first=next(r for r in rows if r['frame']==60);r=dict(scope='HISTORICAL_SUPPLEMENT_FIRST_HANDOFF_NECESSARY_PREFLIGHT',rows=rows,first_handoff_necessary_pass=first['start_goal_necessary_pass'],decision='NECESSARY_ONLY_NOT_ADMITTED' if first['start_goal_necessary_pass'] else 'REJECT_UNIQUE_SUPPLEMENT_BEFORE_PHYSICS',no_actor_or_future_truth=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
