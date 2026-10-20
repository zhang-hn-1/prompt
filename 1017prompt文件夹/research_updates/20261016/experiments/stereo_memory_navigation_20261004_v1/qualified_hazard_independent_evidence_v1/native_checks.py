from pathlib import Path
import sys,json,hashlib
import numpy as np
H=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1');E=H/'qualified_hazard_independent_evidence_v1';D=H/'qualified_hazard_navigation_run_v1';sys.path.insert(0,str(H))
from qualified_hazard_consumer import QualifiedHazardGrid
J=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
proof=J(H/'qualified_hazard_runner_derivation_v1.json');parent=Path(proof['parent']);child=Path(proof['child']);text=child.read_text()
for old,new in reversed(proof['changes']):
 assert text.count(new)==1,(old,text.count(new));text=text.replace(new,old)
assert text.encode()==parent.read_bytes();assert sha(child)==proof['child_sha256'] and sha(parent)==proof['parent_sha256']
protocol=J(H/'qualified_hazard_navigation_protocol_v1.json');assert all(sha(Path(p))==s for p,s in protocol['source_sha256'].items())
rows=J(D/'policy_records.json');states=J(D/'states.json');g=None;arrays=0;ages=0;historical=0;unknown_floor=0;rejected_cells=0
load=lambda p:{k:(a[k].item() if a[k].ndim==0 else a[k]) for a in [np.load(p)] for k in a.files}
for i,r in enumerate(rows):
 m=load(D/'maps'/f'frame_{i:03d}.npz');sf=r['local']['source_frame'];cur=load(D/'qualified_sensor_packets'/f'source_{sf:03d}.npz');past=load(D/'appearance_old_packets'/f'source_{sf:03d}.npz');t=m['time_s']
 if g is None:g=QualifiedHazardGrid(m['origin'],m['labels'].shape,m['resolution'],'current')
 g.set_source(cur);g.set_memory_source(past);g.update_points([],states[i]['position'],t)
 for key,val in dict(labels=g.labels,provenance=g.provenance(),floor_time=g.memory.floor_time,hazard_time=g.memory.hazard_time,task_ground_prior=g.task_ground_prior,safe_start_prior=g.safe_start_prior).items():np.testing.assert_equal(val,m[key]);arrays+=1
 for p in [cur,past]:assert p['time_s']<=t and (not len(p['world']) or t-p['time_s']<=10);ages+=len(p['world'])
 historical+=int((g.provenance()==4).sum())
 if i==sf:
  raw=load(D/'sensor_packets'/f'source_{sf:03d}.npz');h=(raw['world'][:,2]>.05)&(raw['world'][:,2]<=1.5);rejected=h&~np.isin(raw['query_pixel_id'],cur['query_pixel_id']);cells=np.unique(np.rint((raw['world'][rejected,:2]-g.origin)/g.resolution).astype(int),axis=0);cells=cells[g.inside(cells)];vals=g.labels[cells[:,0],cells[:,1]];unknown_floor+=int((vals==0).sum());rejected_cells+=len(cells)
B=H/'observation_handoff_observe_run_recovery_v2';b=np.load(B/'physics.npz');c=np.load(D/'physics.npz');fields=set(c.files)-{'raw_simulation_time_s','raw_physics_step_index'};assert len(fields)==17
assert all(np.array_equal(b[k][:901],c[k][:901]) for k in fields)
first_rgb=next(i for i in range(0,181,5) if not np.array_equal(np.load(B/'online_inputs'/f'frame_{i:03d}.npz')['left_rgb'],np.load(D/'online_inputs'/f'frame_{i:03d}.npz')['left_rgb']))
first_source=next(i for i in range(0,181,5) if not all(np.array_equal(np.load(B/'sensor_packets'/f'source_{i:03d}.npz')[k],np.load(D/'sensor_packets'/f'source_{i:03d}.npz')[k]) for k in np.load(D/'sensor_packets'/f'source_{i:03d}.npz').files))
result=dict(native_map_exact_arrays=arrays,source_point_age_checks=ages,historical_occupied_cell_instances=historical,rejected_hazard_cell_instances=rejected_cells,rejected_hazard_floor_overlap=unknown_floor,reverse_parent_bytes_exact=True,protocol_source_hashes_exact=len(protocol['source_sha256']),physical_prefix_through18_exact_samples=901,physical_prefix_exact_fields=17,first_left_RGB_difference_frame=first_rgb,first_original_source_field_difference_frame=first_source,author_collection_CAD_and_full_video_decode_not_personally_reexecuted=True)
(E/'native_checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
