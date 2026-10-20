"""Consumer-equivalence against sealed actual old-producer whole-sequence grids."""
from pathlib import Path
import json,hashlib,datetime,time,copy
import numpy as np
from point_support_shadow import PointSupportMap,swept_indices
from dense_source_points import records
from grid_only_snapshot import grid_arrays_snapshot
from npz_packet_scalars import restore_packet_scalars
H=Path(__file__).resolve().parent;O=H/'masked_memory_history_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();inputs=[O/'policy_records.json',O/'states.json']+list((O/'sensor_packets').glob('*.npz'))+list((O/'maps').glob('*.npz'))+[H/'grid_only_snapshot.py',H/'test_grid_only_snapshot.py',H/'audit_grid_only_snapshot_v2.py',H/'audit_grid_only_snapshot.py',H/'npz_packet_scalars.py',H/'test_npz_packet_scalars.py',H/'point_support_shadow.py',H/'dense_source_points.py',H.parent.parent/'docs/superpowers/specs/2026-10-07-grid-only-snapshot-design.md'];source={str(p):sha(p) for p in inputs};(H/'grid_only_snapshot_protocol_v2.json').write_text(json.dumps(dict(utc=now(),source_sha256=source,whole_historical_sequence=True,minimum_hot_snapshot_median_speedup=2.,online_not_changed=True),indent=2)+'\n')
rr=json.loads((O/'policy_records.json').read_text());states=json.loads((O/'states.json').read_text());packets={int(np.load(p)['frame']):{k:np.load(p)[k] for k in np.load(p).files} for p in (O/'sensor_packets').glob('*.npz')};packets={k:restore_packet_scalars(p) for k,p in packets.items()};mapper=None;exact=0;diagchecks=0;timings=[];maxrecords=0;first=None
for i,row in enumerate(rr):
 saved=np.load(O/'maps'/f'frame_{i:03d}.npz');t=float(saved['time_s'])
 if mapper is None:mapper=PointSupportMap(saved['origin'],saved['labels'].shape,float(saved['resolution']));first=swept_indices([np.asarray(states[i]['position'])[:2]],mapper.origin,mapper.resolution,.7);startprior=np.zeros(mapper.shape,bool);ok=(first>=0).all(1)&(first[:,0]<mapper.shape[0])&(first[:,1]<mapper.shape[1]);c=first[ok];startprior[c[:,0],c[:,1]]=True
 sourceframe=row['local']['source_frame'];mapper.update(records(packets[sourceframe]),t,0.);snap=grid_arrays_snapshot(mapper,t);maxrecords=max(maxrecords,len(mapper.records))
 for k,v in dict(labels=snap['labels'],provenance=snap['provenance'],floor_time=snap['floor_time'],hazard_time=snap['hazard_time'],task_ground_prior=np.ones(mapper.shape,bool),safe_start_prior=startprior).items():np.testing.assert_equal(v,saved[k]);exact+=1
 codes=snap['provenance'];cells=swept_indices([np.asarray(states[i]['position'])[:2]],mapper.origin,mapper.resolution,.7);inside=(cells>=0).all(1)&(cells[:,0]<mapper.shape[0])&(cells[:,1]<mapper.shape[1]);cells=cells[inside]
 diagnostics=dict(memory_mode='history',current_floor=int((codes==1).sum()),historical_floor=int((codes==2).sum()),current_occupied=int((codes==3).sum()),historical_occupied=int((codes==4).sum()),unknown=int((codes==0).sum()),footprint_unknown=int((snap['labels'][cells[:,0],cells[:,1]]<0).sum()),body_prior_observed_free=0,pose_sigma_assumption_m=0.,full_3d_certified=False,task_ground_prior_is_observation=False,safe_start_prior_is_observation=False)
 for k,v in diagnostics.items():assert row['local'][k]==v,(i,k);diagchecks+=1
 if i in (0,60,145,225):
  before=copy.deepcopy(mapper.records);t0=time.perf_counter();old=mapper.snapshot(t);old_s=time.perf_counter()-t0;t0=time.perf_counter();new=grid_arrays_snapshot(mapper,t);new_s=time.perf_counter()-t0;assert mapper.records==before
  for k,v in new.items():np.testing.assert_equal(v,old[k])
  timings.append(dict(frame=i,records=len(mapper.records),old_full_snapshot_s=old_s,new_grid_only_s=new_s,speedup=old_s/new_s,old_materializes_unused_record_copies=True,new_raw_records_omitted_only_from_return=True))
med=float(np.median([x['speedup'] for x in timings]));result=dict(utc=now(),accepted=med>=2,all_grids_exact=True,whole_sequence_frames=len(rr),exact_saved_arrays=exact,exact_diagnostics=diagchecks,maximum_records=maxrecords,hot_state_timings=timings,median_speedup=med,raw_record_state_unchanged=True,source_identity_checks=len(source),online_not_changed=True,no_navigation_improvement_claim=True,whole_research_complete=False)
(H/'grid_only_snapshot_assessment_v2.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
