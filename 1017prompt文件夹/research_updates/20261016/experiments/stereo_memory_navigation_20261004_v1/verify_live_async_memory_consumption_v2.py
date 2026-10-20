"""Availability-gated rebuild, never reconstitute source before its delivery."""
from pathlib import Path
import json,hashlib,numpy as np
from causal_source_delivery_memory import CausalSourceDeliveryMemory
from partial_point_risk_grid import PartialPointRiskGrid
H=Path(__file__).resolve().parent;D=H/'live_async_stereo_navigation_run_v1';rows=json.loads((D/'policy_records.json').read_text());sources=json.loads((D/'source_rows.json').read_text());native=json.loads((D/'native_predictions.json').read_text());m=CausalSourceDeliveryMemory();g=PartialPointRiskGrid(np.asarray(native[0]['position_world'])[:2]-20,10);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();cursor=0;maps=0;plans=0
for i,row in enumerate(rows):
 t=row['time_s'];n=native[i*5];assert n['available'] and n['time_s']==t
 while cursor<len(sources) and sources[cursor]['available_frame']<=i:
  s=sources[cursor];assert s['available_time_s']<=t and s['time_s']<=s['available_time_s'];birth=s['frame']
  z=np.load(D/'partial_observations'/f'frame_{birth:03d}.npz')
  for f,v in s['provenance'].items():assert sha(f)==v
  m.deliver(dict(frame=birth,time_s=s['time_s'],world=z['world'],query_pixel_id=z['query_pixel_id'],partner_pixel_id=z['partner_pixel_id'],input_sha256=s['provenance']),available_time_s=s['available_time_s'],now=s['available_time_s'])
  expected_times=([r['time_s'] for r in m.snapshot(s['available_time_s'])['records']] if s['async_source'] else [r['time_s'] for r in sources[:cursor+1]])
  # Prefix field is the unexpired original source-generation registry, not
  # the separate actor's 10s TTL map; async field explicitly uses actor TTL.
  assert s['memory_acquisition_times_s']==expected_times;cursor+=1
 snap=m.snapshot(t);g.set_source(dict(frame=snap['source_frame'],time_s=snap['source_time_s'],records=snap['records']));g.update_points(np.empty((0,3)),np.asarray(n['position_world']),t);saved=np.load(D/'maps'/f'frame_{i:03d}.npz')
 for key in ('labels','codes','hazard_acquisition_time','safe_start_prior'):np.testing.assert_array_equal(getattr(g,key),saved[key])
 assert not (g.labels==0).any() and g.birth is None and not np.isfinite(g.memory.floor_time).any();maps+=1
 if i>=140:
  assert row['planner_act_called'] and row['raw_point_memory_past']==snap['historical_count'] and row['raw_point_memory_current']==snap['current_count']
  f=D/'local_plans'/f'frame_{i:03d}.json'
  if f.exists():
   p=json.loads(f.read_text());assert p['time_s']==t and p['reference_version']==row['local_reference_version'];plans+=1
 else:assert not row['planner_act_called']
r=dict(decision='PASS_AVAILABILITY_GATED_GRID_CONSUMPTION_ONLY',prefix_metadata_source_registry_not_actor_TTL=True,actor_TTL10_unchanged_and_maps_rebuilt=True,control_maps_exact=maps,delivered_records_consumed=cursor,local_plan_snapshots=plans,no_future_delivery_or_birth_refresh=True,no_free_or_completed_volume=True,not_navigation_or_realtime_pass=True,whole_complete=False)
(H/'live_async_memory_consumption_assessment_v2.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
