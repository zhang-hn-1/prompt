from pathlib import Path
import json,hashlib,numpy as np
from tentative_point_memory import TentativePointMemory
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
H=Path(__file__).resolve().parent;D=H/'front_view_handoff_navigation_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'artifact_manifest_post_close.json').read_text());assert json.loads((H/'front_view_handoff_navigation_status_v1.json').read_text())['status']=='finished'
for p,d in seal['artifact_sha256'].items():assert sha(p)==d,p
rows=json.loads((D/'policy_records.json').read_text());sources=json.loads((D/'source_rows.json').read_text());states=json.loads((D/'states.json').read_text());m=TentativePointMemory();g=RepeatedExposureMemoryGrid(np.array([-3.2999999940395353,5.960465010446114e-9])-20,10,static_scene_assumption=True);cursor=0;plans=0;older_epochs=0;current_epochs=0
for i,r in enumerate(rows):
 t=r['time_s'];assert t==states[i]['time_s'] and r['known_GT_pose'] and not r['native_pose_used']
 while cursor<len(sources) and sources[cursor]['frame']<=i:
  s=sources[cursor];assert s['time_s']<=t
  p=np.load(D/'partial_observations'/f"frame_{s['frame']:03d}.npz")
  for f,d in s['provenance'].items():assert sha(f)==d,f
  m.update(dict(frame=s['frame'],time_s=s['time_s'],world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=s['provenance']),s['time_s']);cursor+=1
 snap=m.at(t);src=sources[cursor-1];g.set_source(dict(frame=src['frame'],time_s=src['time_s'],records=snap['records']));g.update_points(np.empty((0,3)),np.asarray(states[i]['position']),t);saved=np.load(D/'maps'/f'frame_{i:03d}.npz')
 for key in ['labels','codes','hazard_acquisition_time','safe_start_prior','origin','resolution']:np.testing.assert_array_equal(getattr(g,key),saved[key])
 assert g.diagnostics()['persistent_confirmation_required_epochs']==2
 for rec in snap['records']:
  if t-rec['time_s']<=10.:
   cells=g.cells(rec['world'][:,:2]);cells=cells[g.inside(cells)];assert np.all(g.labels[tuple(cells.T)]==1)
 assert r['confirmed_persistent_hazard_cells']==g.diagnostics()['confirmed_persistent_hazard_cells']
 assert not np.any(g.labels==0) and not np.isfinite(g.memory.floor_time).any()
 assert r['raw_point_memory_current']==snap['current_count'] and r['raw_point_memory_past']==snap['historical_count']
 current_epochs+=snap['current_count'];older_epochs+=sum(len(rec['world']) for rec in snap['records'] if rec['time_s']<src['time_s'])
 serial=json.loads((D/'raw_point_memory_snapshots'/f'frame_{i:03d}.json').read_text());assert serial['current_count']==snap['current_count'] and serial['historical_count']==snap['historical_count'] and len(serial['records'])==len(snap['records'])
 for a,b in zip(serial['records'],snap['records']):
  for key in ['frame','time_s','input_sha256']:assert a[key]==b[key]
  for key in ['world','query_pixel_id','partner_pixel_id']:np.testing.assert_array_equal(np.asarray(a[key]).reshape(np.asarray(b[key]).shape),b[key])
 f=D/'local_plans'/f'frame_{i:03d}.json'
 if f.exists():
  plan=json.loads(f.read_text());assert plan['time_s']==t and plan['reference_version']==r['local_reference_version'];plans+=1
 assert all(np.isfinite(r[k]) and r[k]>=0 for k in ['policy_wall_s','execute_capture_NN_wall_s','end_to_end_wall_s'])
q=np.asarray([r['end_to_end_wall_s'] for r in rows]);result=dict(decision='PASS_ACTUAL_REPEATED_EXPOSURE_PROMOTION_MEMORY_MAP_ONLY',conditional_gated120_not_original35=True,static_scene_assumption=True,maps_exact=len(rows),source_records_consumed=cursor,unconsumed_terminal_exposure=len(sources)-cursor,explicit_memory_snapshots_exact=len(rows),local_plan_snapshots=plans,current_capture_point_record_epochs=current_epochs,genuine_older_exposure_point_record_epochs=older_epochs,raw_points_birth_TTL10_no_refresh=True,past_measured_hazard_cells_persist_static_assumption=False,only_two_distinct_epoch_corroborated_risk_persists=True,recent_TTL10_risk_kept=True,expired_unconfirmed_unknown_not_free=True,no_free_or_completed_volume=True,action_wall_p50_s=float(np.median(q)),action_wall_p90_s=float(np.percentile(q,90)),action_wall_peak_s=float(q.max()),within100ms_actions=int((q<=.1).sum()),not_realtime_certification=True,not_navigation_gate=True,known_GT_pose=True,whole_research_complete=False)
(H/'front_view_handoff_memory_consumption_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
