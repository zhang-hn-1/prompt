from pathlib import Path
import json,hashlib,numpy as np
from tentative_point_memory import TentativePointMemory
from partial_point_risk_grid import PartialPointRiskGrid
H=Path(__file__).resolve().parent;D=H/'live_partial_point_navigation_run_v1';rows=json.loads((D/'policy_records.json').read_text());sources={r['frame']:r for r in json.loads((D/'source_rows.json').read_text())};native=json.loads((D/'native_predictions.json').read_text());memory=TentativePointMemory();g=PartialPointRiskGrid(np.asarray(native[0]['position_world'])[:2]-20,10);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();maps=0;plans=0;birth=0
for i,row in enumerate(rows):
 t=row['time_s'];n=native[i*5];assert n['available'] and n['time_s']==t
 if i%5==0:
  s=sources[i];surface=np.load(D/'partial_observations'/f'frame_{i:03d}.npz')
  for f,h in s['provenance'].items():assert sha(f)==h,f
  memory.update(dict(frame=i,time_s=t,world=surface['world'],query_pixel_id=surface['query_pixel_id'],partner_pixel_id=surface['partner_pixel_id'],input_sha256=s['provenance']),t);birth+=len(surface['world'])
  snap=memory.at(t);saved=json.loads((D/'raw_point_memory_snapshots'/f'frame_{i:03d}.json').read_text());assert snap['current_count']==saved['current_count'] and snap['historical_count']==saved['historical_count'];assert len(snap['records'])==len(saved['records'])
  for a,b in zip(snap['records'],saved['records']):
   for k in ('time_s','frame','input_sha256','layer'):assert a[k]==b[k]
   for k in ('world','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(a[k],np.asarray(b[k]).reshape(a[k].shape))
 snap=memory.at(t);sf=(i//5)*5;g.set_source(dict(frame=sf,time_s=sources[sf]['time_s'],records=snap['records']));g.update_points(np.empty((0,3)),np.asarray(n['position_world']),t);saved=np.load(D/'maps'/f'frame_{i:03d}.npz')
 for key in ('labels','codes','hazard_acquisition_time','safe_start_prior'):np.testing.assert_array_equal(getattr(g,key),saved[key])
 maps+=1;assert not (g.labels==0).any() and g.birth is None and not np.isfinite(g.memory.floor_time).any()
 if i>=140:
  assert row['planner_act_called'] and row['native_pose'] and row['partial_source_not_full_volume'];assert row['raw_point_memory_past']==snap['historical_count'];assert row['raw_point_memory_current']==snap['current_count'];assert row['end_to_end_wall_s']>=row['execute_capture_NN_wall_s']>=0
  pf=D/'local_plans'/f'frame_{i:03d}.json'
  if pf.exists():
   plan=json.loads(pf.read_text());assert plan['frame']==i and plan['time_s']==t and plan['reference_version']==row['local_reference_version'];assert len(plan['waypoints'])>=2;plans+=1
 else:assert not row['planner_act_called']
# Final post-action capture is not a consumed control map; this scope is explicit.
a=dict(decision='PASS_CURRENT_PAST_PARTIAL_MEMORY_GRID_CONSUMPTION_ONLY',control_maps_exact=maps,source_points_before_last_postaction=birth,local_plan_snapshots=plans,no_free_or_completed_volume=True,final_postaction_source_not_a_control_map=True,legacy_policy_pose_privileged_flag_inherited=True,actual_inputs='current native IMU+encoder pose; privileged initial gauge/flatheight; measured joints/RGB; idealIMU lowlevel PID',not_full_volume_or_formal35_pass=True,whole_complete=False);(H/'live_partial_point_memory_consumption_assessment_v1.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
