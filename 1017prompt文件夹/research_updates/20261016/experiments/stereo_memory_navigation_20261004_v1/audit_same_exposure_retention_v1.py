"""Bounded same-current-arm exposure retention diagnostic; never new online action."""
from pathlib import Path
import json,hashlib,numpy as np
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent
D=H/'sgbm_risk_lifetime_e1_current_run_v1'
if __name__=='__main__':
 seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
 for f,d in seal['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==d
 rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());ep=json.loads((D/'task_episode.json').read_text())['actual_episode'];sources=json.loads((D/'source_rows.json').read_text())
 grid=RepeatedExposureMemoryGrid(np.array(ep['start_position_world_m'][:2])-20,10,static_scene_assumption=True);out=[]
 for r in rows:
  f=r['frame'];t=r['time_s'];snap=json.loads((D/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text());grid.set_source(dict(frame=r['source_frame'],time_s=r['source_time_s'],records=snap['records']));root=states[f]['position'];grid.update_points([],np.array(root),t)
  if not r.get('guard_measured_this_call') or r['reference_decision']['guard'] is None:continue
  oldpath=rows[f-1]['installed_reference_xy_after_act'];xy=np.array(root[:2])
  with np.load(D/'maps'/f'frame_{f:03d}.npz') as m:
   original=reference_geometry_status(xy,oldpath,m['labels'],m['origin'],float(m['resolution']))
   assert original==r['reference_decision']['guard'],f
  history=reference_geometry_status(xy,oldpath,grid.labels,grid.origin,grid.resolution)
  source=next(x for x in sources if x['frame']==r['source_frame'])
  out.append(dict(frame=f,time_s=t,current_source_points=source['points'],actual_retention_screen_pass=original['retention_necessary_screen_pass'],same_exposure_history_screen_pass=history['retention_necessary_screen_pass'],history_reject_reasons=history['reject_reasons'],actual_hazard_cells=original['observed_hazard_count'],history_hazard_cells=history['observed_hazard_count'],actual_suffix_clearance_m=original['suffix_clearance_m'],history_suffix_clearance_m=history['suffix_clearance_m'],future_input_used=False,source_frame=r['source_frame'],source_time_s=r['source_time_s']))
 result=dict(scope='Offline same actual-current-arm source records/current pose/current saved old reference; pure retention predicate only. Does not simulate a changed planner/controller or prove a collision counterfactual.',whole_mission_complete=False,new_physics=0,new_act_calls=0,boundaries_exact=len(out),rows=out,actual_retain_but_history_reject=[r for r in out if r['actual_retention_screen_pass'] and not r['same_exposure_history_screen_pass']],input_seal_sha256=hashlib.sha256((D/'artifact_manifest_post_close.json').read_bytes()).hexdigest(),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
 p=H/'same_exposure_retention_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
