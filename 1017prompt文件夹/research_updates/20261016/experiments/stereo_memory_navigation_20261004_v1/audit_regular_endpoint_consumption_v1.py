"""Closed-log current-input boundary audit; not online actor replay or new nav score."""
from pathlib import Path
import json,hashlib,collections
import numpy as np
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def audit(directory):
 d=H/directory;rows=json.loads((d/'policy_records.json').read_text());predictions=json.loads((d/'native_predictions.json').read_text());out=[];inputs={str(d/n):sha(d/n) for n in ['policy_records.json','native_predictions.json','artifact_manifest_post_close.json']}
 for i,row in enumerate(rows):
  if not row.get('guard_measured_this_call'):continue
  decision=row['reference_decision'];old=decision['guard']
  if old is None:continue
  mapfile=d/'maps'/f'frame_{i:03d}.npz';pathfile=d/'local_plans'/f'frame_{i-1:03d}.json';inputs[str(mapfile)]=sha(mapfile);inputs[str(pathfile)]=sha(pathfile)
  path=json.loads(pathfile.read_text());xy=np.array([[p['x_m'],p['y_m']] for p in path['waypoints']])
  t=row['time_s'];candidates=[p for p in predictions if p['available'] and abs(p['time_s']-t)<1e-8];assert len(candidates)==1
  current=candidates[0];assert current['live_before_next_action']
  with np.load(mapfile) as m:
   assert abs(float(m['time_s'])-t)<1e-8
   value=reference_geometry_status(current['position_world'][:2],xy,m['labels'],m['origin'],float(m['resolution']))
  assert value==old,(directory,i)
  out.append(dict(frame=i,time_s=t,remaining_arc_m=value['remaining_arc_m'],endpoint_distance_m=float(np.linalg.norm(np.asarray(current['position_world'][:2])-xy[-1])),decision=decision['decision'],reject_reasons=value['reject_reasons'],reference_version=row['local_reference_version']))
 return dict(boundaries_exact=len(out),boundaries=out,near_endpoint_retained=[x for x in out if x['endpoint_distance_m']<=.2 and x['decision']=='retain'],rebuild_under_lookahead=[x for x in out if x['decision']=='rebuild' and 'remaining_arc_under_lookahead' in x['reject_reasons']],input_sha256=inputs)
if __name__=='__main__':
 out=dict(scope='offline current-input ordinary retention audit, not fixed harness nor new online validation',hypothesis='ordinary baseline retains measured completed local endpoint',arms={n:audit(n) for n in ['native_pose_baseline35_e0_run_v1','native_pose_baseline35_e1_coverage_run_v1','native_pose_baseline35_e2_coverage_run_v1']},physics_added=0)
 out['hypothesis_supported_on_closed_boundary_calls']=any(v['near_endpoint_retained'] for v in out['arms'].values())
 out['source_sha256']={str(H/n):sha(H/n) for n in ['audit_regular_endpoint_consumption_v1.py','reference_continuity_audit.py','continuous_cell_tube_consumer_v1.py','minimum_prefix_transition_consumer_v1.py','fixed_reference_pacing_harness_v1.py']}
 p=H/'regular_endpoint_consumption_assessment_v1.json';p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:dict(boundaries_exact=v['boundaries_exact'],near_endpoint_retained=v['near_endpoint_retained'],rebuild_under_lookahead=v['rebuild_under_lookahead']) for k,v in out['arms'].items()},indent=2))
