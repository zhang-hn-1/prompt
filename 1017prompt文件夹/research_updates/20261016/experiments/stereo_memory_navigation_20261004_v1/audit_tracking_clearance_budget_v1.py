"""Actual pre-failure normal tracking; no counterfactual/model/control change."""
from pathlib import Path
import json,datetime,hashlib,math
import numpy as np
H=Path(__file__).resolve().parent;results={}
wrap=lambda x:np.arctan2(np.sin(x),np.cos(x))
for arm,D in [('C1',H/'typed_hypothesis_memory_C1_run_v1'),('C2',H/'bridge_consumer_C2_run_v1')]:
 rows=json.loads((D/'policy_records.json').read_text());seal=json.loads((D/'artifact_manifest_post_close.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();assert all(sha(f)==s for f,s in seal['artifact_sha256'].items())
 failure=next(i for i,r in enumerate(rows) if r['local']['plan'].get('status') in ['unreachable','observed_obstacle_blocks_route']);valid=[]
 for i,r in enumerate(rows[:failure]):
  tr=r['tracking']
  if not tr or not tr.get('valid') or tr.get('source')!='tracking' or tr.get('active_drive_direction')!='forward' or tr.get('world_xy_speed_m_s',0)<.05:continue
  assert tr['velocity_course_valid'];mp=r['mapping'];valid.append(dict(frame=i,time_s=r['local']['time_s'],projection_error_m=tr['projection_distance_m'],root_course_error_rad=float(wrap(tr['root_heading_rad']-tr['velocity_course_rad'])),course_reference_error_rad=float(wrap(tr['velocity_course_rad']-tr['reference_motion_tangent_rad'])),speed=tr['world_xy_speed_m_s'],bank_target_rad=mp['bank_target_after_outer_correction_rad'],bank_actual_rad=math.asin(np.clip(2*(np.asarray(json.loads((D/'states.json').read_text())[i]['quaternion'])[0]*np.asarray(json.loads((D/'states.json').read_text())[i]['quaternion'])[1]+np.asarray(json.loads((D/'states.json').read_text())[i]['quaternion'])[2]*np.asarray(json.loads((D/'states.json').read_text())[i]['quaternion'])[3]),-1,1)),speed_clamped=mp['speed_clamped']))
 assert valid
 def stat(k):
  v=np.asarray([x[k] for x in valid]);return dict(min=float(v.min()),p50=float(np.median(v)),p90=float(np.quantile(v,.9)),max=float(v.max()))
 results[arm]=dict(first_unsolvable_frame=failure,first_unsolvable_time_s=rows[failure]['local']['time_s'],valid_tracking_rows=len(valid),projection_error_m=stat('projection_error_m'),absolute_root_course_error_deg={k:v*180/np.pi for k,v in dict(p50=float(np.median([abs(x['root_course_error_rad']) for x in valid])),max=max(abs(x['root_course_error_rad']) for x in valid)).items()},speed_m_s=stat('speed'),bank_absolute_tracking_error_rad=dict(p50=float(np.median([abs(x['bank_actual_rad']-x['bank_target_rad']) for x in valid])),max=max(abs(x['bank_actual_rad']-x['bank_target_rad']) for x in valid)),bank_mapping_speed_clamped_fraction=sum(x['speed_clamped'] for x in valid)/len(valid),empirical_error_tube_quantized_m=math.ceil(max(x['projection_error_m'] for x in valid)/.1)*.1,rows=valid)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),results=results,measurement_only=True,empirical_error_not_certified_bound=True,source_model_uncertainty_not_observation=True,old_pursuit_reject_preserved=True,current_bridge_reject_preserved=True,no_new_actual_or_threshold_scan=True,formal35_not_triggered=True,whole_research_complete=False,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(H/'tracking_clearance_budget_audit_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({a:{k:v for k,v in r.items() if k!='rows'} for a,r in results.items()},indent=2))
