"""Read-only actual run review; writes only root review, never sealed run."""
import bootstrap
from bootstrap import D,W
import json,hashlib,numpy as np,importlib.util,sys
V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1'
spec=importlib.util.spec_from_file_location('raw_audit',V/'audit_actual.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit);audit.D=D
arm=sys.argv[1];out=D/(arm+'_run');status=json.loads((D/(arm+'_status.json')).read_text());assert status['status']=='finished' and status['child_returncode']==0
manifest=json.loads((out/'artifact_manifest_post_close.json').read_text());print('manifest keys',list(manifest),flush=True)
# Manifest format handled explicitly from actual sealed file.
for section in ['artifact_sha256','files']:
 if section in manifest and isinstance(manifest[section],dict):
  for p,h in manifest[section].items():
   if isinstance(h,str):assert hashlib.sha256(__import__('pathlib').Path(p).read_bytes()).hexdigest()==h
raw=audit.audit(arm);assert raw==json.loads((D/(arm+'_raw_audit.json')).read_text())
score=json.loads((D/(arm+'_assessment.json')).read_text());states=json.loads((out/'states.json').read_text());contacts=json.loads((out/'physics_contacts.json').read_text());ep=json.loads((out/'task_episode.json').read_text());goal=np.array(ep['actual_episode']['goal_position_world_m'][:2]);initial=float(np.linalg.norm(np.array(states[0]['position'][:2])-goal));remaining=float(np.linalg.norm(np.array(states[-1]['position'][:2])-goal));assert initial==score['initial_distance_m'] and remaining==score['remaining_m'];assert sum(bool(x['contact']) for x in contacts)==score['contacts_detected_by_named_fields'];assert sum(not x['available'] for x in contacts)==score['unavailable_contacts'];errors=[];near=0;points=0;pairs=0
for p in sorted((out/'source_predictions').glob('*.npz')):
 a=np.load(p);f=np.load(out/'frames'/p.name);z=a['depth_m'];gt=f['left_depth_m'].squeeze();v=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(gt)&(gt>0);errors.append(np.abs(z[v]-gt[v]));near+=int((v&(z<1)).sum());pkt=np.load(out/'sensor_packets'/p.name.replace('frame_','source_'));points+=len(pkt['world']);pairs+=1
err=np.concatenate(errors);d=score['depth_scoring'];assert len(err)==d['pixels'] and float(np.quantile(err,.9))==d['p90_absolute_error_m'] and float((err>.3).mean())==d['outlier_fraction_above_0p3m'];assert [near,points,pairs]==[d[k] for k in ['under1m_confident_pixels','source_points','saved_pairs']];depthpass=len(err)>=1000 and np.quantile(err,.9)<=.15 and (err>.3).mean()<=.1;assert depthpass==d['pass_gate'];term=json.loads((out/'termination.json').read_text());assert term==score['termination'];arrival=remaining<=.2 and 0<=states[-1]['time_s']<=90+1e-5 and abs(states[-1]['time_s']-term['time_s'])<=1e-5;assert arrival==score['raw_arrival_pass']
rows=json.loads((out/'policy_records.json').read_text());intervention={}
if arm=='candidate_e1':
 assert ep['only_planning_change']=='future_replanning_reserve_0p2m_beyond_unchanged_safety'
 decisions=[r for r in rows if r.get('guard_measured_this_call')];reserve=[r for r in decisions if (r.get('reference_decision') or {}).get('replanning_reserve') is not None];plans=[r for r in rows if r.get('local_plan_created') and (r.get('plan') or {}).get('status')=='success']
 assert decisions and plans
 for r in plans:
  meta=r['plan'];assert meta['future_replanning_reserve_m']==.2 and abs(meta['mandatory_safety_clearance_m']-.8707106781186548)<1e-12;assert meta['generated_reference_reserve_screen']['pass'];assert abs(meta['planning_clearance_m']-1.0707106781186548)<1e-12
 for r in reserve:
  s=r['reference_decision']['replanning_reserve'];assert s['future_replanning_reserve_m']==.2 and abs(s['reserve_clearance_m']-1.0707106781186548)<1e-12
 intervention=dict(boundaries=len(decisions),reserve_screens=len(reserve),successful_installs=len(plans),fallback_no_route=sum((r.get('plan') or {}).get('status') not in ['success',None] for r in rows))
review=dict(independent_review_pass=True,arm=arm,raw_audit_recomputed_exact=True,source_depth_recomputed_exact=True,manifest_identity_reviewed=True,termination=term,remaining_m=remaining,contacts_detected=score['contacts_detected_by_named_fields'],unavailable_contacts=score['unavailable_contacts'],depth_p90_m=float(np.quantile(err,.9)),depth_gate=bool(depthpass),arrival=bool(arrival),retain=score['retain'],intervention=intervention,scope='Actual terminal/contact/current source/native registration/map/request audit; no independent IMU dynamics reconstruction or causal paired replication. Reserve unverified assumption; sameE1 development.',review_fit=0,review_physical=0)
(D/(arm+'_independent_review.json')).write_text(json.dumps(review,indent=2)+'\n');print(arm,'independent actual review complete',flush=True)
