"""Read-only verification of sealed baseline and memory evidence; no new trials."""
from pathlib import Path
import json,hashlib,sys,csv,datetime
import numpy as np
HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'stereo_memory_navigation_20261004_v1'
sys.path.insert(0,str(OLD))
from baseline35_metrics_v1 import assess
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence,arm
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from reference_continuity_audit import reference_geometry_status
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
verified={}
def verify(table):
 for p,d in table.items():
  assert sha(p)==d,p
  verified[p]=d

def run():
 out=HERE/'existing_evidence_audit_v1.json'
 assert not out.exists(),'immutable audit output'
 index=OLD/'research_phase_reconciliation_assessment_v2.json';prior=json.loads(index.read_text())
 verify(prior['verified_referenced_file_sha256']);verify(prior['source_sha256'])
 mapping={'known_e0':'rolling_endpoint_navigation_e0_paced_run_v1','native_e0':'rolling_endpoint_native_e0_paced_run_v1'}
 for layer in ['known','native']:
  for scene in ['e1','e2']:mapping[f'{layer}_{scene}']=f'rolling_endpoint_coverage_{layer}_{scene}_run_v1'
 rows=[]
 for record in prior['records']:
  r=OLD/mapping.get(record['run'],record['run']);seal=json.loads((r/'artifact_manifest_post_close.json').read_text());verify(seal['artifact_sha256']);verified[str(r/'artifact_manifest_post_close.json')]=sha(r/'artifact_manifest_post_close.json')
  if 'status' in seal:assert seal['status']['status']=='finished' and seal['status']['child_returncode']==0
  states=json.loads((r/'states.json').read_text());contacts=json.loads((r/'physics_contacts.json').read_text());ep=json.loads((r/'task_episode.json').read_text())['actual_episode'];req=np.load(r/'requests.npz')['requests'];physics=np.load(r/'physics.npz');journal=json.loads((r/'policy_records.json').read_text());identity=json.loads((r/'identity.json').read_text())
  assert len(journal)==len(req);np.testing.assert_array_equal(req,np.asarray([x['request'] for x in journal],np.float32));np.testing.assert_array_equal(physics['raw_command_target'][1:],req.reshape(-1,2))
  assert identity['complete'] and identity['reset_count']==1 and not identity['internal_reset']
  normalized=complete_execution_evidence(states,contacts,physics['simulation_time_s'],len(req));metrics=assess(states,normalized,np.asarray(ep['start_position_world_m'][:2]),np.asarray(ep['goal_position_world_m'][:2]))
  for k in ['outcome','success','goal_remaining_m','arrival_time_s','contact_time_s','budget_s']:assert metrics[k]==record[k],(r.name,k)
  assert metrics['along_start_goal_axis_m']==record['net_axis_progress_m']
  scene=ep['episode_id'].split('_')[1];assert scene in ['e0','e1','e2']
  rows.append(dict(record,scene=scene,run_directory=str(r),requests=len(req),captures=len(states),physics_samples=len(physics['simulation_time_s']),contact_samples=len(contacts),sealed_files=len(seal['artifact_sha256']),actual_score_state_time_s=metrics['metric_state_time_s'],actual_score_end_s=metrics['scored_end_s']))
 assert len(rows)==12 and len({(x['variant'],x['pose_layer'],x['scene']) for x in rows})==12
 memory_design=json.loads((OLD/'sgbm_risk_lifetime_design_v1.json').read_text());verify(memory_design['source_sha256']);verify(memory_design['input_sha256'])
 memory_old=json.loads((OLD/'sgbm_risk_lifetime_assessment_v1.json').read_text());arms={}
 for name in ['history','current']:
  measured,*_=arm(name,memory_design)
  assert measured==memory_old[name],name
  arms[name]=measured
 # Recheck existing same-exposure predicate only; no online action or alternate trajectory.
 p=OLD/'same_exposure_retention_assessment_v1.json';same=json.loads(p.read_text());assert sha(OLD/'audit_same_exposure_retention_v1.py')==same['source_sha256']
 current=OLD/'sgbm_risk_lifetime_e1_current_run_v1';assert sha(current/'artifact_manifest_post_close.json')==same['input_seal_sha256']
 states=json.loads((current/'states.json').read_text());records=json.loads((current/'policy_records.json').read_text());ep=json.loads((current/'task_episode.json').read_text())['actual_episode'];grid=RepeatedExposureMemoryGrid(np.asarray(ep['start_position_world_m'][:2])-20,10,static_scene_assumption=True);checks=[]
 for r in records:
  f=r['frame'];snap=json.loads((current/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text());grid.set_source(dict(frame=r['source_frame'],time_s=r['source_time_s'],records=snap['records']));root=np.asarray(states[f]['position']);grid.update_points([],root,r['time_s'])
  if not r.get('guard_measured_this_call') or r['reference_decision']['guard'] is None:continue
  path=records[f-1]['installed_reference_xy_after_act']
  with np.load(current/f'maps/frame_{f:03d}.npz') as m:
   assert not np.any(m['labels']==0)
   actual=reference_geometry_status(root[:2],path,m['labels'],m['origin'],float(m['resolution']))
  historical=reference_geometry_status(root[:2],path,grid.labels,grid.origin,grid.resolution)
  assert actual==r['reference_decision']['guard']
  checks.append(dict(frame=f,actual_pass=actual['retention_necessary_screen_pass'],history_pass=historical['retention_necessary_screen_pass']))
 assert len(checks)==same['boundaries_exact']==30
 assert [x['frame'] for x in checks if x['actual_pass'] and not x['history_pass']]==[x['frame'] for x in same['actual_retain_but_history_reject']]
 for p in [index,OLD/'sgbm_risk_lifetime_assessment_v1.json',OLD/'same_exposure_retention_assessment_v1.json',Path(__file__)]:verified[str(p)]=sha(p)
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),matrix=rows,success_counts=prior['success_counts'],memory=dict(arms=arms,verified_same_exposure_boundaries=30,changed_retention_boundaries=11,mechanism='same current-arm past source and current pose/reference; predicate only',full_causal_navigation_claim=False,memory_navigation_benefit=False,source_prefix_files_identical=memory_old['prefix_source_input_files_byte_identical'],raw_geometry_prefix_identical=memory_old['prefix_raw_point_geometry_exact']),verified_sha256=verified,new_physics=0,new_policy_act=0,new_model_queries=0,new_teacher_queries=0,new_training=0,whole_mission_complete=False)
 out.write_text(json.dumps(result,indent=2)+'\n')
 with (HERE/'baseline_12_conditions.csv').open('w',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
 print(json.dumps(dict(matrix_conditions=12,verified_files=len(verified),memory_boundaries=30,changed_retention_boundaries=11,new_physics=0)))
if __name__=='__main__':run()
