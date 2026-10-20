"""Actual current-call diagnostics; no virtual action or future-state labels."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
from benchmark.action_layers import normalize_action_request
from scripts.run_pointgoal_benchmark import load_action_protocol
H=Path(__file__).resolve().parent;D=H/'current_tracking_chain_e1_b0_run_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def validate_request_chain(chain,issued,protocol):
 mapping=chain['calibrated_bank_mapping'];correction=float(chain['yaw_rate_feedback_auxiliary_rad'])
 assert abs(float(mapping['new_aux_rad'])-(float(mapping['encoded_aux_rad'])+correction))<1e-12
 layers=normalize_action_request(protocol,main_velocity=chain['policy_raw_main_command_rad_s'],auxiliary_angle=mapping['new_aux_rad'],source='tracking')
 assert layers.raw_component[1]==chain['policy_raw_auxiliary_target_rad']
 assert layers.protocol_component==(chain['policy_protocol_main_command_rad_s'],chain['policy_protocol_auxiliary_target_rad'])
 assert np.all(np.asarray(issued,np.float32)[1::2]==np.float32(layers.protocol_component[1]))
def validate_complete_timeout_clock(t,n):
 assert n==350 and np.asarray(t).shape==(351,)
 np.testing.assert_allclose(t,np.arange(351)*.1,atol=1e-6,rtol=0)
def main():
 status=json.loads((H/'current_tracking_chain_e1_b0_status_v1.json').read_text());assert status['status']=='finished' and status['child_returncode']==0 and not status['timeout'] and not status['forced_kill'] and status['parent_signal'] is None
 manifest=D/'artifact_manifest_post_close.json';seal=json.loads(manifest.read_text())['artifact_sha256']
 for n,h in seal.items():assert sha(Path(n))==h,n
 rows=json.loads((D/'policy_records.json').read_text());identity=json.loads((D/'identity.json').read_text());states=json.loads((D/'states.json').read_text());termination=json.loads((D/'termination.json').read_text());assert len(states)==len(rows)+1==identity['captures'];assert identity['current_tracking_wrapper_count']==1 and identity['actions']==len(rows)
 cfg=json.loads((H/'current_tracking_chain_e1_protocol_v1.json').read_text());actual_cfg=json.loads((D/'candidate_envelope.json').read_text())['design'];assert cfg==actual_cfg
 for group in ('source_sha256','input_sha256'):
  for n,h in cfg[group].items():assert sha(Path(n))==h,n
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
 n=len(rows);requests=np.load(D/'requests.npz')['requests'];assert requests.shape==(n,10);np.testing.assert_array_equal(requests,np.asarray([r['request'] for r in rows],np.float32))
 physics=np.load(D/'physics.npz');assert len(physics['simulation_time_s'])==5*n+1;np.testing.assert_array_equal(physics['physics_step_index'],np.arange(5*n+1));np.testing.assert_array_equal(physics['raw_command_target'][1:],requests.reshape(-1,2));np.testing.assert_allclose(physics['simulation_time_s'],np.arange(5*n+1)*.02,atol=1e-6,rtol=0)
 clocks=json.loads((D/'camera_clocks.json').read_text());assert len(clocks)==n+1 and [r['frame'] for r in clocks]==list(range(n+1));times=np.asarray([r['episode_time_s'] for r in clocks]);np.testing.assert_allclose(times,np.arange(n+1)*.1,atol=1e-6,rtol=0)
 for c in clocks:assert c['markers_hidden'] and c['eyes']['left']==c['eyes']['right']==c['raw_time_s']
 contacts=json.loads((D/'physics_contacts.json').read_text());assert len(contacts)==6*n+1 and all(c['available'] for c in contacts)
 for kind,expected in [('physics',list(range(1,5*n+1))),('capture',list(range(0,5*n+1,5)))]:assert [c['physics_step_index'] for c in contacts if c['source']==kind]==expected
 full=termination['termination']=='timeout35'
 if full:validate_complete_timeout_clock(times,n);assert not any(c['contact'] for c in contacts)
 elif termination['termination']=='collision':assert any(c['contact'] for c in contacts)
 elif termination['termination']=='success':
  goal=np.asarray(json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'])[:2];assert np.linalg.norm(np.asarray(states[-1]['position'])[:2]-goal)<=.2
 else:raise AssertionError('unclassified termination')
 assert n<=350 and termination['actions']==n and abs(termination['time_s']-times[-1])<1e-6
 current=[];normal=[];skipped=0;unavailable=0;early=[]
 for r in rows:
  chain=r.get('current_tracking_chain');called=r['planner_act_called']
  if not called:assert chain is None;skipped+=1;continue
  assert chain is not None
  if not chain['fresh_tracker_record']:
   assert chain=={'valid':False,'source':'tracker_not_called','fresh_tracker_record':False};unavailable+=1;continue
  current.append(r)
  if not chain['valid'] or chain['source']!='tracking':early.append(dict(frame=r['frame'],source=chain['source'],valid=chain['valid']));continue
  assert chain['calibrated_bank_mapping']['active'];mapping=chain['calibrated_bank_mapping'];assert mapping['direction']==r['tracker_state_after_act']['active_direction'];correction=float(chain['yaw_rate_feedback_auxiliary_rad']);assert abs(float(mapping['new_aux_rad'])-(float(mapping['encoded_aux_rad'])+correction))<1e-12
  validate_request_chain(chain,r['request'],protocol)
  normal.append(dict(frame=r['frame'],time_s=r['time_s'],reference_version=r['local_reference_version'],direction=mapping['direction'],pre_handle_audit_active_direction=chain['active_drive_direction'],requested_curvature_1_m=chain['requested_curvature_1_m'],raw_aux_rad=chain['policy_raw_auxiliary_target_rad'],protocol_aux_rad=chain['policy_protocol_auxiliary_target_rad'],mapped_aux_rad=mapping['encoded_aux_rad'],outer_feedback_aux_rad=correction,final_issued_main_rad_s=r['request'][0],tracker_protocol_main_rad_s=chain['policy_protocol_main_command_rad_s'],mapping_curvature_clamped=mapping['curvature_clamped'],mapping_speed_clamped=mapping['speed_clamped']))
 assert len(current)+unavailable+skipped==len(rows)
 video={}
 assert {p.name for p in D.glob('*.mp4')}=={'stereo_view.mp4','third_person.mp4'}
 for p in D.glob('*.mp4'):
  v=cv2.VideoCapture(str(p));assert abs(v.get(cv2.CAP_PROP_FPS)-10.)<1e-6;n=0
  while True:
   ok,im=v.read()
   if not ok:break
   assert im is not None;n+=1
  v.release();assert n==len(states);video[p.name]=n
 assert len(video)==2
 summaries={}
 for direction in ['forward','reverse']:
  rr=[v for v in normal if v['direction']==direction];c=np.asarray([v['outer_feedback_aux_rad'] for v in rr]);summaries[direction]=dict(rows=len(rr),outer_feedback_abs_p90_rad=float(np.percentile(abs(c),90)) if len(c) else None,mapping_curvature_clamped=sum(v['mapping_curvature_clamped'] for v in rr),mapping_speed_clamped=sum(v['mapping_speed_clamped'] for v in rr),official_protocol_aux_target_would_change_without_outer_feedback=sum(normalize_action_request(protocol,main_velocity=v['tracker_protocol_main_rad_s'],auxiliary_angle=v['mapped_aux_rad'],source='tracking').protocol_component[1]!=v['protocol_aux_rad'] for v in rr),runtime_normalized30_target_would_change_without_outer_feedback=sum(np.float32(np.clip(v['mapped_aux_rad'],-np.pi/6,np.pi/6))!=np.float32(np.clip(v['protocol_aux_rad'],-np.pi/6,np.pi/6)) for v in rr))
 for summary in summaries.values():summary['runtime_normalized30_target_would_change_without_outer_feedback']=int(summary['runtime_normalized30_target_would_change_without_outer_feedback'])
 out=dict(actual_readonly_current_call_telemetry_collected=full,complete_original35_timeout_execution=full,legal_early_end_not_complete35=not full,physics_samples=len(physics['simulation_time_s']),contact_samples=len(contacts),raw_to_official_protocol_to_issued_aux_verified=True,protocol_minimum=list(protocol.minimum),protocol_maximum=list(protocol.maximum),scope='one knownpose TaskH B035 missing-field acquisition; not performanceAB',termination=termination,status=status,actual_policy_rows=len(rows),captures=len(states),prefix_tracker_not_called_rows=skipped,current_tracker_records=len(current),unavailable_tracker_records=unavailable,normal_current_rows=len(normal),early_return_records=early,summaries=summaries,normal_records=normal,decoded_video=video,sealed_items=len(seal),seals_all_match=True,input_sha256={str(p):sha(p) for p in [manifest,H/'current_tracking_chain_e1_protocol_v1.json',H/'current_tracking_chain_telemetry_scope_v1.json',Path(__file__)]},outer_main_producer_vs_tracker_request_separately_reported=True,no_hypothetical_applied_or_trajectory_claim=True,not_control_or_model_admission=True,no_new_navigation_success_rate=True,whole_mission_complete=False)
 p=H/'current_tracking_chain_e1_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ['normal_records','early_return_records','input_sha256']}))
if __name__=='__main__':main()
