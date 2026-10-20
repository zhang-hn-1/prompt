"""One approved post-seal evaluation/ABC comparison; no policy/source rerun."""
from pathlib import Path
import sys,json,hashlib,datetime,importlib.util,types,runpy
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[2];B=W/'experiments/four_camera_weak_baseline_20261017_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def main():
 ex=json.loads((D/'execution_protocol.json').read_text());proposal=json.loads((D/'proposal.json').read_text());approval=json.loads((D/'approval.json').read_text());assert approval['user_explicit_authorization'] and approval['proposal_sha256']==sha(D/'proposal.json')
 for p,h in ex['source_sha256'].items():assert sha(p)==h,p
 for p,h in proposal['frozen_identity'].items():assert sha(p)==h,p
 s=json.loads((D/'navigation_status.json').read_text());assert s['status']=='finished' and s['child_returncode']==0 and not s['remaining_live_group_pids']
 out=D/'e1_run';cap=json.loads((out/'capture_seal.json').read_text());source=json.loads((out/'source_seal.json').read_text());term=json.loads((out/'terminal.json').read_text());assert cap['complete'] and source['complete'] and term['exit_code']==0 and len(cap['rows'])==len(source['rows'])
 for c,r in zip(cap['rows'],source['rows']):
  assert sha(out/'frames'/c['frame'])==c['sha256'] and c['sha256']==r['input_sha256'];assert sha(out/'predictions'/c['frame'])==r['prediction_sha256']
 # Completeness gate is structural/policy/video-header only; no numeric GT arrays yet.
 required=['physics.npz','requests.npz','contacts.json','native_imu.json','policy_records.json','episode.json','stage_identity.json','stage_root_initial.usda','loaded_source_identity.json','initial_execution.json','initial_gauge.json','scene_primitives.json']
 for name in required:assert (out/name).is_file() and (out/name).stat().st_size>0,name
 actions=int(term['executed_blocks']);assert 1<=actions<=900 and int(term['frames'])==len(cap['rows'])
 policy_pre=json.loads((out/'policy_records.json').read_text());assert len(policy_pre)==actions
 with np.load(out/'requests.npz') as a:assert a['requests'].shape==(actions,4)
 with np.load(out/'physics.npz') as a:assert {'root_position_world_m','raw_simulation_time_s'}<=set(a.files)
 native=json.loads((out/'native_imu.json').read_text());assert len(native)>=actions*5 and all(x['qualified'] for x in native)
 for i in range(actions):
  for directory,suffix in [('maps','.npz'),('plans','.json')]:assert (out/directory/f'frame_{i:03d}{suffix}').is_file()
 assert len(list((out/'maps').glob('frame_*.npz')))==actions and len(list((out/'plans').glob('frame_*.json')))==actions
 assert len(list((out/'frames').glob('frame_*.npz')))==len(cap['rows']) and len(list((out/'predictions').glob('frame_*.npz')))==len(source['rows'])
 assert cap['camera_names']==['front_left','front_right','rear_left','rear_right']
 import cv2
 video_counts={}
 for eye in cap['camera_names']:
  file=out/(eye+'.mp4');assert file.is_file() and file.stat().st_size>0
  video=cv2.VideoCapture(str(file));assert video.isOpened();count=int(video.get(cv2.CAP_PROP_FRAME_COUNT));video.release();assert count==len(cap['rows']),(eye,count,len(cap['rows']));video_counts[eye]=count
 put(D/'C_completeness_before_GT.json',dict(policy_actions=actions,source_frames=len(source['rows']),map_plan_pairs=actions,native_IMU_packets=len(native),video_header_frame_counts=video_counts,numeric_GT_array_reads=0,proposal_sha256=sha(D/'proposal.json')))
 assert not (out/'artifact_manifest_post_close.json').exists()
 files=sorted(p for p in out.rglob('*') if p.is_file());art={str(p):sha(p) for p in files};manifest=out/'artifact_manifest_post_close.json'
 put(manifest,dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),execution_status=s,artifact_sha256=art,whole_run_complete=True,purpose='Seal actualC complete source/maps/actions/IMU/physics/trajectory/video before numeric GT evaluation',proposal_sha256=sha(D/'proposal.json')))
 def verify():
  for p,h in art.items():assert sha(p)==h,p
 verify()
 # Same evaluator kernel as archived B; only output root and descriptive arm name change.
 sys.path[:0]=[str(B),str(W/'experiments/stereo_memory_navigation_20261004_v1')]
 spec=importlib.util.spec_from_file_location('C_sealed_same_evaluator',str(B/'evaluate_navigation.py'));ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev);ev.D=D;ev.main()
 result=json.loads((D/'e1_result.json').read_text());result['comparison']='One prospective originalC negativecontrol vs known A/B, original C offline rejection preserved';put(D/'e1_result.json',result)
 # Import-time diagnostic helpers inherit D from paths: isolated child namespace shim.
 import paths as basepaths
 shim=types.ModuleType('paths');shim.__dict__.update(vars(basepaths));shim.D=D;sys.modules['paths']=shim
 for helper in ['supported_memory_diagnostic.py','diagnose_no_route.py']:runpy.run_path(str(B/helper),run_name='__main__')
 supported=json.loads((D/'supported_memory_result.json').read_text());supported['scope']=supported['scope'].replace('actual B','actual C negative-control');put(D/'supported_memory_result.json',supported)
 verify();put(D/'evaluation_receipt.json',dict(verified_before_after=True,whole_run_manifest_sha256=sha(manifest),evaluator_sha256=sha(B/'evaluate_navigation.py'),result_sha256=sha(D/'e1_result.json'),supported_sha256=sha(D/'supported_memory_result.json'),no_source_or_physics_rerun=True))
 cfg=json.loads((D/'comparison_protocol.json').read_text())
 for p,h in cfg['reference_sha256'].items():assert sha(p)==h,p
 old=json.loads((W/'experiments/current_exposure_baseline_navigation_20261017_v1/comparison.json').read_text());arms=old['arms'].copy();policy=json.loads((out/'policy_records.json').read_text());req=np.load(out/'requests.npz')['requests']
 no=np.array([r['time_s']>=6 and max(abs(v) for v in r['request'])==0 and r['plan'].get('status')!='success' for r in policy]);run=maxrun=0;first=None
 for i,b in enumerate(no):
  run=run+1 if b else 0;maxrun=max(maxrun,run)
  if run>=50 and first is None:first=i-49
 c=dict(root=str(D),termination=result['termination'],contact_stop=result['termination']=='physical_contact',actions=result['actions'],frames=result['frames'],sim_time_s=result['sim_time_s'],net_goal_progress_m=result['net_goal_progress_m'],path_length_m=result['path_length_m'],request_reverse_time_s=result['request_reverse_time_s'],tracker_pending_direction_time_s=supported['tracker_pending_direction_time_s'],source_supported_blind_denominator=supported['blind_previously_source_supported_target_queries'],source_supported_blind_missing=supported['missing_queries'],source_supported_blind_missing_fraction=supported['missing_fraction'],false_map_cell_seconds=result['memory']['false_map_cell_seconds'],max_false_map_cells=result['memory']['max_false_map_cells'],first_sustained_no_route_policy=first,first_sustained_no_route_time_s=policy[first]['time_s'] if first is not None else None,max_sustained_no_route_s=maxrun*.1,total_no_route_s=float(no.sum()*.1),source=result['source'],full_chain_P50_P90_P95_s=result['end_to_end_policy_execute_render_perception_P50_P90_P95_s']);arms['C']=c
 divergence={}
 for name in ['A','B']:
  root=Path(arms[name]['root']);archived_manifest=json.loads((root/'e1_run/artifact_manifest_post_close.json').read_text());request_path=root/'e1_run/requests.npz';assert sha(request_path)==archived_manifest['artifact_sha256'][str(request_path)]
  prior=np.load(request_path)['requests'];n=min(len(prior),len(req));diff=np.flatnonzero(np.max(abs(req[:n]-prior[:n]),axis=1)>1e-8);index=int(diff[0]) if len(diff) else None
  oldcap=json.loads((root/'e1_run/capture_seal.json').read_text());rgb=[]
  for i,(new,older) in enumerate(zip(cap['rows'],oldcap['rows'])):
   if index is not None and new['request_blocks_executed']>index:break
   assert sha(root/'e1_run/frames'/older['frame'])==older['sha256'] and sha(out/'frames'/new['frame'])==new['sha256']
   with np.load(out/'frames'/new['frame']) as x,np.load(root/'e1_run/frames'/older['frame']) as y:
    per={}
    for eye in ['front_left','front_right','rear_left','rear_right']:
     aa=x[eye+'_rgb'];bb=y[eye+'_rgb'];delta=abs(aa.astype(float)-bb.astype(float));per[eye]=dict(byte_equal=bool(np.array_equal(aa,bb)),MAE_uint8=float(delta.mean()),changed_pixels=int(np.any(aa!=bb,axis=2).sum()))
   rgb.append(dict(frame=i,C_request_blocks=new['request_blocks_executed'],archived_request_blocks=older['request_blocks_executed'],per_eye=per))
  divergence[name]=dict(first_action_divergence_policy=index,time_s=index*.1 if index is not None else None,C_vs_archived_common_action_prefix_RGB=rgb)
 t=cfg['C_development_thresholds'];gates=dict(minimum_visual_support=result['minimum_observation_support'],contact_noninferiority=not c['contact_stop'],net_progress_benefit=c['net_goal_progress_m']>=t['net_goal_progress_min_m'],false_memory_benefit=c['false_map_cell_seconds']<=t['false_map_cellseconds_max'],blind_missing_noninferiority=c['source_supported_blind_missing_fraction'] is not None and c['source_supported_blind_missing_fraction']<=t['source_supported_blind_missing_fraction_max'],latency_noninferiority=c['full_chain_P50_P90_P95_s'][1]<=t['full_chain_P90_max_s'])
 put(D/'ABC_comparison.json',dict(complete=True,arms=arms,C_vs_archived_divergence=divergence,C_frozen_development_gates=gates,all_C_development_gates_pass=all(gates.values()),original_C_offline_rejection_unchanged=True,one_actual_run_per_arm=True,A_B_rerun=False,C_actual_user_exception_sha256=sha(D/'approval.json'),proposal_sha256=sha(D/'proposal.json'),scope=cfg['scope'],no_route_proxy_not_false_block_proof=True,false_block_diagnostic=str(D/'no_route_diagnosis.json'),learning_increment=False,reliable_navigation=False,whole_research_complete=False))
 for p,h in ex['source_sha256'].items():assert sha(p)==h,p
 for p,h in proposal['frozen_identity'].items():assert sha(p)==h,p
 verify();print('C_ABC_COMPARISON',json.dumps(c),json.dumps(gates),flush=True)
if __name__=='__main__':main()
