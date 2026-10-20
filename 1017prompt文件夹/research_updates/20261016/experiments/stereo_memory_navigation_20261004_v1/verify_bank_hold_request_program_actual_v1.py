"""Independent physical GIVEN-program assessment, never navigation admission."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
from scipy.spatial.transform import Rotation
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
from bank_hold_request_program import BankHoldRequestProgram
H=Path(__file__).resolve().parent
O=H/'bank_hold_request_program_actual_verification_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((H/'bank_hold_request_program_stage_status_v1.json').read_text())
assert all(status['arms'][a]['complete'] and status['arms'][a]['returncode']==0 for a in ['B0','C'])
protocol=json.loads((H/'bank_hold_request_program_protocol_v1.json').read_text())
for section in ['source_sha256','input_sha256']:
 assert all(sha(f)==s for f,s in protocol[section].items()),section
given=json.loads((H/'given_raw_request_program_v1.json').read_text())
O.mkdir(exist_ok=False)
summaries={};traces={};requests={};states={}
for arm in ['B0','C']:
 D=H/('bank_hold_request_program_'+arm+'_run_v1')
 seal=D/'artifact_manifest_post_close.json'
 assert sha(seal)==status['arms'][arm]['post_close_manifest_sha256']
 assert all(sha(f)==s for f,s in json.loads(seal.read_text())['artifact_sha256'].items())
 ph=np.load(D/'physics.npz');rows=json.loads((D/'policy_records.json').read_text());N=len(rows)
 es=json.loads((D/'execution_states.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text())
 assert len(es)==len(clocks)==N+1 and len(ph['applied_action'])==5*N+1
 assert es[0] is None and np.all(np.asarray(rows[0]['request'])==0)
 program=BankHoldRequestProgram(given['requests'],given['modes'],arm);ticks=0
 for i,row in enumerate(rows):
  pose=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz')
  q=Rotation.from_matrix(pose['root_rotation_world']).as_quat()
  bank=bank_from_quaternion(q[[3,0,1,2]])
  expected=program.act(i,bank,float(pose['time_s']),float(clocks[i]['time_s']))
  np.testing.assert_array_equal(expected,np.asarray(row['request'],np.float32))
  assert not row['planner_act_called'] and row['local']['request_program_only']
  assert not row['local']['RGB_or_map_consumed'] and row['local']['mode_is_given_program_not_actual_tracker']
  assert row['local']['entry_bank_from_current_native_rad']==bank
  if i:
   clone=ExecutionClone.from_snapshot(es[i])
   for j,command in enumerate(expected.reshape(5,2)):
    k=5*i+j
    applied,smoothed=clone.step(command,bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][k]),aux_position_rad=ph['joint_position_rad'][k,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][k,1])
    np.testing.assert_array_equal(applied,ph['applied_action'][k+1]);np.testing.assert_array_equal(smoothed,ph['smoothed_target'][k+1]);ticks+=1
 assert ticks==5*(N-1)
 videos={}
 for name in ['stereo_view','third_person']:
  c=cv2.VideoCapture(str(D/(name+'.mp4')));n=0
  while True:
   ok,_=c.read()
   if not ok:break
   n+=1
  c.release();assert n==N+1;videos[name]=n
 contacts=json.loads((D/'contacts.json').read_text());pc=json.loads((D/'physics_contacts.json').read_text())
 assert len(contacts)==N+1
 pos=ph['root_position_world_m'];xy=np.linalg.norm(pos[:,:2]-[2.2,.15],axis=1);xyz=np.linalg.norm(pos-[2.2,.15,.45],axis=1)
 time=ph['simulation_time_s'];inside=np.flatnonzero(xy<=.2);pause=5*555
 metrics=json.loads((D/'metrics.json').read_text())
 summaries[arm]=dict(request_blocks=N,physics_samples=len(pos),captures=N+1,execution_clones=ticks,videos=videos,contact_capture_count=sum(v['contact'] for v in contacts),contact_physics_count=sum(bool(v.get('physical_contact',v.get('contact',False))) for v in pc),physics_contact_record_keys=list(pc[0]) if pc else [],closest_true_XY_m=float(xy.min()),closest_time_s=float(time[xy.argmin()]),final_true_XY_m=float(xy[-1]),closest_true_XYZ_m=float(xyz.min()),true_XY_entry_s=float(time[inside[0]]) if len(inside) else None,true_path_length_m=float(np.linalg.norm(np.diff(pos[:,:2],axis=0),axis=1).sum()),elapsed_s=metrics['elapsed_s'],termination=metrics['termination_reason'],given_pause_radial_peak_next_1p5_s_m=float(np.linalg.norm(pos[pause:min(pause+76,len(pos)),:2]-pos[pause,:2],axis=1).max()),postseal_sha256=sha(seal),actual_program_reconstruction_exact=True,NN_inference_count=len(list((D/'source_predictions').glob('*'))),not_navigation_result=True)
 traces[arm]={f:ph[f].copy() for f in ph.files};requests[arm]=np.asarray([r['request'] for r in rows]);states[arm]=es
 b=np.asarray(given['requests'][:N],np.float32).reshape(N,5,2);r=requests[arm].reshape(N,5,2)
 np.testing.assert_array_equal(r[:,:,0],b[:,:,0]);changed=np.flatnonzero(np.any(r[:,:,1]!=b[:,:,1],axis=1))
 assert arm!='B0' or not len(changed)
 assert all(given['modes'][i]=='GIVEN_PROGRAM_GOAL_PAUSE' for i in changed)
 summaries[arm]['changed_program_frames']=changed.tolist()
n=min(len(traces['B0']['applied_action']),len(traces['C']['applied_action']))
different=np.flatnonzero(np.any(traces['B0']['applied_action'][:n]!=traces['C']['applied_action'][:n],axis=1))
first=int(different[0]) if len(different) else None
fields=['root_position_world_m','root_quaternion_world_wxyz','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','smoothed_target']
prefix={f:np.array_equal(traces['B0'][f][:first],traces['C'][f][:first]) if first is not None else False for f in fields}
prefix['capture_execution_states']=states['B0'][:556]==states['C'][:556]
prefix['requests_before_pause']=np.array_equal(requests['B0'][:555],requests['C'][:555])
physical_prefix=all(prefix.values())
contacts_zero=all(s['contact_capture_count']==s['contact_physics_count']==0 for s in summaries.values())
newentry=summaries['C']['true_XY_entry_s'] is not None and summaries['B0']['true_XY_entry_s'] is None
decision='PASS_GIVEN_PROGRAM_PHYSICAL_TARGET_ENTRY_ONLY' if physical_prefix and contacts_zero and newentry else 'REJECT_GIVEN_PROGRAM_PHYSICAL_AB_PREFIX' if not physical_prefix else 'REJECT_NO_NEW_GIVEN_PROGRAM_TARGET_ENTRY'
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision=decision,arms=summaries,prefix_exact=prefix,first_applied_difference_tick=first,first_applied_difference_time_s=float(traces['C']['simulation_time_s'][first]) if first is not None else None,collision_free=contacts_zero,new_target_entry=newentry,not_online_B0=True,not_navigation_success=True,not_formal35=True,not_fresh_heldout=True,whole_research_complete=False,code_sha256=sha(__file__),protocol_sha256=sha(H/'bank_hold_request_program_protocol_v1.json'))
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
