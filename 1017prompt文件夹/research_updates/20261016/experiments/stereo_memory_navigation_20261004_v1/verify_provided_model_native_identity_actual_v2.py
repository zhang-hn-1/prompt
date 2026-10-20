"""Independent closed-actual identity assessment; no navigation admission."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
H=Path(__file__).resolve().parent
D=H/'provided_model_native_identity_B0_run_v1'
B=H/'native_flat_pose_navigation_R1_run_v1'
O=H/'provided_model_native_identity_actual_verification_v2'
O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
st=json.loads((H/'provided_model_native_identity_stage_status_v1.json').read_text())['arms']['B0']
assert st['complete'] and st['returncode']==0
seal=D/'artifact_manifest_post_close.json'
assert sha(seal)==st['post_close_manifest_sha256']
manifest=json.loads(seal.read_text())['artifact_sha256']
assert all(sha(f)==s for f,s in manifest.items())
protocol=json.loads((H/'provided_model_native_identity_protocol_v1.json').read_text())
assert all(sha(f)==s for f,s in protocol['source_sha256'].items())
assert all(sha(f)==s for f,s in protocol['input_sha256'].items())
sensor=json.loads((H/'provided_model_native_identity_B0_sensor_verification_v2.json').read_text())
ph=np.load(D/'physics.npz');bp=np.load(B/'physics.npz')
rows=json.loads((D/'policy_records.json').read_text());br=json.loads((B/'policy_records.json').read_text())
es=json.loads((D/'execution_states.json').read_text());be=json.loads((B/'execution_states.json').read_text())
N=len(rows)
assert len(es)==N+1 and len(ph['applied_action'])==5*N+1
comparisons={};firsts={}
def compare(label,a,b):
 a=np.asarray(a);b=np.asarray(b)
 comparisons[label]=a.shape==b.shape and np.array_equal(a,b)
 n=min(len(a),len(b));same=np.equal(a[:n],b[:n]).reshape(n,-1).all(1)
 bad=np.flatnonzero(~same)
 firsts[label]=int(bad[0]) if len(bad) else n if a.shape!=b.shape else None
for f in ['root_position_world_m','root_quaternion_world_wxyz','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','smoothed_target','raw_command_target','clipped_target']:
 compare('physics_'+f,ph[f],bp[f])
compare('current_requests',[r['request'] for r in rows],[r['request'] for r in br])
comparisons['execution_states']=es==be
firsts['execution_states']=next((i for i,(a,b) in enumerate(zip(es,be)) if a!=b),None if len(es)==len(be) else min(len(es),len(be)))
poseexact=refexact=mapexact=True
firstpose=firstref=firstmap=None
mapfields=cloneticks=blockedframes=0
for i in range(N+1):
 pose=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz')
 old=np.load(B/'estimated_poses'/f'frame_{i:03d}.npz')
 if pose.files!=old.files or any(not np.array_equal(pose[k],old[k]) for k in pose.files):
  poseexact=False
  if firstpose is None:firstpose=i
 if i<N:
  if es[i]['blocked']:
   blockedframes+=1
   assert np.all(np.asarray(rows[i]['request'])==0)
  else:
   clone=ExecutionClone.from_snapshot(es[i])
   for j,cmd in enumerate(np.asarray(rows[i]['request']).reshape(5,2)):
    p=5*i+j
    ap,sm=clone.step(cmd,bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][p]),aux_position_rad=ph['joint_position_rad'][p,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][p,1])
    np.testing.assert_array_equal(ap,ph['applied_action'][p+1])
    np.testing.assert_array_equal(sm,ph['smoothed_target'][p+1])
    cloneticks+=1
 if 140<=i<N:
  if rows[i]['local']['installed_reference_xy_after_act']!=br[i]['local']['installed_reference_xy_after_act']:
   refexact=False
   if firstref is None:firstref=i
  a=np.load(D/'maps'/f'frame_{i:03d}.npz');b=np.load(B/'maps'/f'frame_{i:03d}.npz')
  for f in ['labels','provenance','safe_start_prior','task_ground_prior','hypothesis_birth_time']:
   mapfields+=1
   if not np.array_equal(a[f],b[f]):
    mapexact=False
    if firstmap is None:firstmap=i
  assert not np.isfinite(a['floor_time']).any() and not np.isfinite(a['hazard_time']).any()
  assert rows[i]['local']['source_is_clock_only'] and not rows[i]['local']['current_geometry_observed']
prior=np.load(protocol['provided_prior_seed']);provided=np.load(D/'frozen_memory_seed.npz')
geometry=all(np.array_equal(provided[k],prior[k]) for k in ['possible_xy','raw_world_voxel_centres'])
assert str(provided['source_consumer'])=='PROVIDED_HISTORICAL_MODEL_CONTROL_DIAGNOSTIC'
provenance=json.loads((D/'provided_model_provenance.json').read_text())
assert provenance['prior_sha256']==sha(protocol['provided_prior_seed']) and not provenance['current_observation']
videos={}
for name in ['stereo_view','third_person']:
 c=cv2.VideoCapture(str(D/(name+'.mp4')));n=0
 while True:
  ok,_=c.read()
  if not ok:break
  n+=1
 c.release();videos[name]=n
checks=dict(physics_and_requests_and_execution_exact=all(comparisons.values()),native_estimated_pose_exact=poseexact,complete_current_reference_exact=refexact,current_model_map5fields_exact=mapexact,provided_geometry_exact=geometry,pose_original_gate=sensor['pose_gate'],full_expected_counts=N==600 and sensor['sensor_samples_exact']==3001 and all(v==601 for v in videos.values()),measured_execution_reconstruction_exact=cloneticks==5*(N-blockedframes),no_current_NN_source=len(list((D/'source_predictions').glob('*')))==0)
metrics=json.loads((D/'metrics.json').read_text())
dist=np.linalg.norm(ph['root_position_world_m'][:,:2]-[2.2,.15],axis=1)
contacts=json.loads((D/'contacts.json').read_text())
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_PHYSICAL_PROVIDED_MODEL_B0_IDENTITY_ONLY' if all(checks.values()) else 'REJECT_PHYSICAL_PROVIDED_MODEL_B0_IDENTITY',checks=checks,comparisons=comparisons,first_differences=firsts,first_native_pose_difference=firstpose,first_reference_difference=firstref,first_map_difference=firstmap,current_map_fields_checked=mapfields,measured_execution_ticks=cloneticks,blocked_frames=blockedframes,videos=videos,diagnostic_only=dict(elapsed_s=metrics['elapsed_s'],termination=metrics['termination_reason'],native_success=metrics['success'],last_true_XY_distance_m=float(dist[-1]),closest_true_XY_distance_m=float(dist.min()),contacts=sum(v['contact'] for v in contacts)),not_live_RGB_geometry=True,not_source_admission=True,not_navigation_admission=True,not_fixed_reference_control_AB=True,no_controller_candidate=True,no_formal35=True,old_rejects_preserved=True,whole_research_complete=False,code_sha256=sha(__file__),postseal_sha256=sha(seal))
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(r,indent=2))
