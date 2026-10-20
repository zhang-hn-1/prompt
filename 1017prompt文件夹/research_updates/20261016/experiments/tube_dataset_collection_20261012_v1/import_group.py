"""Sealed local-run consistency; not external attestation or statistical independence."""
from pathlib import Path
import json,hashlib,math,sys
from scipy.spatial.transform import Rotation
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"stereo_memory_navigation_20261004_v1"))
from native_rolling_shadow import NativeRollingShadow
from imu_interval_packet import interval_packet
from flat_ground_pose_bridge import FlatGroundPoseBridge
import numpy as np
from support import derive_support,TEMPLATES,f32

def read_group(run,status_path,seal_path,expected_group):
 run=Path(run);status_path=Path(status_path);status=json.loads(status_path.read_text());seal=json.loads(Path(seal_path).read_text());identities=seal['artifact_sha256']
 if status['status']!='finished' or status['child_returncode']!=0:raise ValueError('normal durable terminal required')
 required={str(f) for f in run.rglob('*') if f.is_file()}|{str(status_path)}
 if not required.issubset(identities):raise ValueError('run seal coverage incomplete')
 if any(hashlib.sha256(Path(f).read_bytes()).hexdigest()!=sha for f,sha in identities.items()):raise ValueError('sealed content changed')
 group=json.loads((run/'group_binding.json').read_text())
 if group!=expected_group:raise ValueError('schedule group mismatch')
 if json.loads((run/'task_episode.json').read_text())!=group['runtime_binding']['episode']:raise ValueError('initial episode mismatch')
 admission=json.loads((run/'source_admission.json').read_text())
 if any(hashlib.sha256(Path(f).read_bytes()).hexdigest()!=sha for f,sha in admission['source_sha256'].items()):raise ValueError('run source identity changed')
 windows=json.loads((run/'windows.json').read_text());physics=np.load(run/'physics.npz',allow_pickle=False);records=[]
 reads=json.loads((run/'native_reads.json').read_text());rolling_saved=json.loads((run/'rolling_records.json').read_text());gauge=json.loads((run/'initial_gauge.json').read_text())
 if len(reads)!=len(physics['simulation_time_s']) or not all(r['qualified'] for r in reads):raise ValueError('native qualification incomplete')
 normal=Rotation.from_quat(physics['root_quaternion_world_wxyz'][0][[1,2,3,0]]).as_matrix().T@np.array([0.,0.,1.]);shadow=NativeRollingShadow()
 for ix,r in enumerate(reads):
  if r['time_s']!=float(physics['simulation_time_s'][ix]):raise ValueError('native time mismatch')
  reading=SimpleNamespace(time=r['sensor_begin_stamp_s'],is_valid=r['is_valid'],ang_vel_x=r['gyro_body_rad_s'][0],ang_vel_y=r['gyro_body_rad_s'][1],ang_vel_z=r['gyro_body_rad_s'][2],lin_acc_x=r['linear_acceleration_body_m_s2'][0],lin_acc_y=r['linear_acceleration_body_m_s2'][1],lin_acc_z=r['linear_acceleration_body_m_s2'][2])
  sensor,checked=interval_packet(reading,r['begin_raw_s'],r['end_raw_s'],r['time_s'],float(physics['joint_velocity_rad_s'][ix][0]))
  if sensor is None or checked!=r or abs(r['end_raw_s']-float(physics['raw_simulation_time_s'][ix]))>1e-10 or (ix>0 and abs(r['begin_raw_s']-float(physics['raw_simulation_time_s'][ix-1]))>1e-10):raise ValueError('native interval provenance mismatch')
  shadow.consume(dict(time_s=r['time_s'],gyro_body_rad_s=r['gyro_body_rad_s'],linear_acceleration_body_m_s2=r['linear_acceleration_body_m_s2'],main_encoder_rad_s=float(physics['joint_velocity_rad_s'][ix][0])),normal if ix==0 else None)
 if shadow.records!=rolling_saved:raise ValueError('native reconstruction mismatch')
 pose=FlatGroundPoseBridge(gauge['position'],gauge['rotation'])
 if not np.array_equal(gauge['position'],physics['root_position_world_m'][0]) or not np.allclose(gauge['rotation'],Rotation.from_quat(physics['root_quaternion_world_wxyz'][0][[1,2,3,0]]).as_matrix(),atol=1e-12,rtol=0):raise ValueError('initial gauge mismatch')
 contacts=json.loads((run/'contact_samples.json').read_text());clocks=json.loads((run/'capture_clocks.json').read_text())
 if not contacts or any(r['available'] is not True or type(r['contact']) is not bool or not math.isfinite(r['simulation_time_s']) for r in contacts):raise ValueError('contact measurement unavailable')
 for r in rolling_saved:pose.consume(r)
 if len(windows)>70 or [w['window'] for w in windows]!=list(range(len(windows))):raise ValueError('invalid attempted windows')
 for window in windows:
  if window['scorable'] is not True:continue
  i=window['window'];rows=[json.loads(line) for line in (run/f'window_{i:03d}.jsonl').read_text().splitlines()];commits=[r for r in rows if r['type']=='commit'];feedback=[r for r in rows if r['type']=='block_feedback']
  if len(commits)!=1 or len(feedback)!=5:raise ValueError('complete window lacks records')
  required_types=['anchor_source_receipt','commit','initial_supervision']+['block_dispatch_intent','block_feedback']*5
  if [r['type'] for r in rows]!=required_types:raise ValueError('write-ahead journal order incomplete')
  receipt=rows[0]['receipt'];initial=rows[2];snapshot=bytes.fromhex(commits[0]['snapshot_hex'])
  if hashlib.sha256(snapshot).hexdigest()!=commits[0]['snapshot_sha256']:raise ValueError('snapshot checksum mismatch')
  program=commits[0]['program']
  expected_program=[list(map(f32,TEMPLATES[group['window_templates'][i]]))]*25
  if program!=expected_program:raise ValueError('scheduled program mismatch')
  envelope=json.loads(bytes.fromhex(commits[0]['snapshot_hex']));online=envelope['online_state'][0];offset=i*25
  if len(program)!=25 or not math.isfinite(online['time_s']) or abs(online['time_s']-float(physics['simulation_time_s'][offset]))>1e-7:raise ValueError('anchor clock mismatch')
  if initial['time_s']!=online['time_s'] or not np.array_equal(initial['world_xy'],physics['position_world_xy'][offset]):raise ValueError('anchor supervision mismatch')
  if envelope['source']!=dict(source='native_imu_rolling',initial_gauge=True,flat_ground_prior=True):raise ValueError('source privilege mismatch')
  if not all(math.isfinite(t) for t in (receipt['time_s'],receipt['raw_time_s'],receipt['left_render_time_s'],receipt['right_render_time_s'],*receipt['clock_metadata'])):raise ValueError('nonfinite camera clock')
  if receipt['clock_metadata']!=clocks[i*5]:raise ValueError('capture clock mismatch')
  if any(r['contact'] and online['time_s']-1e-7<=r['simulation_time_s']<=online['time_s']+.5+1e-7 for r in contacts):raise ValueError('complete window contains contact')
  if receipt['clock_metadata'][0]!=online['time_s'] or receipt['clock_metadata'][2]!=receipt['raw_time_s'] or abs(receipt['raw_time_s']-float(physics['raw_simulation_time_s'][offset]))>1e-7:raise ValueError('same-runtime source clock mismatch')
  for eye in ('left','right'):
   rgb=bytes.fromhex(envelope[eye+'_rgb_hex'])
   if len(rgb)!=480*640*3 or hashlib.sha256(rgb).hexdigest()!=receipt[eye+'_rgb_sha256'] or abs(receipt[eye+'_render_time_s']-receipt['raw_time_s'])>1e-5:raise ValueError('RGB source mismatch')
  estimate=pose.at(online['time_s'])
  if not np.array_equal(online['estimated_xy'],estimate['root_position_world_m'][:2]) or not np.array_equal(online['estimated_velocity'],estimate['velocity_world_m_s'][:2]) or online['estimated_heading']!=estimate['yaw_world_rad']:raise ValueError('online pose reconstruction mismatch')
  for b,block in enumerate(feedback):
   intent=rows[3+b*2]
   if intent['index']!=b or intent['program']!=program[b*5:b*5+5]:raise ValueError('committed block intent mismatch')
   if block['index']!=b or block['physical_contact'] or block['collision'] or len(block['records'])!=5:raise ValueError('invalid complete block')
   for j,r in enumerate(block['records']):
    ix=offset+b*5+j+1
    if not math.isfinite(r['end_time']) or abs(r['end_time']-(online['time_s']+(b*5+j+1)*.02))>1e-7 or not np.isfinite(r['end_world_xy']).all():raise ValueError('nonfinite or irregular supervision')
    if not np.array_equal(r['actual_raw_pair'],program[b*5+j]) or not np.array_equal(r['actual_raw_pair'],physics['raw_command_target'][ix]) or r['end_time']!=float(physics['simulation_time_s'][ix]) or not np.array_equal(r['end_world_xy'],physics['position_world_xy'][ix]):raise ValueError('raw program supervision mismatch')
  records.append(dict(scorable=True,program=program,estimated_velocity=online['estimated_velocity']))
 result=dict(group=group['group'],split=group['split'],planned=70,attempted=len(windows),scorable=len(records),execution_identity=[status['start_utc'],status['child_pid']],**derive_support(records))
 return result
