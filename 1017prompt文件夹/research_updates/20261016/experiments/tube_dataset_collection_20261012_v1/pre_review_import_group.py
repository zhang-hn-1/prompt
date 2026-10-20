"""Sealed local-run consistency; not external attestation or statistical independence."""
from pathlib import Path
import json,hashlib,math
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
 windows=json.loads((run/'windows.json').read_text());physics=np.load(run/'physics.npz',allow_pickle=False);records=[]
 if len(windows)>70 or len({w['window'] for w in windows})!=len(windows):raise ValueError('invalid attempted windows')
 for window in windows:
  if window['scorable'] is not True:continue
  i=window['window'];rows=[json.loads(line) for line in (run/f'window_{i:03d}.jsonl').read_text().splitlines()];commits=[r for r in rows if r['type']=='commit'];feedback=[r for r in rows if r['type']=='block_feedback']
  if len(commits)!=1 or len(feedback)!=5:raise ValueError('complete window lacks records')
  program=commits[0]['program']
  expected_program=[list(map(f32,TEMPLATES[group['window_templates'][i]]))]*25
  if program!=expected_program:raise ValueError('scheduled program mismatch')
  envelope=json.loads(bytes.fromhex(commits[0]['snapshot_hex']));online=envelope['online_state'][0];offset=i*25
  if len(program)!=25 or not math.isfinite(online['time_s']) or abs(online['time_s']-float(physics['simulation_time_s'][offset]))>1e-7:raise ValueError('anchor clock mismatch')
  for b,block in enumerate(feedback):
   if block['index']!=b or block['physical_contact'] or block['collision'] or len(block['records'])!=5:raise ValueError('invalid complete block')
   for j,r in enumerate(block['records']):
    ix=offset+b*5+j+1
    if not math.isfinite(r['end_time']) or abs(r['end_time']-(online['time_s']+(b*5+j+1)*.02))>1e-7 or not np.isfinite(r['end_world_xy']).all():raise ValueError('nonfinite or irregular supervision')
    if not np.array_equal(r['actual_raw_pair'],program[b*5+j]) or not np.array_equal(r['actual_raw_pair'],physics['raw_command_target'][ix]) or r['end_time']!=float(physics['simulation_time_s'][ix]) or not np.array_equal(r['end_world_xy'],physics['position_world_xy'][ix]):raise ValueError('raw program supervision mismatch')
  records.append(dict(scorable=True,program=program,estimated_velocity=online['estimated_velocity']))
 result=dict(group=group['group'],split=group['split'],planned=70,attempted=len(windows),scorable=len(records),execution_identity=[str(run),status['start_utc'],status['child_pid']],**derive_support(records))
 return result
