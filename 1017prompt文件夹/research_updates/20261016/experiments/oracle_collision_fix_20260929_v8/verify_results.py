import json,hashlib
from pathlib import Path
import numpy as np
h=Path(__file__).resolve().parent
r=json.loads((h/'results.json').read_text())
assert len(r)==5
assert all(x['returncode']==0 for x in json.loads((h/'status.json').read_text()))
assert all(json.loads((h/'identity_after.json').read_text()).values())
for case,x in r.items():
 assert 'old_path_missing' not in x
 assert x['unavailable_samples']==0
 for name,p in x['prefix_parity'].items():
  for k,v in p.items():
   if k not in ['collision','raw_simulation_time_s','raw_physics_step_index']:assert v['exact'],(case,name,k)
 if case!='pid_0002_legacy':
  assert x['metrics']['evaluation_protocol_id']=='ball_pointgoal_v1_2_contact_35s'
  assert x['samples']==x['metrics']['action_blocks']*6+1
  episode=next((h/case).glob('episode_*'));z=np.load(episode/'states.npz')
  assert np.array_equal(z['collision'],z['physical_contact'])
assert r['pid_0002_contact']['metrics']['termination_reason']=='timeout'
assert r['pid_0002_contact']['metrics']['action_blocks']==350
assert r['pid_0002_contact']['metrics']['map_nonpositive_clearance_samples']==38
assert r['pid_0002_contact']['first_physical_contact'] is None
assert r['course_0004_contact']['metrics']['termination_reason']=='collision'
assert r['course_0004_contact']['first_physical_contact']['physics_step_index']==678
assert r['course_0004_contact']['metrics']['map_nonpositive_clearance_samples']==0
for case in ['golden_e0_contact','golden_e1_contact']:assert r[case]['metrics']['termination_reason']=='success'
assert r['pid_0002_legacy']['metrics']['termination_reason']=='collision'
assert r['pid_0002_legacy']['metrics']['action_blocks']==157
(h/'verification.json').write_text(json.dumps(dict(passed=True,actual_runs=5,source_identity_unchanged=True,physical_action_state_prefix_exact_excluding_absolute_clock=True),indent=2))
print('PASS: five actual runs, unchanged control/state prefixes, legacy reproduction, false-positive and false-negative regression.')
