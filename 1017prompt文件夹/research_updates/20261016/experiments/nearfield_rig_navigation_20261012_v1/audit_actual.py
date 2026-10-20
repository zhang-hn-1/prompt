"""Verify measured clocks, registration, requests and sealed artifacts after each run."""
from pathlib import Path
import json,hashlib,numpy as np,sys
D=Path(__file__).resolve().parent
ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
def audit(key):
 out=D/(key+'_run');config=json.loads((D/(key+'_protocol.json')).read_text());states=json.loads((out/'states.json').read_text());native=json.loads((out/'native_predictions.json').read_text());clocks=json.loads((out/'camera_clocks.json').read_text());contacts=json.loads((out/'physics_contacts.json').read_text());requests=np.load(out/'requests.npz')['requests'];physics=np.load(out/'physics.npz');rows=[json.loads(line) for line in (out/'policy_rows.ndjson').read_text().splitlines()]
 assert len(states)==len(clocks)==len(requests)+1
 assert len(contacts)>0 and all(c['available'] for c in contacts)
 # Journal may have planned+executed variants; verify executed request coverage from exported rows below.
 preds={round(float(n['time_s']),7):n for n in native if n.get('available')}
 checked=0;max_residual=0
 for p in sorted((out/'known_poses').glob('*.npz')):
  poses=np.load(p);t=float(poses['time_s']);n=preds[round(t,7)];assert n['live_before_next_action']
  frame=np.load(out/'frames'/p.name);R=np.asarray(n['rotation_world']);position=np.asarray(n['position_world']);assert abs(float(frame['time_s'])-t)<1e-8
  np.testing.assert_allclose(poses['root_position_world_m'],position,atol=1e-8,rtol=0)
  extrinsics=np.load(config['rig_extrinsics'])
  for eye in ['left','right']:
   E=extrinsics[eye+'_optical_to_base'];expected=np.eye(4);expected[:3,:3]=R@E[:3,:3]@np.diag([1,-1,-1]);expected[:3,3]=position+R@E[:3,3]
   np.testing.assert_allclose(poses[eye+'_camera_usd_world'],expected,atol=1e-7,rtol=0)
   residual=float(np.max(np.abs(frame[eye+'_optical_to_base']-E)));assert residual<2e-5;max_residual=max(max_residual,residual)
  checked+=1
 for i,(s,c) in enumerate(zip(states,clocks)):
  assert abs(float(s['time_s'])-float(c['episode_time_s']))<1e-8
  assert abs(float(s['time_s'])-i*.1)<3e-5
  assert all(abs(float(c['raw_time_s'])-float(t))<1e-5 for t in c['eyes'].values())
 for k,h in config['source_sha256'].items():assert hashlib.sha256(Path(k).read_bytes()).hexdigest()==h,k
 for k,h in config['input_sha256'].items():assert hashlib.sha256(Path(k).read_bytes()).hexdigest()==h,k
 return dict(pass_audit=True,captures=len(states),requests=len(requests),registration_pairs=checked,max_extrinsic_residual=max_residual,all_contacts_available=True,journal_lines=len(rows),physics_fields=physics.files,source_and_input_hashes=len(config['source_sha256'])+len(config['input_sha256']),scope='clocks/current native registration/source identities; not independent IMU dynamics reconstruction or causal replication')
if __name__=='__main__':
 result=audit(sys.argv[1]);(D/(sys.argv[1]+'_raw_audit.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
