from pathlib import Path
import json,hashlib

def require_complete(status,directory):
 d=Path(directory)
 if status.get('status')!='finished' or status.get('child_returncode')!=0:raise ValueError('Actual process not normally finished')
 if (d/'exception.txt').exists():raise ValueError('Recorded exception invalidates nominal exit zero')
 p=d/'capture_seal.json'
 if not p.exists():raise ValueError('No complete capture seal, exit zero insufficient')
 r=json.loads(p.read_text())
 if not r.get('complete') or len(r.get('rows',[]))!=8:raise ValueError('Incomplete bounded actual sequence')
 previous=[-1,-1]
 for row in r['rows']:
  f=d/'frames'/row['frame']
  if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Original capture identity mismatch')
  if not row['imu']['qualified'] or row['pair_error']>2e-5:raise ValueError('IMU/exposure pose contract failed')
  if any(x<=y for x,y in zip(row['render_frames'],previous)):raise ValueError('Reused exposures')
  previous=row['render_frames']
 return r
if __name__=='__main__':
 D=Path(__file__).resolve().parent;Q=D.parent/'wide_pinhole_camera_20261015_v1';checks=[]
 for name,status in [('native_run','status.json'),('native_recovery_run','recovery_status.json')]:
  try:require_complete(json.loads((Q/status).read_text()),Q/name)
  except ValueError as e:checks.append(dict(stage=name,rejected=True,reason=str(e)))
  else:raise AssertionError('An incomplete failed capture was accepted')
 (D/'completion_guard_check.json').write_text(json.dumps(dict(checks=checks,nominal_zero_rejected=True,scope='Existing raw failed attempts only; no actual SDK success proof.'),indent=2)+'\n');print(checks)
