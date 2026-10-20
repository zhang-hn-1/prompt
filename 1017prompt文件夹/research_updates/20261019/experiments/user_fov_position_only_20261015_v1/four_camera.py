"""Four ORIGINAL nominal mounts; rear geometry from user's symmetry, not measured hardware calibration."""
from pathlib import Path
import sys,json,hashlib,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'camera_exposure_contract_20261015_v1'))
from exposure_contract import local_transform,snapshot
NAMES=('front_left','front_right','rear_left','rear_right')
def nominal_transforms(x,y,z):
 return [local_transform([x,y,z],0),local_transform([x,-y,z],0),local_transform([-x,y,z],180),local_transform([-x,-y,z],180)]
def validate_packets(packets,previous,time,transforms):
 if len(packets)!=4 or len(previous)!=4 or len(transforms)!=4:raise ValueError('Exactly four named cameras required')
 worst=0.
 for i,p in enumerate(packets):
  if p['render_frame']<=previous[i]:raise ValueError('Stale exposure')
  if abs(p['render_time']-time)>1e-5:raise ValueError('Exposure clock skew')
  if p['rgb'].shape!=(480,640,3) or p['depth_label'].shape!=(480,640):raise ValueError('Unexpected native shapes')
  for j in range(i):
   actual=np.linalg.inv(packets[j]['camera_usd_world_label'])@p['camera_usd_world_label'];expected=np.linalg.inv(transforms[j])@transforms[i];err=float(np.max(abs(actual-expected)));worst=max(err,worst)
 if worst>2e-5:raise ValueError(f'Four camera relative calibration differs: {worst}')
 return worst
def capture(cameras,previous,time,focal,transforms):
 packets=[snapshot(c,p,time,focal) for c,p in zip(cameras,previous)]
 err=validate_packets(packets,previous,time,transforms)
 # Ensure none of the four cameras advanced during serial copies.
 for c,p in zip(cameras,packets):
  current=c.get_current_frame()
  if int(current['rendering_frame'])!=p['render_frame'] or float(current['rendering_time'])!=p['render_time']:raise ValueError('Exposure changed during four-copy transaction')
 return packets,err

def validate_complete(status,d):
 d=Path(d)
 if status.get('status')!='finished' or status.get('child_returncode')!=0 or (d/'exception.txt').exists():raise ValueError('Process did not complete cleanly')
 if not (d/'capture_seal.json').exists():raise ValueError('No four-camera capture seal')
 for required in ['terminal.json','physics.npz','stage_identity.json','stage_root_initial.usda']:
  if not (d/required).is_file():raise ValueError('Missing final evidence: '+required)
 terminal=json.loads((d/'terminal.json').read_text())
 if terminal.get('exit_code')!=0 or terminal.get('frames')!=8:raise ValueError('Final terminal differs')
 with np.load(d/'physics.npz') as physics:
  ts=physics['raw_simulation_time_s']
  if len(ts)<201 or not np.all(np.diff(ts)>0):raise ValueError('Incomplete native physics')
 if not json.loads((d/'stage_identity.json').read_text()).get('used_layers'):raise ValueError('Missing stage identity')
 r=json.loads((d/'capture_seal.json').read_text());rows=r.get('rows',[])
 if not r.get('complete') or r.get('camera_names')!=list(NAMES) or len(rows)!=8:raise ValueError('Incomplete four-camera capture set')
 previous=[-1]*4
 for row in rows:
  f=d/'frames'/row['frame']
  if hashlib.sha256(f.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('Frame identity differs')
  if not row['imu']['qualified'] or row['pair_error']>2e-5:raise ValueError('IMU or calibration invalid')
  if len(row['render_frames'])!=4 or any(x<=y for x,y in zip(row['render_frames'],previous)):raise ValueError('Incomplete or reused exposures')
  with np.load(f) as a:
   for n in NAMES:
    if a[n+'_rgb'].shape!=(480,640,3) or a[n+'_depth_label'].shape!=(480,640) or a[n+'_camera_usd_world_label'].shape!=(4,4):raise ValueError('Missing actual image/calibration')
  previous=row['render_frames']
 return r
