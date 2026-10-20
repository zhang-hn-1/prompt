from pathlib import Path
import numpy as np,pytest
from mvs_capture_causality import captured_request

class Trap(dict):
 def __getitem__(self,k):
  if k not in ['time_s','frame']:raise AssertionError('payload before complete clock validation')
  return super().__getitem__(k)

def loader_for(bad_kind=None,bad_frame=None):
 def load(p):
  frame=int(Path(p).stem.split('_')[-1]);kind=Path(p).parent.name;t=frame/10
  if kind==bad_kind and frame==bad_frame:t+=.1
  return Trap(time_s=t,frame=frame)
 return load

def test_current_RGB_mismatch_rejected_before_payload():
 with pytest.raises(ValueError):captured_request(Path('/capture'),20,'left',{20:2.,10:1.,0:0.},loader_for('online_inputs',20))

def test_future_source_pose_rejected_before_current_RGB_read():
 with pytest.raises(ValueError):captured_request(Path('/capture'),20,'left',{20:2.,10:1.,0:0.},loader_for('known_poses',0))

def test_CAD_timestamp_or_frame_identity_rejected_before_payload():
 with pytest.raises(ValueError):captured_request(Path('/capture'),20,'left',{20:2.,10:1.,0:0.},loader_for('masks',10))
 def bad(p):
  x=loader_for()(p)
  if Path(p).parent.name=='masks':x['frame']=99
  return x
 with pytest.raises(ValueError):captured_request(Path('/capture'),20,'left',{20:2.,10:1.,0:0.},bad)

def test_valid_actual_schema_native_capture_builds_independent_clock_request():
 def load(p):
  f=int(Path(p).stem.split('_')[-1]);return dict(time_s=f/10,frame=f,left_rgb=np.zeros((480,640,3),np.uint8),left_camera_usd_world=np.eye(4),left=np.zeros((480,640),bool))
 r=captured_request(Path('/capture'),20,'left',{20:2.,10:1.,0:0.},load)
 assert r['current_time_s']==2. and r['source_time_s'].tolist()==[1.,0.]
