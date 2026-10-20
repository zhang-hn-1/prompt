"""Direct frozen pose bridge adapter; metadata consistency is not camera attestation."""
import sys,math,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'stereo_memory_navigation_20261004_v1'))
from flat_ground_pose_bridge import FlatGroundPoseBridge

def provide(receiver,pose_bridge,*,time_s,raw_time_s,left_render_time_s,right_render_time_s,left_rgb,right_rgb):
 if type(pose_bridge) is not FlatGroundPoseBridge:raise ValueError('existing frozen flat-ground pose bridge required')
 if not all(math.isfinite(t) for t in (time_s,raw_time_s,left_render_time_s,right_render_time_s)) or any(abs(t-raw_time_s)>1e-5 for t in (left_render_time_s,right_render_time_s)):raise ValueError('synchronized raw camera metadata required')
 images=[]
 for value in (left_rgb,right_rgb):
  if not isinstance(value,np.ndarray) or value.shape!=(480,640,3) or value.dtype!=np.uint8:raise ValueError('fixed640x480 uint8 RGB required')
  images.append(np.ascontiguousarray(value).tobytes())
 state=pose_bridge.at(time_s)
 if not state['pose_estimated'] or not state['initial_gauge_privileged'] or not state['flat_ground_height_prior'] or state['height_prior_is_observation']:raise ValueError('explicit estimated state and privileges required')
 receiver.anchor(time_s,left_rgb=images[0],right_rgb=images[1],estimated_xy=tuple(state['root_position_world_m'][:2]),estimated_velocity=tuple(state['velocity_world_m_s'][:2]),estimated_heading=state['yaw_world_rad'],source='native_imu_rolling',initial_gauge=True,flat_ground_prior=True)
 return dict(time_s=float(time_s),raw_time_s=float(raw_time_s),left_render_time_s=float(left_render_time_s),right_render_time_s=float(right_render_time_s),image_shape=[480,640,3],dtype='uint8',left_rgb_sha256=hashlib.sha256(images[0]).hexdigest(),right_rgb_sha256=hashlib.sha256(images[1]).hexdigest(),pose_source='FlatGroundPoseBridge.at',initial_gauge_privileged=True,flat_ground_prior=True,metadata_only_camera_sync=True,live_camera_attested=False)

def provide_clocked(receiver,pose_bridge,runtime,read_rgb):
 """Use a single runtime clock pair; pixel freshness/reader implementation still unaudited."""
 before=tuple(runtime._clock_metadata())
 if len(before)!=4:raise ValueError('original runtime clock tuple required')
 left=runtime._left_camera;right=runtime._right_camera
 # Copy before subsequent reader calls to avoid reusable camera buffer aliasing.
 left_rgb=np.asarray(read_rgb(left)).copy();left_time=float(left.get_current_frame()['rendering_time'])
 right_rgb=np.asarray(read_rgb(right)).copy();right_time=float(right.get_current_frame()['rendering_time'])
 if tuple(runtime._clock_metadata())!=before:raise ValueError('runtime advanced during source capture')
 receipt=provide(receiver,pose_bridge,time_s=float(before[0]),raw_time_s=float(before[2]),left_render_time_s=left_time,right_render_time_s=right_time,left_rgb=left_rgb,right_rgb=right_rgb)
 receipt['clock_metadata']=list(before);receipt['same_runtime_epoch_bound']=True;receipt['pixel_freshness_attested']=False
 if hasattr(receiver,'journal'):
  try:receiver.journal.append(dict(type='anchor_source_receipt',receipt=receipt))
  except BaseException:receiver.failed=True;receiver.journal.close();raise
 return receipt
