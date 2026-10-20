"""Post-run author replay: no physics, inference or action changes."""
from pathlib import Path
import json,datetime,numpy as np
from prior_rgb_navigation import PriorRGBGrid,require_boolean_confidence
from dense_source_points import point_packet
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent
m=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');result={}
for arm in ('current',):
 O=H/'rgb_complete_goal_current_run';rows=json.loads((O/'policy_records.json').read_text());states=json.loads((O/'states.json').read_text());grid=None;shadowgrid=None;packets={};rebuilt=0;mapchecks=0;shadow=json.loads((O/'shadow_records.json').read_text());shadowchecks=0
 for f in sorted((O/'source_predictions').glob('*.npz')):
  d=np.load(f);i=int(d['frame']);pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz');rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');assert set(rgb.files)=={'left_rgb','right_rgb','time_s'};assert set(pose.files)=={'left_camera_usd_world','right_camera_usd_world','time_s'}
  p=point_packet(dict(depth_m=d['depth_m'],confidence=require_boolean_confidence(d['confidence']),disparity_native_px=restore_native(d['disparity_px']),frame=i,time_s=float(d['time_s'])),dict(left_camera_usd_world=pose['left_camera_usd_world'],right_camera_usd_world=pose['right_camera_usd_world'],root_position_world_m=states[i]['position'],time_s=float(pose['time_s'])),m['left'],m['right']);saved=np.load(O/'sensor_packets'/f'source_{i:03d}.npz')
  for k,v in p.items():np.testing.assert_equal(v,saved[k])
  packets[i]=p;rebuilt+=1
 for i,row in enumerate(rows):
  s=np.load(O/'maps'/f'frame_{i:03d}.npz');t=float(s['time_s']);root=states[i]['position']
  if grid is None:grid=PriorRGBGrid(s['origin'],shape=s['labels'].shape,resolution=float(s['resolution']),memory_mode=arm)
  grid.set_source(packets[row['local']['source_frame']]);grid.update_points([],root,t)
  if shadowgrid is None:shadowgrid=PriorRGBGrid(s['origin'],shape=s['labels'].shape,resolution=float(s['resolution']),memory_mode='history')
  shadowgrid.set_source(packets[row['local']['source_frame']]);shadowgrid.update_points([],root,t)
  for k,v in shadowgrid.diagnostics().items():assert v==shadow[i]['local'][k],(i,k,v,shadow[i]['local'][k]);shadowchecks+=1
  for k,v in dict(labels=grid.labels,provenance=grid.provenance(),floor_time=grid.memory.floor_time,hazard_time=grid.memory.hazard_time,task_ground_prior=grid.task_ground_prior,safe_start_prior=grid.safe_start_prior).items():np.testing.assert_equal(v,s[k]);mapchecks+=1
 class Bomb(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload accessed')
   return super().__getitem__(k)
 grid.set_source(Bomb(time_s=grid.time+1))
 try:grid.update_points([],states[-1]['position'],grid.time)
 except ValueError:pass
 else:raise AssertionError('future source admitted')
 result[arm]=dict(exact_source_packets=rebuilt,exact_map_frames=len(rows),exact_map_arrays=mapchecks,exact_shadow_diagnostic_fields=shadowchecks,future_packet_payload_trap_passed=True)
(H/'rgb_complete_goal_author_replay.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=result,scope='offline exact source projection/primary maps and same-stream shadow diagnostic reconstruction; no fresh navigation or inference'),indent=2)+'\n');print(json.dumps(result,indent=2))
