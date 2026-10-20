"""102 exact current-state processor replays, no NN/GT/action calls."""
from pathlib import Path
import numpy as np,json,time,hashlib,datetime
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from static_base_cached_self_mask import StaticBaseCachedSelfMask
from masked_source_pair import paired_sources
from stereo_geometry import restore_native
from paired_texture_surface import PairedTextureSurfaceMemory
H=Path(__file__).resolve().parent;D=H/'live_paired_texture_navigation_run_v1';O=H/'static_base_cache_whole_sequence_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();manifest=D/'artifact_manifest_post_close.json'
for f,s in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==s,f
sources=[Path(__file__).resolve(),H/'static_base_cached_self_mask.py',H/'dynamic_self_mask.py',H/'masked_source_pair.py',H/'paired_texture_surface.py'];(O/'prevalidation_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256={str(f):sha(f) for f in sources},input_manifest_sha256=sha(manifest),maximum_epochs=102,no_GT_NN_or_control=True),indent=2)+'\n')
begin=time.monotonic();old=RobotSelfMask();original_init=time.monotonic()-begin;begin=time.monotonic();cached=StaticBaseCachedSelfMask();cache_init=time.monotonic()-begin;memory=PairedTextureSurfaceMemory();rows=[]
class TimedCAD:
 def __init__(self):self.frozen=cached.frozen
 def project(self,q):
  begin=time.monotonic();self.last=cached.project(q);self.wall=time.monotonic()-begin;return self.last
cad=TimedCAD()
for row in json.loads((D/'source_rows.json').read_text()):
 i=row['frame'];name=f'frame_{i:03d}.npz';hist=np.load(D/'history'/name)['proprio_history'].copy();hist[:,:9]=0.;q=joints_from_proprio(hist);begin=time.monotonic();a=old.project(q);old_wall=time.monotonic()-begin;p=np.load(D/'source_predictions'/name);pose=np.load(D/'estimated_poses'/name);t=row['time_s'];begin=time.monotonic();_,packet,_=paired_sources(dict(depth_m=p['depth_m'],confidence=p['confidence'],disparity_native_px=restore_native(p['disparity_px']),frame=i,time_s=t),dict(**{k:pose[k] for k in ('left_camera_usd_world','right_camera_usd_world','root_position_world_m')},time_s=t),hist,cad,t);source_wall=time.monotonic()-begin
 for eye in ('left','right'):np.testing.assert_array_equal(a[eye],cad.last[eye])
 raw=np.load(D/'sensor_packets'/f'source_{i:03d}.npz')
 for key in raw.files:np.testing.assert_array_equal(packet[key],raw[key])
 rgb=np.load(D/'source_inputs'/name);surface=memory.update(packet,rgb['left_rgb'],rgb['right_rgb'],t,row['provenance'],restore_native(p['disparity_px']));saved=np.load(D/'partial_observations'/name)
 for k in ('world','sigma_m','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(surface[k],saved[k])
 assert [r['acquisition_time_s'] for r in memory.at(t)]==row['memory_acquisition_times_s']
 rows.append(dict(frame=i,time_s=t,old_CAD_wall_s=old_wall,cached_CAD_wall_s=cad.wall,cached_paired_geometry_total_wall_s=source_wall,two_masks_exact=True,raw_packet_all_fields_exact=True,selected_point_four_fields_exact=True,history_birth_times_exact=True));(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n')
assert len(rows)==102
old_t=np.array([r['old_CAD_wall_s'] for r in rows]);new_t=np.array([r['cached_CAD_wall_s'] for r in rows]);result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_WHOLE_SEQUENCE_EXACT_CAD_PACKET_POINT_REPLAY_ONLY' if new_t.sum()<old_t.sum() else 'REJECT_NO_MEASURED_CAD_RUNTIME_SAVING',epochs=102,mask_fields_exact=204,selected_point_fields_exact=408,raw_packet_and_birth_time_exact=True,old_constructor_wall_s=original_init,cached_constructor_wall_s=cache_init,old_CAD_p50_p90_max_s=np.percentile(old_t,[50,90,100]).tolist(),cached_CAD_p50_p90_max_s=np.percentile(new_t,[50,90,100]).tolist(),total_old_CAD_s=float(old_t.sum()),total_cached_CAD_s=float(new_t.sum()),runtime_reduction_fraction=float(1-new_t.sum()/old_t.sum()),no_NN_GPU_GT_or_action_calls=True,not_fresh_online_or_realtime_or_nav_improvement=True,old_bytes_and_gates_preserved=True,whole_complete=False);(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');(O/'postvalidation_sha256.json').write_text(json.dumps({str(f.resolve()):sha(f) for f in O.rglob('*') if f.is_file()},indent=2)+'\n');print(json.dumps(result,indent=2))
