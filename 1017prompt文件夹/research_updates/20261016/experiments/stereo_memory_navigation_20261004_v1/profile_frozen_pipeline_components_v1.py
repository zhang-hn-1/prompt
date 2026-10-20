"""Exactly three predeclared recorded epochs, no simulation/action/GT labels."""
from pathlib import Path
import json,hashlib,time,datetime,traceback
import numpy as np,torch
from fast_stereo import FastStereo
from paired_texture_surface import PairedTextureSurfaceMemory
from stereo_geometry import restore_native
from masked_source_pair import paired_sources
from dynamic_self_mask import RobotSelfMask
H=Path(__file__).resolve().parent;D=H/'live_paired_texture_navigation_run_v1';O=H/'frozen_pipeline_component_timing_v1';config=json.loads((H/'frozen_pipeline_timing_protocol_v1.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for g in ('source_sha256','input_sha256'):
 for f,s in config[g].items():assert sha(f)==s,f
assert config['frames']==[0,150,295] and config['maximum_calls']==3
O.mkdir(exist_ok=False);rows=[];start=time.monotonic();success=False
def save():
 (O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n')
try:
 torch.set_num_threads(1);begin=time.monotonic();model=FastStereo();torch.cuda.synchronize();initialization=time.monotonic()-begin;begin=time.monotonic();cad=RobotSelfMask();cad_init=time.monotonic()-begin;static=np.load(config['static_masks'])
 for i in config['frames']:
  name=f'frame_{i:03d}.npz';rgb=np.load(D/'source_inputs'/name);original=np.load(D/'source_predictions'/name);pose=np.load(D/'estimated_poses'/name);history=np.load(D/'history'/name)['proprio_history'].copy();history[:,:9]=0.;packet=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');t=float(packet['time_s']);torch.cuda.synchronize();begin=time.monotonic();pred=model.predict(rgb['left_rgb'],rgb['right_rgb'],static['left'],static['right']);torch.cuda.synchronize();nn_wall=time.monotonic()-begin
  np.savez_compressed(O/name,**pred,frame=i,time_s=t)
  a=pred['disparity_px'];b=original['disparity_px'];finite=np.isfinite(a)&np.isfinite(b);dd=np.abs(a[finite]-b[finite]);begin=time.monotonic();_,rebuilt,_=paired_sources(dict(depth_m=original['depth_m'],confidence=original['confidence'],disparity_native_px=restore_native(original['disparity_px']),frame=i,time_s=t),dict(**{k:pose[k] for k in ('left_camera_usd_world','right_camera_usd_world','root_position_world_m')},time_s=t),history,cad,t);geometry_wall=time.monotonic()-begin
  for k in packet.files:np.testing.assert_array_equal(rebuilt[k],packet[k])
  provenance={str((D/'source_inputs'/name).resolve()):sha(D/'source_inputs'/name)};memory=PairedTextureSurfaceMemory();begin=time.monotonic();surface=memory.update(packet,rgb['left_rgb'],rgb['right_rgb'],t,provenance,restore_native(original['disparity_px']));texture_wall=time.monotonic()-begin;saved=np.load(D/'partial_observations'/name)
  for k in ('world','sigma_m','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(surface[k],saved[k])
  rows.append(dict(frame=i,time_s=t,NN_predict_wall_s=nn_wall,first_call_includes_warmup=i==0,explicit_first_warmup_s=model.warmup_s if i==0 else None,NN_disparity_exact=bool(np.array_equal(a,b)),NN_disparity_abs_p50_p90_max_px=None if not len(dd) else np.percentile(dd,[50,90,100]).tolist(),NN_confidence_exact=bool(np.array_equal(pred['confidence'],original['confidence'])),original_NN_geometry_CAD_wall_s=geometry_wall,geometry_all_fields_exact=True,paired_texture_wall_s=texture_wall,paired_texture_four_fields_exact=True,points=len(surface['world']),not_online=True));save()
 success=True
except BaseException:
 (O/'failure.txt').write_text(traceback.format_exc());raise
finally:
 save();(O/'status.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),complete=success,actual_profiled_epochs=len(rows),total_wall_s=time.monotonic()-start,model_initialization_wall_s=locals().get('initialization'),CAD_initialization_wall_s=locals().get('cad_init'),no_future_or_GT_reads=True,no_simulation_or_control=True,no_parameters_or_weights_changed=True,not_realtime_certificate=True,not_another_navigation_run=True,whole_complete=False),indent=2)+'\n');(O/'artifact_sha256.json').write_text(json.dumps({str(f.resolve()):sha(f) for f in O.rglob('*') if f.is_file()},indent=2)+'\n')
print(json.dumps(rows,indent=2))
