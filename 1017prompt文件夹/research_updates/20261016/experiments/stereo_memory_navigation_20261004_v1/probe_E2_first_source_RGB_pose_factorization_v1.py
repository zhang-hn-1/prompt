"""Four-case historical diagnostic at .5s: cached stereo output and pose separated.
No inference, no teacher depth, no actor, no future feedback or parameter changes.
"""
from pathlib import Path
import json,hashlib,numpy as np
from dense_source_points import point_packet
from paired_texture_surface import PairedTextureSurfaceMemory
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;cfg=json.loads((H/'E2_first_source_RGB_pose_factorization_protocol_v1.json').read_text());O=H/cfg['output'];O.mkdir(exist_ok=False)
Ds={name:H/path for name,path in cfg['streams'].items()};inputs={}
for D in Ds.values():
 manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in manifest['artifact_sha256'].items())
A,B=Ds['knownpose_stream'],Ds['native_stream'];pa=np.load(A/'physics.npz');pb=np.load(B/'physics.npz');i=5
assert all(np.array_equal(pa[k][:301],pb[k][:301]) for k in pa.files if not k.startswith('raw_') and k not in ('action_block_index','physics_step_in_block'))
cad=TimedStaticBaseSelfMask();m=cad.project(pa['joint_position_rad'][i*5]);rows=[]
for image_name,D in Ds.items():
 f='frame_005.npz';rgb=np.load(D/'source_inputs'/f);pred=np.load(D/'source_predictions'/f);t=float(rgb['time_s']);disparity=restore_native(pred['disparity_px'])
 for pose_name,PD in Ds.items():
  poses=np.load(PD/'known_poses'/f);assert abs(float(poses['time_s'])-t)<1e-8
  prediction=dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=disparity,frame=i,time_s=t)
  pose={k:poses[k] for k in poses.files};pose['time_s']=t
  packet=point_packet(prediction,pose,m['left'],m['right'])
  for p in (D/'source_inputs'/f,D/'source_predictions'/f,PD/'known_poses'/f):inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
  memory=PairedTextureSurfaceMemory();surface=memory.update(packet,rgb['left_rgb'],rgb['right_rgb'],t,inputs.copy(),disparity)
  out=O/(image_name+'__'+pose_name+'.npz');np.savez_compressed(out,**{k:v for k,v in surface.items() if k!='input_hashes'})
  cells=sorted(set(map(tuple,np.rint(surface['world'][:,:2]/.1).astype(int))))
  row=dict(image_stream=image_name,pose_stream=pose_name,packet_points=len(packet['world']),selected_surface_points=len(surface['world']),selected_risk_cells=cells,coordinate_cells_diagnostic_not_actual_actor_replan=True);rows.append(row)
  if image_name==pose_name:
   original=np.load(D/'partial_observations'/f)
   for k in ('world','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(surface[k],original[k])
  print(row,flush=True)
# This reconstructs only the earliest source; whole trajectory cause is not established.
r=dict(protocol=cfg,rows=rows,inputs_sha256=inputs,actual_diagonal_exact=True,causal_future_inputs_read=False,teacher_depth_read=False,new_inference=False,new_physics=False,not_online_validation=True,not_navigation_success=True,old_source_rejections_preserved=True)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.rglob('*') if p.is_file()}),indent=2)+'\n')
