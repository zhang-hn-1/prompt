from pathlib import Path
import json, hashlib, datetime
import numpy as np
import cv2
from partial_surface_memory import PartialSurfaceMemory
from audit_stereo import summarize
from stereo_geometry import FOCAL, restore_native
from masked_source_pair import paired_sources
from dynamic_self_mask import RobotSelfMask
H=Path(__file__).resolve().parent;D=H/'native_fixed_appearance_probe_run_v1';O=H/'native_fixed_appearance_probe_source_assessment_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
protocol=json.loads((H/'native_fixed_appearance_probe_protocol_v1.json').read_text());manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for group in (protocol['source_sha256'],protocol['input_sha256'],manifest):
 for f,s in group.items():assert sha(f)==s,f
model=np.load(protocol['appearance_model']);extrinsics=np.load(protocol['rig_extrinsics']);cad=RobotSelfMask();native=json.loads((D/'native_predictions.json').read_text());memory=PartialSurfaceMemory();rows=json.loads((D/'source_rows.json').read_text());assert len(rows)==29
# Rebuild current classifications and causal memory WITHOUT truth.
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';packet=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');rgb=np.load(D/'source_inputs'/name);classes=np.load(D/'partial_class_masks'/name)
 for file,digest in row['provenance'].items():assert sha(file)==digest
 poses=np.load(D/'estimated_poses'/name);n=native[i*5];assert n['available'] and n['time_s']==float(packet['time_s']);T=np.eye(4);T[:3,:3]=n['rotation_world'];T[:3,3]=n['position_world']
 for eye in ('left','right'):
  optical=T@extrinsics[eye+'_optical_to_base'];optical[:3,:3]=optical[:3,:3]@np.diag([1,-1,-1]);np.testing.assert_array_equal(optical,poses[eye+'_camera_usd_world'])
 pred=np.load(D/'source_predictions'/name);history=np.load(D/'history'/name)['proprio_history'].copy();history[:,:9]=0.
 _,rebuilt,masks=paired_sources(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=float(packet['time_s'])),dict(**{k:poses[k] for k in ('left_camera_usd_world','right_camera_usd_world')},root_position_world_m=T[:3,3],time_s=float(packet['time_s'])),history,cad,float(packet['time_s']))
 for key in packet.files:np.testing.assert_array_equal(rebuilt[key],packet[key])
 for eye in ('left','right'):
  lab=cv2.cvtColor(rgb[eye+'_rgb'],cv2.COLOR_RGB2LAB).astype(float);raw=np.linalg.norm((lab-model['centre'])/model['scale'],axis=-1)<=3
  np.testing.assert_array_equal(classes[eye],raw&~masks[eye])
 surface=memory.update(packet,classes['left'],classes['right'],float(packet['time_s']),row['provenance']);saved=np.load(D/'partial_observations'/name)
 for key in ('world','sigma_m','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(surface[key],saved[key])
 assert row['memory_acquisition_times_s']==[r['acquisition_time_s'] for r in memory.at(float(packet['time_s']))]
 assert row['points']==len(surface['world']) and row['truth_read_after_output']
 assert not saved['certified_free'] and not saved['unseen_volume_observed']
(O/'reconstruction_pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),records=len(rows),point_fields_exact=4*len(rows),not_free=True,appearance_frozen_sha256=sha(Path(protocol['appearance_model'])),code_sha256=sha(__file__)),indent=2)+'\n')
errors=[];classes_truth=[];scores=[]
for row in rows:
 i=row['frame'];saved=np.load(D/'partial_observations'/f'frame_{i:03d}.npz');truth=np.load(D/'frames'/f'frame_{i:03d}.npz');y,x=np.divmod(saved['query_pixel_id'],640);z=truth['left_depth_m'][y,x];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))));T=truth['left_camera_usd_world'];w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(axis=1);e=np.linalg.norm(saved['world']-w,axis=1);e[~valid]=np.inf
 occupied=valid&(np.linalg.norm(w[:,:2],axis=1)<=.351)&(w[:,2]>.001)&(w[:,2]<=1.001)
 errors.append(e);classes_truth.append(occupied);scores.append(dict(frame=i,n=len(e),metric=summarize(e),obstacle_precision=float(occupied.mean()) if len(e) else None))
e=np.concatenate(errors);c=np.concatenate(classes_truth);m=summarize(e);precision=float(c.mean()) if len(c) else 0.
checks=dict(n=len(e)>=100,metric95=m['within_0p10m'] is not None and m['within_0p10m']>=.95,p90=m['finite_p90_m'] is not None and m['finite_p90_m']<=.1,class95=precision>=.95)
pose=json.loads((H/'native_fixed_appearance_probe_pose_assessment_v1/assessment.json').read_text());checks['native_pose']=all(pose['gates'].values());checks['two100point_exposures']=sum(r['points']>=100 for r in rows)>=2
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_WHOLE_SEQUENCE_PARTIAL_SURFACE_INTERFACE_ONLY' if all(checks.values()) else 'REJECT_WHOLE_SEQUENCE_PARTIAL_SURFACE_SOURCE',checks=checks,world_metric=m,obstacle_precision=precision,frames=scores,nonempty_exposures=sum(r['points']>0 for r in rows),fresh_whole_sequence_for_frozen_partial_source=True,appearance_domain_separate_from_old_uniform=True,not_uniform_domain_admission=True,known_fixture_not_generalization=True,no_actor_consumption=True,not_navigation=True,not_original35=True,not_free_or_blind_volume_certificate=True,old_dense_reject_preserved=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='frames'},indent=2))
