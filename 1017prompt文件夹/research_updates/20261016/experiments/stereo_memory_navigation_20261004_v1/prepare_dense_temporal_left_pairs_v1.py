"""Two causal current+past RGB virtual pairs; no GT depth/instance reads or actor changes."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
from temporal_dense_rectification_v1 import rectification,normalize_image,K
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'dense_temporal_left_development_v1';O.mkdir(exist_ok=False)
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),queries=[215,330],maximum_queries=2,past_candidates=4,max_age_s=2.,minimum_baseline_m=.1,selection='largest translation perpendicular to current optical axis, then total baseline, then latest; only given current/past poses; no depth/GTinstance or NNscore selection',rectification='OpenCV stereoRectify alpha1 CALIB_ZERO_DISPARITY; no alpha/ROI sweep; calibrated current/past native sameLEFT',network='same frozen FastFoundationStereo checkpoint, iters8 maxdisp416 LR1.5; no swap/train',source_gates='original >=100 /95%<=10cm/p90<=10cm and target>=100 twoepochs; developmental selection notfresh',source_role='current knownGT pose diagnostic only; separate temporal sourcehypothesis; not observationfree',old_ORB_and_source_REJECT_preserved=True,no_actual_physics=True,no_actor_install=True,no_free_volume=True)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');cad=TimedStaticBaseSelfMask();cache={};inputs={}
def payload(i):
 if i not in cache:
  paths=[D/n/f'frame_{i:03d}.npz' for n in ['source_inputs','known_poses','history']]
  for p in paths:inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
  rgb=np.load(paths[0]);pose=np.load(paths[1]);hist=np.load(paths[2])['proprio_history'].copy();hist[:,:9]=0;mask=cad.project(joints_from_proprio(hist))['left']
  cache[i]=dict(rgb=rgb['left_rgb'],pose=pose['left_camera_usd_world'],root=pose['root_position_world_m'],time_s=float(pose['time_s']),mask=mask)
 return cache[i]
rows=[]
for current in protocol['queries']:
 a=payload(current);candidates=[];axis=a['pose'][:3,:3]@np.array([0.,0.,-1.])
 for past in range(current-20,current,5):
  b=payload(past);assert 0<a['time_s']-b['time_s']<=2.+1e-6
  delta=a['pose'][:3,3]-b['pose'][:3,3];total=np.linalg.norm(delta);perp=np.linalg.norm(delta-axis*(delta@axis))
  candidates.append(dict(frame=past,baseline_m=float(total),perpendicular_m=float(perp)))
 eligible=[x for x in candidates if x['baseline_m']>=.1]
 if not eligible:rows.append(dict(current_frame=current,decision='REJECT_NO_BASELINE_IN_FROZEN_CAUSAL_WINDOW',candidates=candidates));continue
 picked=max(eligible,key=lambda x:(x['perpendicular_m'],x['baseline_m'],x['frame']));past=picked['frame'];b=payload(past);r=rectification(a['pose'],b['pose']);images=[];bad=[];maps=[]
 for i,p in enumerate([a,b]):
  mx,my=cv2.initUndistortRectifyMap(K,None,r['R'][i],r['P'][i][:,:3],r['size'],cv2.CV_32FC1);valid=(mx>=0)&(mx<=639)&(my>=0)&(my<=479)
  rgb=cv2.remap(p['rgb'],mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT);mask=cv2.remap(p['mask'].astype(np.uint8),mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=1).astype(bool)|~valid
  images.append(normalize_image(r,rgb));bad.append(normalize_image(r,mask));maps.append((normalize_image(r,mx),normalize_image(r,my)))
 common=float((~bad[0]&~bad[1]).mean());name=f'pair_{current:03d}_{past:03d}.npz'
 np.savez_compressed(O/name,query_rgb=images[0],past_rgb=images[1],query_mask=bad[0],past_mask=bad[1],query_native_x=maps[0][0],query_native_y=maps[0][1],past_native_x=maps[1][0],past_native_y=maps[1][1],current_pose=a['pose'],past_pose=b['pose'],current_root=a['root'],R1=r['R'][0],R2=r['R'][1],P1=r['P'][0],P2=r['P'][1],Q=r['Q'],vertical=r['vertical'],mirrored=r['mirrored'],current_time_s=a['time_s'],past_time_s=b['time_s'],raw_query_rgb=a['rgb'],raw_past_rgb=b['rgb'])
 rows.append(dict(current_frame=current,past_frame=past,picked=picked,candidates=candidates,pair_file=name,normalized_shape=list(images[0].shape),same_pixel_common_support_fraction=common,vertical=r['vertical'],mirrored=r['mirrored'],decision='READY_FOR_ONE_FROZEN_NN_QUERY' if common>0 else 'REJECT_NO_RECTIFIED_VALID_SUPPORT',knownpose_only=True,no_GT_depth_read=True))
source_files=[Path(__file__),H/'temporal_dense_rectification_v1.py',H/'fast_stereo.py',H/'feature_tracks.py',H/'stereo_geometry.py',H/'audit_generic_height_binocular_support_v1.py']
(O/'pair_identity.json').write_text(json.dumps(dict(inputs=inputs,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files},pairs=rows,pair_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.glob('pair_*.npz')},no_Glabels_before_future_prediction_seal=True),indent=2)+'\n');print(json.dumps(rows,indent=2))
