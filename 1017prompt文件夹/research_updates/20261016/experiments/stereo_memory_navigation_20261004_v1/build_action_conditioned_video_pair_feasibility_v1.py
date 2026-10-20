"""Historical fit-data feasibility, not a trained model or fresh validation.
Causal features are sealed before future image-flow labels are computed.
"""
from pathlib import Path
import json,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent;cfg=json.loads((H/'action_conditioned_video_pair_feasibility_protocol_v1.json').read_text());O=H/cfg['output'];O.mkdir(exist_ok=False);rows=[]
y,x=np.mgrid[12:480:24,12:640:24];points=np.column_stack([x.ravel(),y.ravel()]).astype(np.float32).reshape(-1,1,2)
for name in cfg['sequences']:
 D=H/name;manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in manifest['artifact_sha256'].items());req=np.load(D/'requests.npz')['requests'];native=json.loads((D/'native_imu_packets.json').read_text());states=json.loads((D/'states.json').read_text());lookup={round(float(a['time_s']),6):a for a in native if a is not None};cap=cv2.VideoCapture(str(D/'stereo_view.mp4'));images=[]
 while True:
  ok,img=cap.read()
  if not ok:break
  assert img.shape[:2]==(480,1280);images.append(cv2.cvtColor(img[:,:640],cv2.COLOR_BGR2GRAY))
 cap.release();assert len(images)==len(req)+1
 features=[];indices=[]
 for i in range(1,len(req)):
  t=float(states[i]['time_s']);imu=lookup[round(t,6)];joint=np.load(D/'physics.npz')['joint_position_rad'][i*5]
  # Original issued request at t is a known conditioning input; no future commands.
  feature=np.r_[imu['gyro_body_rad_s'],imu['linear_acceleration_body_m_s2'],imu['main_encoder_rad_s'],joint,req[i,:2],req[i-1,:2]]
  assert np.isfinite(feature).all();features.append(feature);indices.append(i)
 F=O/(name+'_inputs.npz');np.savez_compressed(F,features=np.asarray(features),current_capture_indices=np.asarray(indices),query_xy=points[:,0],current_times_s=np.asarray([states[i]['time_s'] for i in indices]))
 seal=hashlib.sha256(F.read_bytes()).hexdigest();(O/(name+'_input_seal_before_future_labels.json')).write_text(json.dumps(dict(sha256=seal,features='current native IMU encoder measured joints current issued request previous request only; no future feedback/applied/poses',future_frames_already_decoded_for_offline_fit_labeling_not_consumer_inputs=True,not_model_prediction_seal=True),indent=2)+'\n')
 targets=[];valids=[]
 for i in indices:
  forward,ok,error=cv2.calcOpticalFlowPyrLK(images[i],images[i+1],points,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01));back,bok,_=cv2.calcOpticalFlowPyrLK(images[i+1],images[i],forward,None,winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01));valid=ok.ravel().astype(bool)&bok.ravel().astype(bool)&np.isfinite(forward[:,0]).all(1)&(np.linalg.norm(back[:,0]-points[:,0],axis=1)<=1)&(error.ravel()<=20);targets.append(forward[:,0]-points[:,0]);valids.append(valid)
 np.savez_compressed(O/(name+'_future_flow_labels.npz'),flow=np.asarray(targets),usable_LK=np.asarray(valids),teacher='offline next recorded image LK; approximate correspondence not true 3D motion or occupied/free label')
 assert hashlib.sha256(F.read_bytes()).hexdigest()==seal
 valid=np.asarray(valids);motion=np.asarray([np.linalg.norm(req[i,:2])>1e-8 for i in indices]);row=dict(sequence=name,pairs=len(indices),grid_queries=len(points),usable_flow_labels=int(valid.sum()),usable_motion_labels=int(valid[motion].sum()),features_sha256=seal,whole_sequence_historical_used_for_design_not_fresh=True,compressed_video_input_not_raw_RGB=True,not_true_3D_or_safety_label=True);rows.append(row);print(row,flush=True)
(O/'assessment.json').write_text(json.dumps(dict(protocol=cfg,rows=rows,no_fit=True,no_predictions_or_admission=True,no_controller_or_network_change=True,new_hypothesis='next-frame visual motion conditioned on current issued action, distinct from static MODEL reliability and not M3 physical response fit',needed_before_training='freeze one model/control horizon metrics and whole fresh heldout; cannot train/test adjacent-frame random split or use future true feedback as input'),indent=2)+'\n');(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.rglob('*') if p.is_file()}),indent=2)+'\n')
