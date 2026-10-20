"""Predictor reads ideal gyro/encoder + one gauge; rawRGB/depth/currentCAD only."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from rolling_shadow import OnlineRollingShadow
from rolling_pose_bridge import RollingPoseBridge
from dense_source_points import point_packet
from stereo_geometry import restore_native
from past_point_rgb_memory import PastPointRGBMemory
H=Path(__file__).resolve().parent;O=H/'online_RGB_memory_fresh_run_v1';D=H/'RGB_memory_frozen_pose_stress_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();protocol=json.loads((D/'protocol.json').read_text());assert all(sha(f)==s for f,s in protocol['source_sha256'].items());assert sha(D/'ideal_sensor_rows.npz')==protocol['sensor_sha256'];s=np.load(D/'ideal_sensor_rows.npz');assert set(s.files)=={'time_s','gyro_body_rad_s','main_encoder_rad_s','initial_normal','initial_position_world_m','initial_rotation_world'}
for name in ['estimated_poses','source_packets','memory_predictions']:(D/name).mkdir(exist_ok=False)
b=RollingPoseBridge(s['initial_position_world_m'],s['initial_rotation_world']);rolling=OnlineRollingShadow();memory=PastPointRGBMemory();cursor=0;sampled=[];records=[];inputs={str((D/'ideal_sensor_rows.npz').resolve()):sha(D/'ideal_sensor_rows.npz')};poses=[]
def load(p):inputs[str(p.resolve())]=sha(p);return np.load(p)
for f in sorted((O/'source_predictions').glob('*.npz')):
 prediction=load(f);i=int(prediction['frame']);t=float(prediction['time_s'])
 while cursor<len(s['time_s']) and s['time_s'][cursor]<=t:
  st=float(s['time_s'][cursor]);rolling.consume(dict(time_s=st,gyro_body_rad_s=s['gyro_body_rad_s'][cursor],main_encoder_rad_s=float(s['main_encoder_rad_s'][cursor])),s['initial_normal'] if cursor==0 else None);b.consume(rolling.records[-1])
  if cursor%5==0:sampled.append(b.at(st))
  cursor+=1
 pose=b.at(t);poses.append(pose);mask=load(O/'masks'/f'frame_{i:03d}.npz');rgb=load(O/'online_inputs'/f'frame_{i:03d}.npz');known={k:pose[k] for k in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']};pred={k:prediction[k] for k in ['depth_m','confidence','frame','time_s']};pred['disparity_native_px']=restore_native(prediction['disparity_px']);source=point_packet(pred,known,mask['left'],mask['right']);haz=int(((source['world'][:,2]>.05)&(source['world'][:,2]<=1.5)).sum());low=len(sampled)>=5 and all(np.linalg.norm(p['velocity_world_m_s'][:2])<.05 for p in sampled[-5:]);qualified=18-1e-6<=t<24-1e-6 and low and haz>=100
 if not memory.seeded and qualified:memory.seed(source,rgb,t,qualified=True)
 proof=memory.query(rgb,known,mask,t);w=proof['past_world'][proof['appearance_corroborated']];np.savez_compressed(D/'estimated_poses'/f'frame_{i:03d}.npz',**pose);np.savez_compressed(D/'source_packets'/f'source_{i:03d}.npz',**source);np.savez_compressed(D/'memory_predictions'/f'frame_{i:03d}.npz',**proof,frame=i,time_s=t);records.append(dict(frame=i,time_s=t,qualified_seed=qualified,seeded=memory.seeded,seed_frame=memory.source_frame,hazards=haz,support=int(proof['appearance_corroborborated'].sum()) if 'appearance_corroborborated' in proof else int(proof['appearance_corroborated'].sum()),unique_10cm_XY_bins=len(np.unique(np.floor(w[:,:2]/.1).astype(int),axis=0)),estimated_XY_speed_m_s=float(np.linalg.norm(pose['velocity_world_m_s'][:2])),geometry_time_s=float(memory.geometry_time) if memory.seeded else None))
(D/'records.json').write_text(json.dumps(records,indent=2)+'\n');np.savez_compressed(D/'all_pose_samples.npz',time_s=np.array([p['time_s'] for p in sampled]),position=np.array([p['root_position_world_m'] for p in sampled]),rotation=np.array([p['root_rotation_world'] for p in sampled]),velocity=np.array([p['velocity_world_m_s'] for p in sampled]))
files=[p for p in D.rglob('*') if p.is_file()];(D/'prediction_completion.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),code_sha256=sha(__file__),input_sha256=inputs,output_sha256={str(f.resolve()):sha(f) for f in files},all71_predictions_before_labels=True,only_ideal_sensor_and_initial_gauge_pose_inputs=True,current_truth_gyro_emulator_privilege=True,not_native_IMU_or_RGB_only=True,old_models_not_refitted=True,whole_research_complete=False),indent=2)+'\n');print('Frozen pose stress all source-only outputs sealed',len(records),len(sampled))
