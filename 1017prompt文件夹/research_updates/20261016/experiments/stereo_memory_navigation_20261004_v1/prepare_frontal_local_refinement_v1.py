"""Historical actor-file adapter only; no truth file read, inference or physics."""
from pathlib import Path
import json,hashlib,datetime,shutil,numpy as np,cv2
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from partial_surface_memory import PartialSurfaceMemory
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'frontal_local_refinement_actor_inputs_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=D/'artifact_manifest_post_close.json'
for f,s in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==s,f
O.mkdir(exist_ok=False)
for folder in ['source_inputs','partial_observations','estimated_poses','partial_class_masks','source_predictions','history','sensor_packets']:(O/folder).mkdir()
model=np.load(H/'partial_surface_frozen_appearance_v1.npz');cad=RobotSelfMask();memory=PartialSurfaceMemory();rows=[];original_files={}
for i in range(0,141,5):
 name=f'frame_{i:03d}.npz'
 aliases=[(D/'online_inputs'/name,O/'source_inputs'/name),(D/'estimated_poses'/name,O/'estimated_poses'/name),(D/'history'/name,O/'history'/name),(D/'source_predictions'/name,O/'source_predictions'/name),(D/'sensor_packets'/f'source_{i:03d}.npz',O/'sensor_packets'/f'source_{i:03d}.npz')]
 for a,b in aliases:original_files[str(a)]=sha(a);shutil.copyfile(a,b);assert sha(a)==sha(b)
 rgb=np.load(aliases[0][1]);pose=np.load(aliases[1][1]);history=np.load(aliases[2][1]);prediction=np.load(aliases[3][1]);packet=np.load(aliases[4][1]);t=float(packet['time_s'])
 assert float(rgb['time_s'])==float(pose['time_s'])==float(prediction['time_s'])==t and int(packet['frame'])==int(prediction['frame'])==i
 masks=cad.project(joints_from_proprio(history['proprio_history']));classes={eye:(np.linalg.norm((cv2.cvtColor(rgb[eye+'_rgb'],cv2.COLOR_RGB2LAB).astype(float)-model['centre'])/model['scale'],axis=-1)<=3)&~masks[eye] for eye in ['left','right']}
 np.savez_compressed(O/'partial_class_masks'/name,**classes,frame=i,time_s=t,appearance_frozen_before_sequence=True)
 surface=memory.update(packet,classes['left'],classes['right'],t,{str(a):sha(a) for a,_ in aliases})
 np.savez_compressed(O/'partial_observations'/name,**{k:v for k,v in surface.items() if k!='input_hashes'})
 rows.append(dict(frame=i,time_s=t,points=len(surface['world']),historical_original_sequence='native_flat_pose_navigation_R1_run_v1',not_current_capture=True))
(O/'source_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
(O/'actor_input_manifest_v1.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact_sha256={str(p):sha(p) for p in O.rglob('*') if p.is_file()},original_files_sha256=original_files,parent_postseal_sha256=sha(manifest),role='HISTORICAL_FILENAME_ADAPTER_NOT_ACTUAL_RUN',no_truth_files_read=True,no_network_or_physics=True,source_model_admitted=False,code_sha256=sha(__file__)),indent=2)+'\n')
print('Historical0..14 actor aliases only:',len(rows),'epochs',sum(r['points'] for r in rows),'partial points before refinement; no truth')
