"""One existing-prefix reproducibility audit, no NN rerun or fit."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;O=H/'same_prefix_source_variation_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();O.mkdir(exist_ok=False);inputs={};rows=[]
for old,new in [('flat_ground_pose_D3_run_v1','hypothesis_learning_H1_run_v1'),('offset_replication_D4_run_v1','hypothesis_learning_H2_run_v1')]:
 for i in [0,5,10,15,20]:
  datasets=[]
  for case in [old,new]:
   D=H/case;paths=[D/'online_inputs'/f'frame_{i:03d}.npz',D/'sensor_packets'/f'source_{i:03d}.npz',D/'estimated_poses'/f'frame_{i:03d}.npz'];inputs.update({str(f):sha(f) for f in paths});datasets.append([dict(np.load(f)) for f in paths])
  (rgb0,s0,p0),(rgb1,s1,p1)=datasets;eye_rows={}
  for eye in ['left','right']:
   a=rgb0[eye+'_rgb'];b=rgb1[eye+'_rgb'];T0=p0[eye+'_camera_usd_world'];T1=p1[eye+'_camera_usd_world'];eye_rows[eye]=dict(exact_RGB_pixels_fraction=float((a==b).all(2).mean()),RGB_MAE=float(np.abs(a.astype(float)-b).mean()),camera_translation_difference_m=float(np.linalg.norm(T0[:3,3]-T1[:3,3])),camera_rotation_matrix_max_difference=float(np.abs(T0[:3,:3]-T1[:3,:3]).max()))
  seed=[]
  for packet in [s0,s1]:
   haz=(packet['world'][:,2]>.05)&(packet['world'][:,2]<=1.5);seed.append(dict(hazard_count=int(haz.sum()),hazard_sigma_median_m=float(np.median(packet['sigma_m'][haz])) if haz.any() else None,query_pixel_ids=np.asarray(packet['query_pixel_id'])[haz]))
  ids0,ids1=seed[0].pop('query_pixel_ids'),seed[1].pop('query_pixel_ids');union=np.union1d(ids0,ids1);rows.append(dict(old=old,fresh=new,frame=i,eyes=eye_rows,source0=seed[0],source1=seed[1],hazard_query_ID_Jaccard=float(len(np.intersect1d(ids0,ids1))/len(union)) if len(union) else None))
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,causal_explanation_scope='changed RGB/seed inputs with nearly identical estimated pose; not calibrated uncertainty/no isolation of renderer vs depthNN stochasticity',no_new_NN_or_model_or_gate=True,input_sha256=inputs,code_sha256=sha(__file__));(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps([r for r in rows if r['frame']==20],indent=2))
