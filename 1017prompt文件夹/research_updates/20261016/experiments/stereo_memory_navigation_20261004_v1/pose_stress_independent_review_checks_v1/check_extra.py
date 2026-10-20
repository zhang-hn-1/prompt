from pathlib import Path
import numpy as np,json,hashlib
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent.parent;D=H/'RGB_memory_frozen_pose_stress_v1';O=H/'online_RGB_memory_fresh_run_v1';OUT=Path(__file__).parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
checks={}
s=json.loads((D/'prediction_completion.json').read_text());assert s['code_sha256']==sha(H/'predict_RGB_memory_frozen_pose_stress_v1.py');a=json.loads((D/'oracle_component_audit_v1/completion.json').read_text());assert a['code_sha256']==sha(H/'audit_RGB_memory_pose_components_v1.py');g=json.loads((D/'gyro_interval_contract_v1/prediction_seal.json').read_text());assert g['output_sha256']==sha(D/'gyro_interval_contract_v1/all_rotations_before_labels.npz');assert g['input_sensor_sha256']==sha(D/'ideal_sensor_rows.npz');assert g['code_sha256']==sha(H/'audit_gyro_interval_contract_v1.py');r=json.loads((D/'projection_observability_v1/source_only_result.json').read_text());assert r['code_sha256']==sha(H/'audit_RGB_memory_projection_observability_v1.py');checks['single_field_seals_match']=True
states=json.loads((O/'states.json').read_text());pos_delta=[];physics=np.load(O/'physics.npz')
for f in sorted((D/'estimated_poses').glob('*.npz')):
 i=int(f.stem.split('_')[1]);real=np.load(O/'known_poses'/f.name);pos_delta.append(np.linalg.norm(physics['root_position_world_m'][i*5]-np.array(states[i]['position'])))
checks['unlisted_oracle_states_equals_original_sealed_physics_position_max_error']=float(max(pos_delta));checks['omitted_input_current_sha256']={str(p):sha(p) for p in [O/'states.json',D/'all_pose_samples.npz']}
checks['jacobian_max_abs_errors']={};checks['base_projection_max_uv_errors']={}
for frame in [185,190,195]:
 pose=np.load(D/'estimated_poses'/f'frame_{frame:03d}.npz');proof=np.load(D/'memory_predictions'/f'frame_{frame:03d}.npz');sel=proof['appearance_corroborated'];w=proof['past_world'][sel];rootR=pose['root_rotation_world'];rootp=pose['root_position_world_m'];Jstored=np.load(D/'projection_observability_v1'/f'frame_{frame:03d}_jacobian.npz')['J'];local={}
 for eye in ['left','right']:
  T=pose[eye+'_camera_usd_world'];local[eye]=(rootR.T@(T[:3,3]-rootp),rootR.T@T[:3,:3]@np.diag([1,-1,-1]))
 def projection(dp,rotation_delta):
  R=rootR@Rotation.from_rotvec(rotation_delta).as_matrix();p=rootp+dp;result=[]
  for eye in ['left','right']:
   lpos,lR=local[eye];campos=p+R@lpos;camR=R@lR;v=camR.T@(w-campos).T;uv=(v[:2]/v[2])*584.8888888888889+np.array([[319.5],[239.5]]);result.append(uv.T.ravel())
  return np.concatenate(result)
 projected=projection(np.zeros(3),np.zeros(3));uvsaved=np.concatenate([proof[eye+'_uv'][sel].ravel() for eye in ['left','right']]);checks['base_projection_max_uv_errors'][frame]=float(np.abs(projected-uvsaved).max());cols=[]
 for k in range(6):
  d=np.zeros(6);d[k]=1e-5;plus=projection(d[:3],d[3:]);minus=projection(-d[:3],-d[3:]);cols.append((plus-minus)/2e-5)
 diff=np.abs(np.array(cols).T-Jstored).max();assert diff<2e-7;checks['jacobian_max_abs_errors'][frame]=float(diff)
(OUT/'extra_checks.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2))
