from pathlib import Path
import numpy as np,json,hashlib
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'anchor_evidence_development_v1';rows=[];counts={k:0 for k in ['eligible_valid_floor_or_lower','eligible_valid_hazard_height','eligible_valid_above_hazard','miss_floor_or_lower','miss_hazard_height','miss_above_hazard']}
for row in json.loads((O/'predictions.json').read_text()):
 f=row['frame'];gt=np.load(D/'frames'/f'frame_{f:03d}.npz');T=gt['left_camera_usd_world']
 for pair in row['geometries']:
  if not pair['evidence']['eligible_anchor']:continue
  r=pair['original'];q=np.asarray(r['query_xy']);x,y=np.rint(q).astype(int);z=float(gt['left_depth_m'][y,x])
  if not np.isfinite(z) or z<=0:continue
  w=T[:3,3]+(T[:3,:3]@np.diag([1,-1,-1]))@(np.array([(q[0]-319.5)/584.8888888888889,(q[1]-239.5)/584.8888888888889,1.])*z);height='floor_or_lower' if w[2]<=.05 else ('hazard_height' if w[2]<=1.5 else 'above_hazard');inside=r['status']=='bounded' and np.all(np.asarray(r['constraint_A'])@w<=np.asarray(r['constraint_b'])+1e-7);counts['eligible_valid_'+height]+=1
  if not inside:
   counts['miss_'+height]+=1;past=np.load(D/'known_poses'/f"frame_{r['source_frame']:03d}.npz")['left_camera_usd_world'];op=(past[:3,:3]@np.diag([1,-1,-1])).T@(w-past[:3,3]);uv=op[:2]/op[2]*584.8888888888889+[319.5,239.5];rows.append(dict(query_frame=f,source_frame=r['source_frame'],query_xy=r['query_xy'],source_xy=r['source_xy'],GT_projected_source_xy=uv.tolist(),world_height_m=float(w[2]),height_type=height,polytope_status=r['status'],box_lo_world_m=r['lo_world_m'],box_hi_world_m=r['hi_world_m']))
r=dict(counts=counts,misses=rows,GT_aftersealed_prediction_and_reject_only=True,existing_height_split_point05_to1point5_not_new_gate=True,no_selection_for_model_or_controller=True,ground_prior_not_observed_free=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(O/'miss_height_diagnosis_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(counts)
