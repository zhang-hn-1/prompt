"""Postsealed removed-point assessment; no teacher feeds the pixel rule."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'E1_current_pixel_consumer_historical_v1';sealed=json.loads((O/'seal.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,h in sealed['inputs'].items():assert sha(p)==h
for p,h in sealed['outputs'].items():assert sha(p)==h
shapes=json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text());total={k:0 for k in ['original_points','candidate_points','original_correct10cm','candidate_correct10cm','removed_points','removed_GT_missing','removed_GT_valid','removed_correct10cm','removed_true_collision_surface']};rows=[]
for sp in sorted((D/'partial_observations').glob('frame_*.npz')):
 f=int(sp.stem.split('_')[-1]);a=np.load(sp);rgb=np.load(D/'source_inputs'/sp.name);g=np.load(D/'frames'/sp.name);pose=np.load(D/'known_poses'/sp.name);q=a['query_pixel_id'];r=a['partner_pixel_id'];x,y=q%640,q//640;remove=np.all(rgb['left_rgb'][y,x]==0,axis=1)|np.all(rgb['right_rgb'][r//640,r%640]==0,axis=1);depth=g['left_depth_m'][y,x];valid=np.isfinite(depth)&(depth>0);rays=np.column_stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))]);T=pose['left_camera_usd_world'];world=T[:3,3]+(rays*np.where(valid,depth,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(world-a['world'],axis=1);correct=valid&(err<=.1);physical=np.zeros(len(q),bool)
 for obj in shapes:
  c=np.asarray(obj['position']);physical|=valid&(np.linalg.norm(world[:,:2]-c[:2],axis=1)<=obj['radius']+.001)&(abs(world[:,2]-c[2])<=obj['height']/2+.001)
 row=dict(frame=f,original_points=len(q),candidate_points=int((~remove).sum()),original_correct10cm=int(correct.sum()),candidate_correct10cm=int((correct&~remove).sum()),removed_points=int(remove.sum()),removed_GT_missing=int((remove&~valid).sum()),removed_GT_valid=int((remove&valid).sum()),removed_correct10cm=int((remove&correct).sum()),removed_true_collision_surface=int((remove&physical).sum()))
 for k in total:total[k]+=row[k]
 rows.append(row)
result=dict(scope='HISTORICAL_POSTSEAL_SOURCE_QUALITY_NOT_FRESH_OR_CONTROL_ACCEPTANCE',totals=total,rows=rows,original_precision=total['original_correct10cm']/total['original_points'],candidate_precision=total['candidate_correct10cm']/total['candidate_points'],filter_prior_to_GT=True,no_source_gate_override=True,no_teacher_actor=True,code_sha256=sha(__file__));(O/'postseal_removed_points_assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
