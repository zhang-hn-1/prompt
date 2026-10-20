"""Postseal renderer evaluation; never imports producer or changes correspondence."""
from pathlib import Path
import numpy as np,json,hashlib
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'lk_surface_development_v1';seal=json.loads((O/'prediction_seal.json').read_text())
for name,s in seal['output_sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==s
pred=json.loads((O/'predictions.json').read_text());obj=next(x for x in json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text()) if x['path']=='/E1Detour/Divider/divider_06');c=np.array(obj['position']);errors=[];rows=[]
for packet in pred:
 f=packet['frame'];p=packet['prediction'];q=np.array(p['query_xy']);gt=np.load(D/'frames'/f'frame_{f:03d}.npz')['left_depth_m'];T=np.load(D/'known_poses'/f'frame_{f:03d}.npz')['left_camera_usd_world'];u,v=np.rint(q).astype(int).T;z=gt[v,u];R=T[:3,:3]@np.diag([1,-1,-1])
 with np.errstate(invalid='ignore'):truth=T[:3,3]+np.c_[(q[:,0]-319.5)/584.8888888888889*z,(q[:,1]-239.5)/584.8888888888889*z,z]@R.T
 valid=np.isfinite(truth).all(1)&np.isfinite(z)&(z>0);target=valid&(np.linalg.norm(truth[:,:2]-c[:2],axis=1)<=obj['radius']+.001)&(np.abs(truth[:,2]-c[2])<=obj['height']/2+.001)
 e=[];nt=0;correcttarget=0
 for point in p['points']:
  i=point['query'][2];err=float(np.linalg.norm(np.array(point['world'])-truth[i])) if valid[i] else float('inf');e.append(err);nt+=int(target[i]);correcttarget+=int(target[i] and err<=.1)
 stages=[]
 for stage in p['stages']:
  tm=[g for g in stage['geometries'] if target[g['query_index']]];geom=[g['result'] for g in tm if g['result'] is not None]
  stages.append(dict(source_frame=stage['source_frame'],all_matches=len(stage['geometries']),target_matches=len(tm),target_geometry_valid=len(geom),target_sigma_pass=sum(x['sigma_m']<=.1 for x in geom)))
 errors.extend(e);rows.append(dict(frame=f,query_features=len(q),points=len(e),within10cm=sum(x<=.1 for x in e),target_query_features=int(target.sum()),target_points=nt,correct_target_points=correcttarget,stages=stages))
e=np.array(errors);precision=float(np.mean(e<=.1)) if len(e) else None
# Explicit unconditional quantile with infinite error mass retained; no finite-only removal.
p90=float(np.quantile(e,.9)) if len(e) and np.isfinite(e).all() else (None if not len(e) else 'infinite_or_nonfinite_unconditional_quantile')
gates=dict(point100=len(e)>=100,geometry95=precision is not None and precision>=.95,p90=isinstance(p90,float) and p90<=.1,target_two100epochs=sum(r['target_points']>=100 for r in rows)>=2)
result=dict(decision='REJECT_ONE_FROZEN_LK_SURFACE_CANDIDATE' if not all(gates.values()) else 'PASS_DEVELOPMENT_ONLY',points=len(e),within10cm_fraction=precision,unconditional_p90_m=p90,target_points=sum(r['target_points'] for r in rows),correct_target_points=sum(r['correct_target_points'] for r in rows),gates=gates,preexecution_target100_unreachable_with_all_querycounts_below100=True,rows=rows,GT_postseal_only=True,no_controller_or_map_install=True,development_not_fresh=True,whole_research_complete=False,assessor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
