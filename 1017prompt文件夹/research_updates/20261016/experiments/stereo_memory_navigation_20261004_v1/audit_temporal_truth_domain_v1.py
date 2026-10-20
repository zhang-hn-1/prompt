"""Offline layer attribution only: missing labels/domain vs wrong correspondence."""
from pathlib import Path
import numpy as np,json,datetime,hashlib
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'temporal_ray_sweep_diagnostic_v1';a=json.loads((D/'assessment_v1.json').read_text());rows=[]
for r in a['rows']:
 f=np.load(D/r['file']);uv=f['current_uv'];x,y=uv.astype(int).T;eye=r['eye'];i=r['frame'];truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');z=truth[eye+'_depth_m'][y,x];T=truth[eye+'_camera_usd_world'];rays=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
 with np.errstate(invalid='ignore'):world=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(world-f['world'],axis=1)
 finite=np.isfinite(err);wrong=finite&(err>.1);domain=np.isfinite(z)&(z>=.5)&(z<=3)&(world[:,2]>.05)&(world[:,2]<=1.5)&(np.linalg.norm(world-truth['root_position_world_m'],axis=1)>.55);shifts=[]
 for j,model_uv in zip(f['source_frames'],f['past_uv']):
  P=np.load(O/'known_poses'/f'frame_{j:03d}.npz')[eye+'_camera_usd_world'];loc=(world-P[:3,3])@(P[:3,:3]@np.diag([1,-1,-1]))
  with np.errstate(divide='ignore',invalid='ignore'):true_uv=loc[:,:2]/loc[:,2,None]*584.8888888888889+[319.5,239.5];shift=np.linalg.norm(true_uv-model_uv,axis=1)
  shifts.append(shift)
 s=np.asarray(shifts).T;goodshift=wrong&np.isfinite(s).all(1);rows.append(dict(frame=i,eye=eye,n=len(z),current_finite_wrong=int(wrong.sum()),current_wrong_with_true_axial_range_lt50cm=int((wrong&(z<.5)).sum()),current_wrong_with_true_axial_range_gt3m=int((wrong&(z>3)).sum()),current_wrong_with_true_original_source_domain=int((wrong&domain).sum()),current_wrong_true_source_domain_missing=int((wrong&~domain).sum()),wrong_valid_temporal_shifts=int(goodshift.sum()),wrong_temporal_shift_min_p50_px=float(np.quantile(np.min(s[goodshift],axis=1),.5)) if goodshift.any() else None,wrong_atleast_one_temporal_shift_gt1p5px=int((goodshift&np.any(s>1.5,axis=1)).sum()),not_source_error_bound=True))
f=D/'truth_domain_audit_v1.json';assert not f.exists();f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),GT_only_offline=True,no_new_candidates_or_thresholds=True,no_filtered_pass_claim=True),indent=2)+'\n');print(rows)
