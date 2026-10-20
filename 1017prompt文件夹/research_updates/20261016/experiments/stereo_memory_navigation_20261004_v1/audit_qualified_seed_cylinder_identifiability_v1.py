"""One source-only shape fit; spatial sensitivity is NOT fresh validation."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.optimize import least_squares
H=Path(__file__).resolve().parent;A=H/'qualified_seed_cylinder_identifiability_v1';A.mkdir(exist_ok=False)
source=H/'qualified_hazard_navigation_run_v1/sensor_packets/source_180.npz';plan=H.parent.parent/'docs/superpowers/plans/2026-10-07-qualified-seed-shape-identifiability.md'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();ident={str(p):sha(p) for p in [source,plan,Path(__file__)]};(A/'pre_fit_identity.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sha256=ident,GT_input=False,development_only=True),indent=2)+'\n')
p=np.load(source);x=p['world'];select=(x[:,2]>.05)&(x[:,2]<=1.5);xy=x[select,:2];bins=np.floor(xy/.1).astype(int);keys=sorted(set(map(tuple,bins)));assert len(keys)<=16,'bounded spatial sensitivity domain exceeded'
def fit(xy):
 center=xy.mean(0);x=xy-center;matrix=np.c_[2*x,np.ones(len(x))];rhs=(x*x).sum(1);beta,_,rank,_=np.linalg.lstsq(matrix,rhs,rcond=None);r2=beta[2]+(beta[:2]**2).sum()
 if rank<3 or r2<=0:return dict(fit_valid=False,rank=int(rank))
 theta=np.r_[beta[:2],np.sqrt(r2)];result=least_squares(lambda q:np.linalg.norm(x-q[:2],axis=1)-q[2],theta,max_nfev=200)
 norms=np.linalg.norm(x-result.x[:2],axis=1);J=np.c_[(result.x[:2]-x)/norms[:,None],-np.ones(len(x))];sv=np.linalg.svd(J,compute_uv=False);res=norms-result.x[2]
 return dict(fit_valid=bool(result.success and np.isfinite(result.x).all()),rank=int(rank),center_world_xy_m=(result.x[:2]+center).tolist(),radius_m=float(result.x[2]),parameter_jacobian_condition=float(sv[0]/sv[-1]),radial_absolute_residual_p90_m=float(np.quantile(np.abs(res),.9)),radial_residual_RMS_m=float(np.sqrt(np.mean(res*res))),optimizer_evaluations=int(result.nfev))
full=fit(xy);blocks=[]
for key in keys:
 keep=np.any(bins!=np.array(key),axis=1);f=fit(xy[keep]);dc=float(np.linalg.norm(np.array(f['center_world_xy_m'])-full['center_world_xy_m'])) if f.get('fit_valid') and full.get('fit_valid') else None;dr=abs(f['radius_m']-full['radius_m']) if dc is not None else None;blocks.append(dict(excluded_bin=list(key),remaining_points=int(keep.sum()),fit=f,center_shift_m=dc,radius_shift_m=dr))
passed=bool(len(keys)>=3 and full.get('fit_valid') and full['rank']==3 and full['parameter_jacobian_condition']<=1000 and .05<=full['radius_m']<=2 and full['radial_absolute_residual_p90_m']<=.1 and all(r['center_shift_m'] is not None and r['center_shift_m']<=.1 and r['radius_shift_m']<=.1 for r in blocks))
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_SOURCE_ONLY_IDENTIFIABILITY_NOT_MODEL_ADMISSION' if passed else 'REJECT_SINGLE_CYLINDER_IDENTIFIABILITY',point_count=len(xy),spatial_bins=[list(k) for k in keys],source_time_s=float(p['time_s']),source_frame=int(p['frame']),full_fit=full,leave_one_spatial_bin_sensitivity=blocks,no_GT_input=True,no_future_feedback_input=True,old_source_used_for_design_not_fresh=True,source_model_admitted=False,predicted_geometry_not_observed=True,shape_precision_not_tested=True,whole_research_complete=False,formal_navigation_triggered=False)
f=A/'shape_identifiability_v1.json';f.write_text(json.dumps(out,indent=2)+'\n');(A/'output_identity.json').write_text(json.dumps(dict(sha256=sha(f),source_sha256=ident,code_sha256=sha(__file__)),indent=2)+'\n');print(json.dumps(out,indent=2))
