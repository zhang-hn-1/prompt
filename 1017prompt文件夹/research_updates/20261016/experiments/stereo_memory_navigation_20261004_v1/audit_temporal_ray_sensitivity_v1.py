"""Source-only coordinate sensitivity, descriptive after REJECT; no new gate."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'temporal_ray_sweep_diagnostic_v1';a=json.loads((D/'assessment_v1.json').read_text());rows=[]
def rays(uv,T):
 r=np.c_[(uv[:,0]-319.5)/584.8888888888889,(uv[:,1]-239.5)/584.8888888888889,np.ones(len(uv))]@(T[:3,:3]@np.diag([1,-1,-1])).T
 return r/np.linalg.norm(r,axis=1,keepdims=True)
def closest(u,v,A,B):
 r=rays(u,A);s=rays(v,B);delta=B[:3,3]-A[:3,3];c=np.sum(r*s,axis=1);x=r@delta;y=s@delta;den=1-c*c
 with np.errstate(divide='ignore',invalid='ignore'):l=(x-c*y)/den;m=(c*x-y)/den;p=(A[:3,3]+l[:,None]*r+B[:3,3]+m[:,None]*s)/2
 return p,l,m
for r in a['rows']:
 f=np.load(D/r['file']);eye=r['eye'];i=r['frame'];A=np.load(O/'known_poses'/f'frame_{i:03d}.npz')[eye+'_camera_usd_world'];u=f['current_uv'];w=f['world'];sig=np.zeros(len(w));angles=[];nominal=0.
 for j,v in zip(f['source_frames'],f['past_uv']):
  B=np.load(O/'known_poses'/f'frame_{j:03d}.npz')[eye+'_camera_usd_world'];p,l,m=closest(u,v,A,B);nominal=max(nominal,float(np.max(np.linalg.norm(p-w,axis=1))) if len(w) else 0.);angles.append(np.degrees(np.arccos(np.clip(np.sum(rays(u,A)*rays(v,B),axis=1),-1,1))))
  for k in range(4):
   for sign in [-1.,1.]:
    uu=u.copy().astype(float);vv=v.copy()
    if k<2:uu[:,k]+=sign
    else:vv[:,k-2]+=sign
    pp,ll,mm=closest(uu,vv,A,B);s=np.linalg.norm(pp-p,axis=1);s=np.where(np.isfinite(s)&(ll>0)&(mm>0),s,np.inf);sig=np.maximum(sig,s)
 name=f'frame_{i:03d}_{eye}_sensitivity.npz';np.savez_compressed(D/name,one_pixel_sensitivity_m=sig,past_parallax_deg=angles,current_frame=i,eye=eye,descriptive_not_certified_bound=True,no_candidate_selection=True);finite=np.isfinite(sig);rows.append(dict(frame=i,eye=eye,n=len(w),nominal_reconstruction_peak_m=nominal,finite=int(finite.sum()),sensitivity_p50_m=float(np.quantile(sig[finite],.5)) if finite.any() else None,sensitivity_p90_m=float(np.quantile(sig[finite],.9)) if finite.any() else None,descriptive_gt10cm=int((sig>.1).sum()),min_parallax_deg=float(np.min(angles)) if len(w) else None))
f=D/'pixel_sensitivity_audit_v1.json';assert not f.exists();f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),no_GT_or_action_reads=True,no_posthoc_filtered_gate=True,not_uncertainty_bound_or_accuracy=True,old_REJECT_preserved=True),indent=2)+'\n');print(rows)
