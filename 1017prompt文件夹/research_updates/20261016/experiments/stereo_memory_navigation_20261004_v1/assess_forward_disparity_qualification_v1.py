"""All sealed proposals evaluated; no renderer-based selection."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;D=H/'neutral_bank_visibility_run_v1';A=H/'forward_disparity_qualification_ablation_v1';seal=json.loads((A/'prediction_seal.json').read_text())
for p,s in seal['artifact_and_input_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==s
seed=np.load(D/'sensor_packets/source_180.npz')['world'];seed=seed[(seed[:,2]>.05)&(seed[:,2]<=1.5)];sb=set(map(tuple,np.floor(seed[:,:2]/.1).astype(int)));rows=[]
for i in seal['frames']:
 f=np.load(A/f'source_{i:03d}.npz');haz=(f['world'][:,2]>.05)&(f['world'][:,2]<=1.5);w=f['world'][haz];gt=np.load(D/'offline_truth'/f'frame_{i:03d}.npz');ee=[]
 for eye,k in [('left','query_pixel_id'),('right','partner_pixel_id')]:
  idx=f[k][haz];x,y=idx%640,idx//640;z=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world'];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):p=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;ee.append(np.linalg.norm(p-w,axis=1))
 e=np.asarray(ee).T;finite=np.isfinite(e).all(1);correct=finite&(e<=.1).all(1);b=np.floor(w[:,:2]/.1).astype(int);novel=np.array([tuple(x) not in sb for x in b],bool)&correct
 rows.append(dict(frame=i,time_s=float(f['time_s']),n=len(w),correct=int(correct.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~correct).sum()),precision=float(correct.mean()) if len(w) else None,correct_new_points=int(novel.sum()),correct_new_bins=len(set(map(tuple,b[novel])))))
prec=any(r['n'] for r in rows) and all(r['precision']>=.95 for r in rows if r['n']);size=sum(r['n']>=100 for r in rows)>=2;coverage=any(r['correct_new_points']>=20 and r['correct_new_bins']>=2 for r in rows)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_DEVELOPMENT_ONLY_REQUIRES_NEW_VALIDATION' if prec and size and coverage else 'REJECT_REMOVING_LR_QUALIFICATION',every_nonempty_geometry_ge95=prec,two_100point_updates=size,new20points2bins=coverage,rows=rows,GT_ONLY_AFTER_SEAL=True,new_network_inferences=0,original_source_outputs_unchanged=True,not_fresh_or_online_or_navigation=True)
p=A/'assessment_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
