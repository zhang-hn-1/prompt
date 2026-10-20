"""Independent read-only sealed packet review; no predictor import or rerun."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np
import cv2
H=Path(__file__).resolve().parents[1];D=H/'temporal_ray_sweep_diagnostic_v1';O=H/'qualified_hazard_navigation_run_v1';E=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'prediction_seal.json').read_text());checked={p:sha(p)==s for group in ['input_sha256','output_sha256'] for p,s in seal[group].items()};assert all(checked.values())
protocol=json.loads((D/'protocol.json').read_text())
bound={'plan_sha256':H.parent.parent/'docs/superpowers/plans/2026-10-07-temporal-ray-sweep-diagnostic.md','code_sha256':H/'predict_temporal_ray_sweep_v1.py','module_sha256':H/'temporal_ray_sweep.py','CPU_suite_log_sha256':H/'temporal_ray_sweep_full_suite_v1.log'}
assert all(sha(p)==protocol[k] for k,p in bound.items())
read={}
def load(p):
 read[str(p)]=sha(p)
 return np.load(p)
F=584.8888888888889;flip=np.diag([1.,-1.,-1.]);offset=np.array([(x,y) for y in range(-3,4) for x in range(-3,4)])
def world(uv,z,T):
 ray=np.c_[(uv[:,0]-319.5)/F,(uv[:,1]-239.5)/F,np.ones(len(uv))]
 return T[:3,3]+(ray*z[:,None])@(T[:3,:3]@flip).T
def projection(points,T):
 local=(points-T[:3,3])@(T[:3,:3]@flip)
 return np.c_[F*local[:,0]/local[:,2]+319.5,F*local[:,1]/local[:,2]+239.5],local[:,2]
seed=load(O/'sensor_packets/source_180.npz')['world'];seed=seed[(seed[:,2]>.05)&(seed[:,2]<=1.5)];seedbins=set(map(tuple,np.floor(seed[:,:2]/.1).astype(int)));rows=[]
for record in json.loads((D/'records.json').read_text()):
 i=record['frame'];eye=record['eye'];q=load(D/record['file']);uv=q['current_uv'];z=q['selected_depth_m'];points=q['world'];C=load(O/'known_poses'/f'frame_{i:03d}.npz')[eye+'_camera_usd_world'];rgb=load(O/'online_inputs'/f'frame_{i:03d}.npz');cm=load(O/'masks'/f'frame_{i:03d}.npz');assert float(rgb['time_s'])==float(cm['time_s'])==float(q['current_time_s'])
 reconstructed=world(uv,z,C);assert np.max(np.abs(reconstructed-points))<1e-10
 assert np.array_equal(q['source_frames'],[i-10,i-20]);assert np.all(q['source_time_s']<q['current_time_s']);assert np.allclose(q['current_time_s']-q['source_time_s'],[1,2],atol=1e-6)
 assert len(np.unique(q['query_pixel_id']))==len(uv);assert np.array_equal(q['query_pixel_id'],uv[:,1]*640+uv[:,0]);assert np.all(uv%2==0)
 yy,xx=uv.astype(int).T[::-1];assert np.all(q['score'][yy,xx]>=.8);assert np.all(q['uniqueness_margin'][yy,xx]>=.1);assert np.all((z>=.5)&(z<=3.));assert not q['certified_free'] and not q['source_model_admitted']
 errors=[];mask_contaminated=np.zeros(len(uv),bool);boundary_contaminated=np.zeros(len(uv),bool);nearest_all_valid=np.ones(len(uv),bool);project_error=[];centred_baseline=[]
 for ti,j in enumerate([i,i-10,i-20]):
  t=load(O/'offline_truth'/f'frame_{j:03d}.npz');T=t[eye+'_camera_usd_world'];pred_uv,pz=projection(points,T);label_uv=uv if ti==0 else pred_uv
  integer=np.rint(label_uv).astype(int);inside=(integer[:,0]>=0)&(integer[:,0]<640)&(integer[:,1]>=0)&(integer[:,1]<480)&(pz>0);err=np.full(len(uv),np.nan)
  d=t[eye+'_depth_m'][integer[inside,1],integer[inside,0]];truthpoint=world(integer[inside],d,T);err[inside]=np.linalg.norm(points[inside]-truthpoint,axis=1);errors.append(err)
  if ti==0:
   root_error=float(np.max(np.abs(q['derived_root_position_world_m']-t['root_position_world_m'])))
  else:
   assert np.max(np.abs(pred_uv-q['past_uv'][ti-1]))<1e-8
   B=load(O/'known_poses'/f'frame_{j:03d}.npz')[eye+'_camera_usd_world'];m=load(O/'masks'/f'frame_{j:03d}.npz')[eye];centred_baseline.append(float(np.linalg.norm(B[:3,3]-C[:3,3])))
   patch_uv=(uv[:,None,:]+offset).reshape(-1,2);patch_z=np.repeat(z,49);warped,pdepth=projection(world(patch_uv,patch_z,C),B);mx=warped[:,0].astype(np.float32).reshape(-1,49);my=warped[:,1].astype(np.float32).reshape(-1,49)
   # Exactly sample OpenCV's actual weighted mask interpolation; any positive value
   # means a masked pixel contributes to the RGB remap at the selected depth.
   weighted=cv2.remap(m.astype(np.float32),mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
   nearest=cv2.remap(m.astype(np.uint8),mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=1)
   inside_patch=(mx>=0)&(mx<=639)&(my>=0)&(my<=479)&(pdepth.reshape(-1,49)>0)
   mask_contaminated|=(weighted>0).any(1);nearest_all_valid&=(inside_patch&(nearest==0)).all(1)
   # Constant-zero border contributes if image-validity weight is below one.
   support=cv2.remap(np.ones((480,640),np.float32),mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
   boundary_contaminated|=(support<1).any(1)
 errors=np.array(errors).T;finite=np.isfinite(errors).all(1);correct=finite&(errors<=.1).all(1);bins=np.floor(points[:,:2]/.1).astype(int);new=np.array([tuple(b) not in seedbins for b in bins]);n=len(points)
 np.savez_compressed(E/f'geometry_{i:03d}_{eye}.npz',errors_m=errors,correct=correct,finite=finite,new_bin=new,masked_interpolation=mask_contaminated,nearest_patch_valid=nearest_all_valid)
 rows.append(dict(frame=i,eye=eye,n=n,correct=int(correct.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~correct).sum()),precision=float(correct.sum()/n),correct_new_points=int((correct&new).sum()),correct_new_bins=len(set(map(tuple,bins[correct&new]))),root_max_abs_error_m=root_error,masked_interpolation_queries=int(mask_contaminated.sum()),masked_interpolation_wrong=int((mask_contaminated&~correct).sum()),border_interpolation_queries=int(boundary_contaminated.sum()),nearest_patch_valid_queries=int(nearest_all_valid.sum()),baselines_m=centred_baseline))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),seal_files_checked=len(checked),seal_all_match=True,protocol_bound_sources_all_match=True,personally_reran_only_five_unit_tests=True,predictor_rerun=False,full482_suite_personally_rerun=False,rows=rows,all_nonempty_precision_pass=all(r['precision']>=.95 for r in rows),decision='REJECT_SINGLE_SWEEP',truth_inputs_sha256=read)
(E/'independent_geometry.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='truth_inputs_sha256'},indent=2))
