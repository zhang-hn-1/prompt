from pathlib import Path
import numpy as np,json,hashlib
D=Path(__file__).resolve().parent;R=D.parent/'replanning_reserve_20261013_v1';out={};flip=np.diag([1,-1,-1]);K=584.8888888888889
quant=lambda a:np.quantile(a,[.5,.9,.95,.99]).tolist() if len(a) else None
for arm in ['baseline_e1','candidate_e1']:
 r=R/(arm+'_run');errors=[[],[],[]];poses=[];frames=[];ids={};total=validn=0
 for p in sorted((r/'partial_observations').glob('*.npz')):
  a=np.load(p);f=np.load(r/'frames'/p.name);n=np.load(r/'known_poses'/p.name);q=a['query_pixel_id'].astype(int);x=q%640;y=q//640;z=f['left_depth_m'][y,x];ok=np.isfinite(z)&(z>0);total+=len(q);validn+=int(ok.sum());T=f['left_camera_usd_world'];E=n['left_camera_usd_world'];rg=T[:3,:3]@flip;re=E[:3,:3]@flip
  local=(a['world'][ok]-E[:3,3])@re;gtpose_pred=T[:3,3]+local@rg.T;rays=np.stack([(x[ok]-319.5)/K,(y[ok]-239.5)/K,np.ones(ok.sum())],axis=1);gt=T[:3,3]+(rays*z[ok,None])@rg.T
  posevector=a['world'][ok]-gtpose_pred;depthvector=gtpose_pred-gt;fullvector=a['world'][ok]-gt
  np.testing.assert_allclose(posevector+depthvector,fullvector,atol=1e-12,rtol=0)
  vectors=[np.linalg.norm(v,axis=1) for v in [posevector,depthvector,fullvector]]
  for e,v in zip(errors,vectors):e.extend(v)
  pos=float(np.linalg.norm(E[:3,3]-T[:3,3]));angle=float(np.arccos(np.clip((np.trace(re.T@rg)-1)/2,-1,1)));poses.append([pos,angle]);frames.append(dict(frame=p.name,time_s=float(f['time_s']),accepted_points=len(q),finite_GT=int(ok.sum()),world_pose_only_p90_m=None if not len(vectors[0]) else float(np.quantile(vectors[0],.9)),measurement_only_p90_m=None if not len(vectors[1]) else float(np.quantile(vectors[1],.9)),full_p90_m=None if not len(vectors[2]) else float(np.quantile(vectors[2],.9)),camera_position_error_m=pos,camera_rotation_error_rad=angle))
  for path in [p,r/'frames'/p.name,r/'known_poses'/p.name]:ids[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
 a=np.asarray(poses);out[arm]=dict(source_frames=len(frames),accepted_instances=total,finite_GT=validn,finite_GT_fraction=validn/total,pose_only_p50_p90_p95_p99_m=quant(errors[0]),measurement_only_p50_p90_p95_p99_m=quant(errors[1]),full_p50_p90_p95_p99_m=quant(errors[2]),camera_position_max_m=float(a[:,0].max()),camera_rotation_max_rad=float(a[:,1].max()),frames=frames,input_sha256=ids)
(D/'qualification.json').write_text(json.dumps(dict(arms=out,scope='Vector identity full=pose contribution+measurement contribution. Norm quantiles do not add. GT only offline, original accepted point domain; no hole filling, no floor relabeling, no memory clearing. Exact GT-pose reprojection is diagnostic, not online correction. Grid quantization max centre shift .070711m is separate geometry, not measured point error.'),indent=2)+'\n');print({k:{x:y for x,y in v.items() if x not in ['frames','input_sha256']} for k,v in out.items()})
