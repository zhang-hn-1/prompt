from pathlib import Path
import json,hashlib,datetime
import numpy as np
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from temporal_rgb_support import supports
from stereo_geometry import FOCAL
from audit_stereo import summarize
H=Path(__file__).resolve().parent;D=H/'native_goal_camera_observation_run_v1';O=H/'temporal_right_rgb_support_preflight_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for f,s in manifest.items():assert sha(f)==s,f
rows=json.loads((D/'source_rows.json').read_text());cad=RobotSelfMask();history=[];proofs=[];inputs={}
# Only current/past actor-side files; no truth or fixture geometrical filter.
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz'
 files=[D/'partial_observations'/name,D/'partial_class_masks'/name,D/'estimated_poses'/name,D/'history'/name]
 for f in files:inputs[str(f)]=sha(f)
 s=np.load(files[0]);classes=np.load(files[1]);poses=np.load(files[2]);h=np.load(files[3])['proprio_history'];t=float(np.load(D/'sensor_packets'/f'source_{i:03d}.npz')['time_s'])
 masks=cad.project(joints_from_proprio(h));T=poses['right_camera_usd_world'].copy();T[:3,:3]=T[:3,:3]@np.diag([1,-1,-1])
 proof=supports(s['world'],i,t,history)
 np.savez_compressed(O/name,world=s['world'],query_pixel_id=s['query_pixel_id'],keep=proof['keep'],max_support_baseline_m=proof['max_support_baseline_m'])
 proofs.append(dict(frame=i,time_s=t,n=len(s['world']),retained=int(proof['keep'].sum()),support_frames=proof['support_frames']))
 history.append(dict(frame=i,time_s=t,optical_world=T,positive=classes['right'].copy(),self_mask=masks['right']))
(O/'point_proofs.json').write_text(json.dumps(proofs,indent=2)+'\n')
outputs={str(f):sha(f) for f in O.iterdir()}
(O/'pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_manifest_sha256=sha(D/'artifact_manifest_post_close.json'),input_sha256=inputs,output_sha256=outputs,source_sha256={str(f):sha(f) for f in (Path(__file__),H/'temporal_rgb_support.py',H/'dynamic_self_mask.py')},strictly_past=True,no_point_coordinate_change=True,not_free=True),indent=2)+'\n')
# Truth starts here, solely independent assessment.
all_e=[];all_c=[];keeps=[];scored=[]
for p in proofs:
 i=p['frame'];s=np.load(O/f'frame_{i:03d}.npz');truth=np.load(D/'frames'/f'frame_{i:03d}.npz');y,x=np.divmod(s['query_pixel_id'],640);z=truth['left_depth_m'][y,x];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))));T=truth['left_camera_usd_world'];w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(axis=1);e=np.linalg.norm(s['world']-w,axis=1);e[~valid]=np.inf;c=valid&(np.linalg.norm(w[:,:2],axis=1)<=.351)&(w[:,2]>.001)&(w[:,2]<=1.001);k=s['keep'];all_e.append(e);all_c.append(c);keeps.append(k);scored.append(dict(frame=i,n=len(e),retained=int(k.sum()),retained_metric=summarize(e[k])))
e=np.concatenate(all_e);c=np.concatenate(all_c);k=np.concatenate(keeps);m=summarize(e[k]);precision=float(c[k].mean()) if k.any() else 0.
gates=dict(n=int(k.sum())>=100,metric95=m['within_0p10m'] is not None and m['within_0p10m']>=.95,p90=m['finite_p90_m'] is not None and m['finite_p90_m']<=.1,class95=precision>=.95,two100point_exposures=sum(p['retained']>=100 for p in proofs)>=2)
r=dict(decision='PASS_DEVELOPMENT_ONLY' if all(gates.values()) else 'REJECT_TEMPORAL_POSITIVE_SUPPORT',gates=gates,original_n=len(e),retained_n=int(k.sum()),retention_fraction=float(k.mean()),original_metric=summarize(e),retained_metric=m,unverified_metric=summarize(e[~k]),retained_obstacle_precision=precision,frames=scored,seen_development_not_fresh=True,no_online_or_navigation_claim=True,missing_evidence_is_unknown=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({key:value for key,value in r.items() if key!='frames'},indent=2))
