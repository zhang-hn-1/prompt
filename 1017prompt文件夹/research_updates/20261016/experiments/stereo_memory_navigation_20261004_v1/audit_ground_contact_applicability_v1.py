from pathlib import Path
import json,hashlib,datetime
import numpy as np
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from ground_contact_hypothesis import predict
from audit_stereo import summarize
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'native_goal_camera_observation_run_v1';O=H/'ground_contact_applicability_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for f,s in manifest.items():assert sha(f)==s,f
rows=json.loads((D/'source_rows.json').read_text());cad=RobotSelfMask();meta=[];inputs={}
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';files=[D/'partial_class_masks'/name,D/'estimated_poses'/name,D/'history'/name,D/'sensor_packets'/f'source_{i:03d}.npz']
 for f in files:inputs[str(f)]=sha(f)
 cls=np.load(files[0]);poses=np.load(files[1]);masks=cad.project(joints_from_proprio(np.load(files[2])['proprio_history']));t=float(np.load(files[3])['time_s']);assert row['time_s']==t if 'time_s' in row else True
 for eye in ('left','right'):
  T=poses[eye+'_camera_usd_world'].copy();T[:3,:3]=T[:3,:3]@np.diag([1,-1,-1]);p=predict(cls[eye],masks[eye],T)
  np.savez_compressed(O/(eye+'_'+name),uv=p['uv'],world=p['world'],frame=i,time_s=t,label=p['label'],certified_free=False,measured_stereo=False)
  meta.append(dict(frame=i,eye=eye,n=len(p['uv']),time_s=t,reason=p['reason']))
(O/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n');outputs={str(f):sha(f) for f in O.iterdir()};(O/'pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256=outputs,source_sha256={str(f):sha(f) for f in (Path(__file__),H/'ground_contact_hypothesis.py',H/'dynamic_self_mask.py')},floor_prior_z0=True,grounded_contact_unproved=True,not_admitted_observation=True,no_fixture_geometry_filter=True),indent=2)+'\n')
# GT after seal: conditional applicability only.
scored=[];arms={eye:dict(errors=[],classes=[],ground=[],rim_visible=[]) for eye in ('left','right')}
angle=np.arange(360)*np.pi/180.;rim=np.c_[.35*np.cos(angle),.35*np.sin(angle),np.full(360,.005)]
for row in meta:
 i=row['frame'];eye=row['eye'];s=np.load(O/(eye+f'_frame_{i:03d}.npz'));truth=np.load(D/'frames'/f'frame_{i:03d}.npz');uv=s['uv'];x,y=uv[:,0],uv[:,1];depth=truth[eye+'_depth_m'][y,x];T=truth[eye+'_camera_usd_world'].copy();T[:3,:3]=T[:3,:3]@np.diag([1,-1,-1]);rays=np.c_[(uv-[319.5,239.5])/FOCAL,np.ones(len(uv))];w=T[:3,3]+(rays*depth[:,None])@T[:3,:3].T;valid=np.isfinite(depth)&(depth>0)&np.isfinite(w).all(1);e=np.linalg.norm(s['world']-w,axis=1);e[~valid]=np.inf;cl=valid&(np.linalg.norm(w[:,:2],axis=1)<=.351)&(w[:,2]>.001)&(w[:,2]<=1.001);ground=valid&(np.abs(w[:,2])<=.1)
 q=(rim-T[:3,3])@T[:3,:3];v=q[:,:2]/q[:,2:3]*FOCAL+[319.5,239.5];inside=(q[:,2]>0)&(v[:,0]>=0)&(v[:,0]<=639)&(v[:,1]>=0)&(v[:,1]<=479);ids=np.flatnonzero(inside);pix=np.rint(v[ids]).astype(int);z=truth[eye+'_depth_m'][pix[:,1],pix[:,0]];vis=np.isfinite(z)&(np.abs(z-q[ids,2])<=.1)
 a=arms[eye];a['errors'].append(e);a['classes'].append(cl);a['ground'].append(ground);a['rim_visible'].append(int(vis.sum()));scored.append(dict(frame=i,eye=eye,n=len(e),metric=summarize(e),class_precision=float(cl.mean()) if len(cl) else None,near_floor_fraction=float(ground.mean()) if len(ground) else None,rim_FOV_samples=int(inside.sum()),rim_visible_samples=int(vis.sum())))
results={}
for eye,a in arms.items():
 e=np.concatenate(a['errors']);cl=np.concatenate(a['classes']);ground=np.concatenate(a['ground']);m=summarize(e);p=float(cl.mean()) if len(cl) else 0.;g=dict(n=len(e)>=100,metric95=m['within_0p10m'] is not None and m['within_0p10m']>=.95,p90=m['finite_p90_m'] is not None and m['finite_p90_m']<=.1,class95=p>=.95,two100_exposures=sum(r['n']>=100 for r in meta if r['eye']==eye)>=2);results[eye]=dict(diagnostic_decision='PASS_CONDITIONAL_APPLICABILITY_ONLY' if all(g.values()) else 'REJECT_CONDITIONAL_APPLICABILITY',gates=g,n=len(e),metric=m,class_precision=p,near_floor_fraction=float(ground.mean()) if len(ground) else None,rim_visible_exposures=sum(n>0 for n in a['rim_visible']))
r=dict(arms=results,frames=scored,typed_model_hypothesis=True,extra_grounded_object_interpretation_not_proved=True,not_online_or_stereo_source_admission=True,no_free_or_volume_certificate=True,observer_prior_and_source_reject_preserved=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps(results,indent=2))
