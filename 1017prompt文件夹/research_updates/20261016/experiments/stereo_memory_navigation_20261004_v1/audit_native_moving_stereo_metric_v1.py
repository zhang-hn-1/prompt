from pathlib import Path
import json,hashlib,datetime,numpy as np
from audit_stereo import summarize
from stereo_geometry import FOCAL,BASELINE
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'native_moving_stereo_metric_audit_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();protocol=json.loads((H/'native_flat_pose_navigation_protocol_v1.json').read_text());manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for group in [protocol['source_sha256'],manifest]:
 for f,s in group.items():assert sha(f)==s,f
staticfile=H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz';static=np.load(staticfile);predfiles=sorted((D/'source_predictions').glob('*.npz'));assert len(predfiles)==121
(O/'prediction_inputs_seal_before_labels.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),predictions={str(f):sha(f) for f in predfiles},mask_sha256=sha(staticfile),old_audit_definition_sha256=sha(H/'audit_stereo.py'),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-08-native-moving-stereo-metric-source-audit.md'),code_sha256=sha(__file__),offlineassessment_of_saved_online_predictions=True,no_inference_or_fit=True,not_fresh_heldout=True),indent=2)+'\n')
y,x=np.mgrid[0:480:2,0:640:2];rays=np.stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(x.shape)),axis=-1);raynorm=np.linalg.norm(rays,axis=-1);aggregate={};frames=[]
for f in predfiles:
 i=int(f.stem.split('_')[1]);prediction=np.load(f);truth=np.load(D/'offline_truth'/f.name);estimate=np.load(D/'estimated_poses'/f.name);dynamic=np.load(D/'masks'/f.name);z=truth['left_depth_m'][::2,::2];depth=prediction['depth_m'][::2,::2];T=truth['left_camera_usd_world'];GT=T[:3,3]+(rays*z[...,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>=.5)&(z<=3)&~static['left'][::2,::2]&(np.linalg.norm(GT-truth['root_position_world_m'],axis=-1)>=.55)
 vr=y-FOCAL*BASELINE/np.maximum(z,1e-9);iv=np.rint(np.nan_to_num(vr,nan=-1,posinf=-1,neginf=-1)).astype(int);clip=np.clip(iv,0,479);rz=truth['right_depth_m'][clip,x];overlap=(vr>=0)&(vr<=479)&~static['right'][clip,x]&np.isfinite(rz)&(np.abs(rz-z)<=.03+.01*z)
 err=np.abs(depth-z)*raynorm;err=np.where(np.isfinite(err),err,np.inf);E=estimate['left_camera_usd_world'];W=E[:3,3]+(rays*depth[...,None])@(E[:3,:3]@np.diag([1,-1,-1])).T;we=np.linalg.norm(W-GT,axis=-1);we=np.where(np.isfinite(we),we,np.inf)
 body=valid&(GT[...,2]>=.05)&(GT[...,2]<=1.5);floor=valid&(np.abs(GT[...,2])<=.05);dyn=~dynamic['left'][::2,::2]&~dynamic['right'][clip,x]
 groups=dict(body_all=body,body_overlap=body&overlap,floor_all=floor,floor_overlap=floor&overlap,secondary_dynamic_body_overlap=body&overlap&dyn)
 scores={}
 for name,selection in groups.items():
  for quantity,values in [('ray_metric',err),('native_world_metric',we)]:
   key=name+'_'+quantity;sample=values[selection];aggregate.setdefault(key,[]).append(sample);scores[key]=summarize(sample)
 frames.append(dict(frame=i,time_s=float(prediction['time_s']),groups=scores));print('PAIR_ASSESSED',i,flush=True)
result={k:summarize(np.concatenate(v)) for k,v in aggregate.items()};p=result['body_overlap_ray_metric'];metricpass=p['n']>=100 and p['within_0p10m']>=.9 and p['finite_p90_m'] is not None and p['finite_p90_m']<=.1
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_CURRENT_MOVING_VISIBLE_BODY_METRIC_ONLY' if metricpass else 'REJECT_CURRENT_MOVING_RAW_METRIC_SOURCE',groups=result,frames=frames,actual_source_pairs=121,primary_gate='same strict visiblebodyoverlap100/90%within.10/p90<=.10; secondary notsubstitution',not_full_blind_volume_coverage=True,not_observed_free_or_full3Dsafety=True,not_fresh_heldout=True,not_new_online_run=True,no_fit_or_networkchange=True,old_source_rejects_preserved=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='frames'},indent=2))
