"""Posthoc point attribution only; substituted points never reach an actor."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from audit_stereo import summarize
H=Path(__file__).resolve().parent;D=H/'native_reverse_bank_observation_run_v1';O=H/'reverse_source_pose_depth_attribution_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=json.loads((D/'source_rows.json').read_text());original=[];substituted=[];shift=[];depth_error=[];inputs={}
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz'
 files=[D/'partial_observations'/name,D/'estimated_poses'/name,D/'frames'/name]
 for f in files:inputs[str(f)]=sha(f)
 s=np.load(files[0]);est=np.load(files[1]);gt=np.load(files[2]);w=s['world'];y,x=np.divmod(s['query_pixel_id'],640)
 A=est['left_camera_usd_world'];RA=A[:3,:3]@np.diag([1,-1,-1]);B=gt['left_camera_usd_world'];RB=B[:3,:3]@np.diag([1,-1,-1])
 local=(w-A[:3,3])@RA;pose_sub=B[:3,3]+local@RB.T
 z=gt['left_depth_m'][y,x];rays=np.column_stack(((x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))))
 truth=B[:3,3]+(rays*z[:,None])@RB.T;valid=np.isfinite(z)&(z>0)&np.isfinite(truth).all(1)
 e=np.linalg.norm(w-truth,axis=1);es=np.linalg.norm(pose_sub-truth,axis=1);e[~valid]=np.inf;es[~valid]=np.inf
 original.append(e);substituted.append(es);shift.append(np.linalg.norm(pose_sub-w,axis=1));depth_error.append(local[:,2]-z)
e=np.concatenate(original);s=np.concatenate(substituted);delta=np.concatenate(shift);de=np.concatenate(depth_error)
np.savez_compressed(O/'errors_GT_ONLY_NO_ACTOR.npz',original_error_m=e,true_pose_substitution_error_m=s,point_pose_shift_m=delta,predicted_minus_true_camera_z_m=de)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='GT_ONLY_POSTHOC_CAUSE_NOT_REPAIRED_SOURCE',n=len(e),original_metric=summarize(e),true_pose_substitution_metric=summarize(s),pose_shift_median_m=float(np.median(delta)),pose_shift_p90_m=float(np.percentile(delta,90)),signed_depth_error_median_m=float(np.median(de)),old_raw_source_reject_preserved=True,no_actor_points_saved=True,no_filtered_rescore=True,no_calibration_or_depth_bias_correction=True,not_new_gate=True,whole_research_complete=False,input_sha256=inputs,code_sha256=sha(__file__))
(O/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='input_sha256'},indent=2))
