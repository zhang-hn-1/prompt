from pathlib import Path
import json,hashlib
import numpy as np
from right_temporal_observability import triangulate
from audit_stereo import summarize
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'native_goal_camera_observation_run_v1';O=H/'right_temporal_observability_cause_v1'
seal=json.loads((O/'pretruth_seal.json').read_text())
for group in ('input_sha256','output_sha256','source_sha256'):
 for f,sha in seal[group].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha,f
counts=json.loads((O/'attrition.json').read_text())
old=json.loads((H/'right_temporal_observability_preflight_v1/attrition.json').read_text());assert old==counts
# Reconstructed same78candidate tracks sealed above; GT only cause diagnosis.
scored=[];n=0;true_accepted=0;correct_pairs=0;pixel_errors=[];world_pair_errors=[]
for row in counts:
 i=row['frame'];current=np.load(D/'frames'/f'frame_{i:03d}.npz')
 for pair in json.loads((O/f'pair_proofs_{i:03d}.json').read_text()):
  j=pair['past_frame'];a=np.array(pair['uv_past'],float).reshape(-1,2);b=np.array(pair['uv_current'],float).reshape(-1,2)
  if not len(a):continue
  past=np.load(D/'frames'/f'frame_{j:03d}.npz');A=past['right_camera_usd_world'].copy();B=current['right_camera_usd_world'].copy();A[:3,:3]=A[:3,:3]@np.diag([1,-1,-1]);B[:3,:3]=B[:3,:3]@np.diag([1,-1,-1]);w,ok=triangulate(a,A,b,B);n+=len(a);true_accepted+=int(ok.sum())
  def actual_world(uv,frame,T):
   xy=np.rint(uv).astype(int);z=frame['right_depth_m'][xy[:,1],xy[:,0]];r=np.c_[(uv-[319.5,239.5])/FOCAL,np.ones(len(uv))];return T[:3,3]+(r*z[:,None])@T[:3,:3].T
  wa=actual_world(a,past,A);wb=actual_world(b,current,B);err=np.linalg.norm(wa-wb,axis=1);err[~np.isfinite(err)]=np.inf;local=(wa-B[:3,3])@B[:3,:3];proj=local[:,:2]/local[:,2:3]*FOCAL+[319.5,239.5];pix=np.linalg.norm(proj-b,axis=1);correct=np.isfinite(err)&(err<=.1);correct_pairs+=int(correct.sum());world_pair_errors.extend(err.tolist());pixel_errors.extend(pix[np.isfinite(pix)].tolist());scored.append(dict(current=i,past=j,n=len(a),GT_pose_accepted=int(ok.sum()),world_pair_within10cm=int(correct.sum())))
r=dict(candidate_tracks=n,native_pose_accepted=0,GT_pose_accepted=true_accepted,world_pair_within10cm=correct_pairs,world_pair_metric=summarize(np.array(world_pair_errors)),reprojection_finite_labels=len(pixel_errors),reprojection_insufficient_labels=n-len(pixel_errors),reprojection_pixel_median=float(np.median(pixel_errors)) if pixel_errors else None,reprojection_pixel_p90=float(np.percentile(pixel_errors,90)) if pixel_errors else None,frames=scored,only_posthoc_cause=True,no_new_candidate_or_gate=True,whole_research_complete=False)
(O/'cause_assessment_v2.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='frames'},indent=2))
