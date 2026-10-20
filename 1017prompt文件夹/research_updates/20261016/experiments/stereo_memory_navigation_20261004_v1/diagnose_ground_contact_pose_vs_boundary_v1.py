from pathlib import Path
import json,hashlib
import numpy as np
from audit_stereo import summarize
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'native_goal_camera_observation_run_v1';O=H/'ground_contact_applicability_v1';seal=json.loads((O/'pretruth_seal.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for group in ('input_sha256','output_sha256','source_sha256'):
 for f,s in seal[group].items():assert sha(f)==s,f
arms={eye:dict(native=[],true_pose=[],height=[],classes=[]) for eye in ('left','right')}
for row in json.loads((O/'metadata.json').read_text()):
 i=row['frame'];eye=row['eye'];s=np.load(O/(eye+f'_frame_{i:03d}.npz'));uv=s['uv'];truth=np.load(D/'frames'/f'frame_{i:03d}.npz');T=truth[eye+'_camera_usd_world'].copy();T[:3,:3]=T[:3,:3]@np.diag([1,-1,-1]);rays=np.c_[(uv-[319.5,239.5])/FOCAL,np.ones(len(uv))];direction=rays@T[:3,:3].T;x,y=uv[:,0],uv[:,1];z=truth[eye+'_depth_m'][y,x];w=T[:3,3]+direction*z[:,None];valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(1)
 with np.errstate(divide='ignore',invalid='ignore'):depth=-T[2,3]/direction[:,2];plane=T[:3,3]+direction*depth[:,None]
 e=np.linalg.norm(s['world']-w,axis=1);e[~valid]=np.inf;eg=np.linalg.norm(plane-w,axis=1);eg[~valid|~np.isfinite(depth)|(depth<.5)|(depth>3)]=np.inf;a=arms[eye];a['native'].append(e);a['true_pose'].append(eg);a['height'].append(np.abs(w[:,2]));a['classes'].append(valid&(np.linalg.norm(w[:,:2],axis=1)<=.351)&(w[:,2]>.001)&(w[:,2]<=1.001))
result={}
for eye,a in arms.items():
 e=np.concatenate(a['native']);eg=np.concatenate(a['true_pose']);height=np.concatenate(a['height']);c=np.concatenate(a['classes']);result[eye]=dict(n=len(e),original_native_metric=summarize(e),GT_pose_only_same_pixel_metric=summarize(eg),original_selected_pixel_abs_height_p90_m=float(np.percentile(height[np.isfinite(height)],90)) if np.isfinite(height).any() else None,finite_height_labels=int(np.isfinite(height).sum()),true_obstacle_pixels=int(c.sum()),near_floor_pixels=int((height<=.1).sum()))
r=dict(arms=result,posthoc_cause_only=True,unchanged_pixel_selection=True,GT_pose_never_actor_input=True,no_original_gate_rescore=True,no_inference_or_control_change=True,source_assessment_sha256=sha(O/'assessment.json'),source_code_sha256=sha(__file__))
f=O/'pose_vs_boundary_cause_v1.json';assert not f.exists();f.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps(r,indent=2))
