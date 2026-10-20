"""Analytic fixture + stored renderer labels; GT postclose causal diagnosis only."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scene_ray_geometry import cylinder_depth,camera_rays
H=Path(__file__).resolve().parent;D=H/'controlled_heading_probe_run_v1';O=H/'controlled_heading_probe_visibility_v1';O.mkdir(exist_ok=False)
s=json.loads((H/'controlled_heading_probe_stage_status_v1.json').read_text());assert s['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION';entry=s;seal=D/'artifact_manifest_post_close.json';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();assert sha(seal)==entry['post_close_manifest_sha256'];manifest=json.loads(seal.read_text())['artifact_sha256'];assert all(sha(f)==v for f,v in manifest.items());F=584.8888888888889;rows=[];inputs={}
for i in range(0,81,5):
 paths=[D/'offline_truth'/f'frame_{i:03d}.npz',D/'masks'/f'frame_{i:03d}.npz',D/'source_predictions'/f'frame_{i:03d}.npz']
 for p in paths:inputs[str(p)]=sha(p)
 gt=np.load(paths[0]);mask=np.load(paths[1]);pred=np.load(paths[2]);eyes=[]
 for eye,other in [('left','right'),('right','left')]:
  T=gt[eye+'_camera_usd_world'];origin,rays,yy,xx=camera_rays(T,480,640,F,319.5,239.5,stride=1);expected=cylinder_depth(origin,rays,np.array([0.,0.,.5]),np.eye(3),.35,1.);z=gt[eye+'_depth_m']
  analytic=np.isfinite(expected);visible=analytic&np.isfinite(z)&(np.abs(z-expected)*np.linalg.norm(rays,axis=-1)<=.02)
  world=origin+np.where(visible,z,0)[...,None]*rays
  domain=visible&(z>=.5)&(z<=3)&(world[...,2]>.05)&(world[...,2]<=1.5)&(np.linalg.norm(world-gt['root_position_world_m'],axis=-1)>.55)
  eligible=domain&~mask[eye];w=world[eligible];ot=gt[other+'_camera_usd_world'];R=ot[:3,:3]@np.diag([1,-1,-1]);local=(w-ot[:3,3])@R
  with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*F+[319.5,239.5]
  fov=np.isfinite(uv).all(1)&(local[:,2]>0)&(uv[:,0]>=0)&(uv[:,0]<639.5)&(uv[:,1]>=0)&(uv[:,1]<479.5);inside=np.flatnonzero(fov);same=np.zeros(len(w),bool);cad=np.zeros(len(w),bool)
  if len(inside):
   u,v=np.rint(uv[inside]).astype(int).T;cad[inside]=mask[other][v,u];oz=gt[other+'_depth_m'][v,u];orr=np.c_[(u-319.5)/F,(v-239.5)/F,np.ones(len(u))]
   with np.errstate(invalid='ignore'):ow=ot[:3,3]+(orr*oz[:,None])@R.T;error=np.linalg.norm(ow-w[inside],axis=1)
   same[inside]=~cad[inside]&np.isfinite(error)&(error<=.1)
  confidence=pred['confidence'][eligible] if eye=='left' else np.zeros(len(w),bool)
  confident_in_range=confidence&np.isfinite(pred['depth_m'][eligible])&(pred['depth_m'][eligible]>=.5)&(pred['depth_m'][eligible]<=3) if eye=='left' else np.zeros(len(w),bool)
  eyes.append(dict(eye=eye,analytic_cylinder_FOV_pixels=int(analytic.sum()),renderer_visible_cylinder_pixels=int(visible.sum()),analytic_but_unresolved_occluded=int((analytic&~visible).sum()),visible_in_old_source_domain=int(domain.sum()),visible_domain_current_CAD_blocked=int((domain&mask[eye]).sum()),visible_domain_current_CAD_unmasked=int(eligible.sum()),other_FOV_out=int((~fov).sum()),other_CAD_blocked=int(cad.sum()),mutually_unmasked_same_surface=int(same.sum()),model_confident_in_range_mutual_left=int((same&confident_in_range).sum()) if eye=='left' else None))
  np.savez_compressed(O/f'{i:03d}_{eye}_diagnostic_masks.npz',expected_cylinder_fov=analytic,renderer_cylinder_visible=visible,current_CAD_mask=mask[eye],source_domain_visible=domain,time_s=float(gt['time_s']),GT_only_posthoc=True)
 rows.append(dict(frame=i,time_s=float(gt['time_s']),eyes=eyes))
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),GT_only_posthoc=True,online_inputs_not_changed=True,model_gate_not_rescored=True,fixture_task_NOT_FORMAL=True,old_reject_preserved=True,rows=rows,input_sha256=inputs,script_sha256=sha(__file__),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-07-controlled-heading-source-causal-probe.md'))
(O/'assessment_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(rows,indent=2))
