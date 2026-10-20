"""Sealed GT-only all-native ownership at ten declared control stages."""
from pathlib import Path
import datetime,json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;D=H/'source_event_handoff_C1_run_v1';A=H/'source_event_collision_relevant_coverage_v1';A.mkdir(exist_ok=False);assessment=json.loads((H/'source_event_handoff_assessment_v1.json').read_text());assert assessment['arms'][1]['decision']=='REJECT_LOCAL_CONTROL_CANDIDATE';states=json.loads((D/'states.json').read_text());collision_xy=np.asarray(states[-1]['position'][:2]);F=584.8888888888889;yy,xx=np.mgrid[0:480,0:640];base=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');rows=[];inputs={};sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
for i in [0,60,120,145,150,180,245,250,260,265]:
 truthp=D/'offline_truth'/f'frame_{i:03d}.npz';maskp=D/'masks'/f'frame_{i:03d}.npz';predp=D/'source_predictions'/f'frame_{i:03d}.npz'
 for p in [truthp,maskp,predp]:inputs[str(p)]=sha(p)
 gt=np.load(truthp);mask=np.load(maskp);pred=np.load(predp);z=gt['left_depth_m'];T=gt['left_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);ray=np.stack([(xx-319.5)/F,(yy-239.5)/F,np.ones_like(xx)],axis=-1)
 with np.errstate(invalid='ignore'):w=T[:3,3]+(ray*z[:,:,None])@R.T
 valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask['left']&(w[:,:,2]>.05)&(w[:,:,2]<=1.5)&(np.linalg.norm(w-gt['root_position_world_m'],axis=-1)>.55)
 world=w[valid];lx=xx[valid];ly=yy[valid];T=gt['right_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);local=(world-T[:3,3])@R
 with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*F+[319.5,239.5]
 positive=(local[:,2]>0)&np.isfinite(uv).all(1);fov=positive&(uv[:,0]>=0)&(uv[:,0]<639.5)&(uv[:,1]>=0)&(uv[:,1]<479.5);ix=np.flatnonzero(fov);u,v=np.rint(uv[ix]).astype(int).T;blocked=np.zeros(len(world),bool);fixed=np.zeros(len(world),bool);common=np.zeros(len(world),bool);blocked[ix]=mask['right'][v,u];fixed[ix]=base['right'][v,u];rz=gt['right_depth_m'][v,u];rr=np.c_[(u-319.5)/F,(v-239.5)/F,np.ones(len(u))]
 with np.errstate(invalid='ignore'):rw=T[:3,3]+(rr*rz[:,None])@R.T;err=np.linalg.norm(rw-world[ix],axis=1)
 common[ix]=~blocked[ix]&np.isfinite(err)&(err<=.1)
 relevant=np.linalg.norm(world[:,:2]-collision_xy,axis=1)<=.6
 confident=pred['confidence'][ly,lx].astype(bool)&np.isfinite(pred['depth_m'][ly,lx])&(pred['depth_m'][ly,lx]>=.5)&(pred['depth_m'][ly,lx]<=3)
 counts=[]
 for name,region in [('all_visible',np.ones(len(world),bool)),('collision_relevant_posthoc',relevant)]:
  counts.append(dict(region=name,left_unmasked_visible=int(region.sum()),right_FOV_out=int((region&~fov).sum()),right_fixed_base_blocked=int((region&fov&fixed).sum()),right_additional_dynamic_blocked=int((region&fov&blocked&~fixed).sum()),right_unmasked_same_surface=int((region&common).sum()),right_unmasked_mismatch_or_insufficient=int((region&fov&~blocked&~common).sum()),mutual_visible_model_confident_in_range=int((region&common&confident).sum()),mutual_visible_XY_bins=len(np.unique(np.floor(world[region&common,:2]/.1).astype(int),axis=0))))
 rows.append(dict(frame=i,time_s=float(gt['time_s']),left_axial_median_m=float(np.median(z[valid])) if valid.any() else None,counts=counts))
 # Real left view with posthoc relevant surface colored, not a made-up camera picture.
 rgb=np.load(D/'online_inputs'/f'frame_{i:03d}.npz')['left_rgb'];overlay=rgb.copy();overlay[ly[relevant],lx[relevant],:3]=(np.array([255,70,30])*.55+overlay[ly[relevant],lx[relevant],:3]*.45).astype(np.uint8)
 np.savez_compressed(A/f'frame_{i:03d}.npz',left_rgb=rgb,left_relevant_overlay=overlay,frame=i,time_s=float(gt['time_s']))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),GT_only_posthoc=True,future_collision_position_only_as_diagnostic_region=True,not_online_inputs_or_control=True,not_source_admission=True,full_native_resolution=[480,640],collision_final_xy=collision_xy.tolist(),diagnostic_radius_m=.6,rows=rows,input_sha256=inputs,code_sha256=sha(__file__),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-07-collision-relevant-view-coverage.md'),whole_research_complete=False)
(A/'coverage_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(rows,indent=2))
