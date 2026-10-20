"""Offline visibility upper bound. NO RGB matcher or action outputs."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'temporal_surface_observability_v1';assert not D.exists();D.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();plan=H.parent.parent/'docs/superpowers/plans/2026-10-07-temporal-surface-observability-audit.md';inputs={};history=[];rows=[]
def load(p):inputs[str(p.resolve())]=sha(p);return np.load(p)
for f in sorted((O/'sensor_packets').glob('*.npz')):
 src=load(f);i=int(src['frame']);t=float(src['time_s']);truth=load(O/'offline_truth'/f'frame_{i:03d}.npz');mask=load(O/'masks'/f'frame_{i:03d}.npz');eyes=[]
 for eye in ['left','right']:
  yy,xx=np.mgrid[0:480:2,0:640:2];z=truth[eye+'_depth_m'][yy,xx];T=truth[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);ray=np.stack([(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones_like(xx)],axis=-1)
  with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,:,None])@R.T
  valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask[eye][yy,xx]&(world[:,:,2]>.05)&(world[:,:,2]<=1.5)&(np.linalg.norm(world-truth['root_position_world_m'],axis=-1)>.55);w=world[valid];seen=np.zeros(len(w),bool);geom=np.zeros(len(w),bool);views=[]
  for oldi,oldt,oldtruth,oldmask in history[-4:]:
   assert oldt<t
   if t-oldt>2.+1e-9:continue
   P=oldtruth[eye+'_camera_usd_world'];Q=P[:3,:3]@np.diag([1,-1,-1]);loc=(w-P[:3,3])@Q;baseline=float(np.linalg.norm(P[:3,3]-T[:3,3]))
   with np.errstate(divide='ignore',invalid='ignore'):uv=loc[:,:2]/loc[:,2,None]*584.8888888888889+[319.5,239.5]
   fov=(loc[:,2]>0)&np.isfinite(uv).all(1)&(uv[:,0]>=0)&(uv[:,0]<639.5)&(uv[:,1]>=0)&(uv[:,1]<479.5);q=np.flatnonzero(fov);x,y=np.rint(uv[q]).astype(int).T;oz=oldtruth[eye+'_depth_m'][y,x];rays=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))];same=np.zeros(len(w),bool)
   with np.errstate(invalid='ignore'):ow=P[:3,3]+(rays*oz[:,None])@Q.T;err=np.linalg.norm(ow-w[q],axis=1)
   same[q]=~oldmask[eye][y,x]&np.isfinite(err)&(err<=.1)
   va=w-T[:3,3];vb=w-P[:3,3]
   with np.errstate(divide='ignore',invalid='ignore'):angle=np.degrees(np.arccos(np.clip(np.sum(va*vb,axis=1)/(np.linalg.norm(va,axis=1)*np.linalg.norm(vb,axis=1)),-1,1)))
   good=same&(angle>=.5)&(baseline>=.05);seen|=same;geom|=good;views.append(dict(source_frame=oldi,source_time_s=oldt,baseline_m=baseline,past_fov=int(fov.sum()),same_surface=int(same.sum()),adequate_baseline_parallax=int(good.sum())))
  eyes.append(dict(eye=eye,current_visible_hazard=len(w),any_past_same_surface=int(seen.sum()),any_past_adequate_baseline_parallax=int(geom.sum()),views=views))
 rows.append(dict(frame=i,time_s=t,eyes=eyes));history.append((i,t,truth,mask))
f=D/'assessment_v1.json';f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),plan_sha256=sha(plan),code_sha256=sha(__file__),input_sha256=inputs,rows=rows,GT_only_offline_upper_bound=True,no_correspondences_or_labels_sent_to_predictor=True,no_actual_new_model_or_control=True,knownpose_privileged=True,development_not_fresh=True,old_rejects_preserved=True),indent=2)+'\n');print([(r['frame'],[e['any_past_adequate_baseline_parallax'] for e in r['eyes']]) for r in rows if r['frame']>=180])
