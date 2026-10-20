"""Four postseal oracle-semantics information bounds, NOT an RGB actor/free map."""
from pathlib import Path
import json,hashlib,numpy as np
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';queries=[116,216,221,331]
protocol=dict(queries=queries,maximum_queries=4,role='GT-visible floor upper bound on possible RGB-floor plus plane-prior source; not implemented classifier/actor',GT_semantics_only_postseal=True,memory='best-case unlimited causal past visible-floor memory, deliberately optimistic information bound not actualTTL10memory',ground_pixels='GT valid world|z|<=.01; evaluator not actor input',no_new_physics_NN_fit=True,no_free_volume_certificate=True)
(H/'causal_floor_visibility_upper_bound_protocol_v1.json').write_text(json.dumps(protocol,indent=2)+'\n')
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s
first=np.load(D/'maps/frame_000.npz');origin=first['origin'];res=float(first['resolution']);shape=first['labels'].shape;seen=np.zeros(shape,bool);initial_prior=first['safe_start_prior'].astype(bool);Y,X=np.indices((480,640));rays=np.stack(((X-319.5)/FOCAL,(Y-239.5)/FOCAL,np.ones_like(X)),axis=-1);exposures=json.loads((D/'source_rows.json').read_text());cursor=0;out=[]
offsets=np.array([(i,j) for i in range(-6,7) for j in range(-6,7) if np.hypot(i,j)*res<=.6+1e-8])
for cutoff in queries:
 while cursor<len(exposures) and exposures[cursor]['frame']<=cutoff:
  i=exposures[cursor]['frame'];name=f'frame_{i:03d}.npz';gt=np.load(D/'frames'/name);pose=np.load(D/'known_poses'/name)
  for eye in ['left','right']:
   T=pose[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);dep=gt[eye+'_depth_m'];valid=np.isfinite(dep)&(dep>0)
   with np.errstate(invalid='ignore'):world=T[:3,3]+(rays*dep[...,None])@R.T
   floor=valid&np.isfinite(world).all(-1)&(np.abs(world[...,2])<=.01)
   cells=np.rint((world[floor,:2]-origin)/res).astype(int);inside=(cells>=0).all(1)&(cells<shape).all(1);cells=cells[inside];seen[tuple(cells.T)]=True
  cursor+=1
 plan=json.loads((D/'local_plans'/f'frame_{cutoff:03d}.json').read_text());q=np.array([[w['x_m'],w['y_m']] for w in plan['waypoints']]);points=[];remaining=1.
 for a,b in zip(q[:-1],q[1:]):
  length=np.linalg.norm(b-a)
  if length<=0:continue
  portion=min(remaining,length);end=a+(b-a)*portion/length;points.append(np.linspace(a,end,max(2,int(np.ceil(portion/.05))+1)));remaining-=portion
  if remaining<=1e-8:break
 centres=np.rint((np.vstack(points)-origin)/res).astype(int);foot=np.unique((centres[:,None,:]+offsets[None,:,:]).reshape(-1,2),axis=0);inside=(foot>=0).all(1)&(foot<shape).all(1);covered=np.zeros(len(foot),bool);prior=covered.copy();covered[inside]=seen[tuple(foot[inside].T)];prior[inside]=initial_prior[tuple(foot[inside].T)]
 out.append(dict(frame=cutoff,reference_version=plan['reference_version'],last_floor_source_frame=exposures[cursor-1]['frame'],max_source_time_s=exposures[cursor-1]['time_s'],query_time_s=cutoff*.1,past_exposures=cursor,first_nominal1m_footprint_cells=len(foot),visible_floor_history_cells=int(covered.sum()),visible_floor_history_fraction=float(covered.mean()),initial_safe_prior_only_cells=int((prior&~covered).sum()),visible_or_explicit_initial_prior_fraction=float((covered|prior).mean()),still_unknown_cells=int((~(covered|prior)).sum())))
r=dict(protocol=protocol,rows=out,perfect_semantic_floor_assumed=True,not_RGB_classifier_validation=True,not_actual_actor_or_free_space=True,not_complete3D_or_stopping_certificate=True,prior_explicitly_separate=True,no_future_exposure_read=True,whole_research_complete=False)
(H/'causal_floor_visibility_upper_bound_assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
