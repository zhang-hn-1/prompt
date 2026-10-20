"""Authored scene primitives only as independent offline identity evidence."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from scene_ray_geometry import cylinder_depth
from pxr import Usd,UsdGeom,UsdPhysics
H=Path(__file__).resolve().parent;D=H/'source_event_handoff_C1_run_v1';A=H/'near_field_static_ownership_v1';A.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();scene=Path('/home/an/project/NavDP/assets/scenes/benchmark_e1/e1_detour.usda');task=json.loads((D/'task_episode.json').read_text());assert sha(scene)==task['actual_episode']['scene_sha256'];stage=Usd.Stage.Open(str(scene));cache=UsdGeom.XformCache();assert UsdGeom.GetStageMetersPerUnit(stage)==1 and UsdGeom.GetStageUpAxis(stage)=='Z';shapes=[]
for p in stage.Traverse():
 assert not p.HasAPI(UsdPhysics.RigidBodyAPI) and not any(a.GetTimeSamples() for a in p.GetAttributes())
 if not p.IsA(UsdGeom.Gprim) or UsdGeom.Imageable(p).ComputeVisibility()=='invisible':continue
 assert p.IsA(UsdGeom.Cylinder),'do not silently omit another static geometry'
 c=UsdGeom.Cylinder(p);assert c.GetAxisAttr().Get()=='Z';T=np.asarray(cache.GetLocalToWorldTransform(p)).T;R=T[:3,:3];assert np.allclose(R.T@R,np.eye(3));shapes.append(dict(path=str(p.GetPath()),centre=T[:3,3].tolist(),rotation=R.tolist(),radius=float(c.GetRadiusAttr().Get()),height=float(c.GetHeightAttr().Get())))
(A/'scene_geometry_v1.json').write_text(json.dumps(dict(scene=str(scene),scene_sha256=sha(scene),shapes=shapes,GT_offline_only=True),indent=2)+'\n')
previous=json.loads((H/'qualified_appearance_later_motion_v1/assessment_v1.json').read_text());yy,xx=np.mgrid[:480,:640];F=584.8888888888889;localray=np.stack([(xx-319.5)/F,(yy-239.5)/F,np.ones_like(xx)],-1);rows=[];totals={}
for row in previous['all_historical_motion_rows']:
 if row['pass_row']:continue
 i=row['frame'];eye=row['eye'];predp=H/'qualified_appearance_later_motion_v1'/f'{eye}_{i:03d}.npz';p=np.load(predp)['potential_hazard_mask'];gt=np.load(D/'offline_truth'/f'frame_{i:03d}.npz');z=gt[eye+'_depth_m'][p];T=gt[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);rays=localray[p]@R.T;origin=T[:3,3]
 with np.errstate(invalid='ignore'):world=origin+rays*z[:,None]
 finite=np.isfinite(world).all(1);near=np.linalg.norm(world-gt['root_position_world_m'],axis=1)<=.55;height=(world[:,2]>.05)&(world[:,2]<=1.5);rangeok=(z>=.5)&(z<=3);expected=np.full(len(world),np.inf);nearest_boundary=np.full(len(world),np.inf)
 for s in shapes:
  center=np.array(s['centre']);rot=np.array(s['rotation']);loc=(world-center)@rot;q=np.c_[np.linalg.norm(loc[:,:2],axis=1)-s['radius'],np.abs(loc[:,2])-s['height']/2];sdf=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0);nearest_boundary=np.minimum(nearest_boundary,np.abs(sdf));expected=np.minimum(expected,cylinder_depth(origin,rays,center,rot,s['radius'],s['height']))
 error=np.abs(expected-z)*np.linalg.norm(rays,axis=1);consistent=finite&np.isfinite(expected)&(error<=.02)&(nearest_boundary<=.02);cats={};remaining=np.ones(len(world),bool)
 for name,condition in [('insufficient_GT',~finite),('within_root_055',near),('outside_height_domain',~height),('physical_hazard_outside_axial_domain',~rangeok),('correct_old_domain',np.ones(len(world),bool))]:
  select=remaining&condition;remaining[select]=False;v=dict(n=int(select.sum()),static_cylinder_consistent=int((select&consistent).sum()),unresolved=int((select&~consistent).sum()),finite_ray_world_error_p90_m=float(np.quantile(error[select&np.isfinite(error)],.9)) if (select&np.isfinite(error)).any() else None);cats[name]=v
  if name not in totals:totals[name]=dict(n=0,static_cylinder_consistent=0,unresolved=0)
  for k in totals[name]:totals[name][k]+=v[k]
 assert sum(c['n'] for c in cats.values())==row['predicted_pixels'];rows.append(dict(frame=i,eye=eye,categories=cats))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),all_failed_pixels_included=True,static_scene_cylinders=len(shapes),geometry_tolerance_m=.02,GT_only_posthoc=True,unmatched_UNRESOLVED_not_robot=True,old_REJECT_unchanged=True,no_gate_or_model_rescore=True,totals=totals,rows=rows,source_sha256={str(x):sha(x) for x in [scene,Path(__file__),H/'scene_ray_geometry.py',H.parent.parent/'docs/superpowers/plans/2026-10-07-near-field-surface-ownership.md']})
(A/'assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(totals,indent=2))
