"""Bounded same-run postseal labels only, no source or action alteration."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'frontal_obstacle_acquisition_run_v1';O=H/'frontal_obstacle_acquisition_sparse_assessment_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for p,d in seal['artifact_sha256'].items():assert sha(p)==d,p
source_assessment={'decision':'postseal original source gate evaluated below'}
scene=Path('/home/an/project/NavDP/assets/scenes/benchmark_e1/e1_detour.usda');stage=Usd.Stage.Open(str(scene));cyl=[]
for prim in stage.Traverse():
 if prim.IsA(UsdGeom.Cylinder) and prim.HasAPI(UsdPhysics.CollisionAPI):
  shape=UsdGeom.Cylinder(prim);M=np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
  assert np.allclose(M[:3,:3],np.eye(3)) and shape.GetAxisAttr().Get()=='Z'
  cyl.append(dict(path=str(prim.GetPath()),position=M[:3,3].tolist(),radius=float(shape.GetRadiusAttr().Get()),height=float(shape.GetHeightAttr().Get())))
assert cyl
cad=RobotSelfMask();rows=json.loads((D/'source_rows.json').read_text());scores=[];E=[];inside=[];good_visibility=[];bad=[];factor_tests=[]
for r in rows:
 i=r['frame'];name=f'frame_{i:03d}.npz';p=np.load(D/'partial_observations'/name);gt=np.load(D/'frames'/name);pred=np.load(D/'source_predictions'/name);pose=np.load(D/'known_poses'/name);hist=np.load(D/'history'/name)['proprio_history'].copy();hist[:,:9]=0.;masks=cad.project(joints_from_proprio(hist));y,x=np.divmod(p['query_pixel_id'],640);n=len(x);z=gt['left_depth_m'][y,x];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(n)));T=pose['left_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);w=T[:3,3]+(rays*z[:,None])@R.T;valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(axis=1);e=np.linalg.norm(p['world']-w,axis=1);e[~valid]=np.inf
 predicted_depth=pred['depth_m'][y,x];factor=np.abs(predicted_depth-z)*np.linalg.norm(rays,axis=1);finite=valid&np.isfinite(predicted_depth);np.testing.assert_allclose(e[finite],factor[finite],atol=1e-6,rtol=1e-6);factor_tests.append(int(finite.sum()))
 membership=np.zeros(n,bool)
 for c in cyl:
  centre=np.asarray(c['position']);membership |= valid&(np.linalg.norm(w[:,:2]-centre[:2],axis=1)<=c['radius']+.001)&(np.abs(w[:,2]-centre[2])<=c['height']/2+.001)
 RT=pose['right_camera_usd_world'];RR=RT[:3,:3]@np.diag([1,-1,-1]);local=(w-RT[:3,3])@RR;uv=np.column_stack((319.5+FOCAL*local[:,0]/local[:,2],239.5+FOCAL*local[:,1]/local[:,2]));px=np.rint(np.nan_to_num(uv,nan=-999,posinf=-999,neginf=-999)).astype(int);inframe=valid&(local[:,2]>0)&(px[:,0]>=0)&(px[:,0]<640)&(px[:,1]>=0)&(px[:,1]<480);clipx=np.clip(px[:,0],0,639);clipy=np.clip(px[:,1],0,479);rz=gt['right_depth_m'][clipy,clipx];vis=inframe&~masks['left'][y,x]&~masks['right'][clipy,clipx]&np.isfinite(rz)&(np.abs(rz-local[:,2])<=.03+.01*local[:,2]);isbad=e>.1
 target=next(c for c in cyl if c['path']=='/E1Detour/Divider/divider_01');tc=np.asarray(target['position']);targetmember=valid&(np.linalg.norm(w[:,:2]-tc[:2],axis=1)<=target['radius']+.001)&(np.abs(w[:,2]-tc[2])<=target['height']/2+.001)
 scores.append(dict(frame=i,target_GT_selected_points=int(targetmember.sum()),n=n,bad10cm=int(isbad.sum()),both_depth_visible=int(vis.sum()),bad_despite_both_visible=int((isbad&vis).sum()),scene_obstacle=int(membership.sum()),within_true_self55cm=int((valid&(np.linalg.norm(w-gt['root_position_world_m'],axis=1)<.55)).sum())))
 E.append(e);inside.append(membership);good_visibility.append(vis);bad.append(isbad)
e=np.concatenate(E);m=np.concatenate(inside);v=np.concatenate(good_visibility);b=np.concatenate(bad);precision=float(m.mean()) if len(m) else None
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scene_sha256=sha(scene),collision_cylinders=len(cyl),selected_points=len(e),GT_scene_obstacle_precision=precision,semantic_subset95=precision is not None and precision>=.95,source_geometry_decision=source_assessment['decision'],selected_scene_membership_only=True,conditional_gated120_not_original35=True,dense90gate_original_reject_preserved=True,no_free_or_complete3D=True,bad10cm=int(b.sum()),true_both_depth_visible=int(v.sum()),bad_despite_both_visible=int((b&v).sum()),samepixel_depth_to_world_error_factor_exact=int(sum(factor_tests)),GT_reference_certificate_not_upgraded=True,no_extra_physics_or_NN=True,no_new_filter_or_threshold=True,no_navigation_admission=True,whole_research_complete=False)
termination=json.loads((D/'termination.json').read_text());states=json.loads((D/'states.json').read_text());requests=np.load(D/'requests.npz')['requests'];result.update(not_navigation=True,not_original35=True,conditional_gated120_not_original35=False,source_gate=dict(points100=len(e)>=100,metric95=float((e<=.1).mean())>=.95 if len(e) else False,p90=float(np.percentile(e,90))<=.1 if len(e) else False,two100point_exposures=sum(r['n']>=100 for r in scores)>=2),target_exposures100=sum(r['target_GT_selected_points']>=100 for r in scores),first_target_frame=next((r['frame'] for r in scores if r['target_GT_selected_points']>0),None),first_target100_frame=next((r['frame'] for r in scores if r['target_GT_selected_points']>=100),None),first_nonzero_request_frame=next((i for i,a in enumerate(requests) if np.any(a)),None),retained_target_points=sum(r['target_GT_selected_points'] for r in scores),maximum_translation_from_start_m=float(max(np.linalg.norm(np.array(s['position'][:2])-np.array(states[0]['position'][:2])) for s in states)),physical_contact=sum(r['contact'] for r in json.loads((D/'physics_contacts.json').read_text())),termination=termination)
result['acquisition_gates']=dict(source=all(result['source_gate'].values()),target_two100=result['target_exposures100']>=2,contact_free=result['physical_contact']==0,motion_exercised=result['first_nonzero_request_frame'] is not None)
result['decision']='PASS_ONLY_FRONTAL_TARGET_ACQUISITION' if all(result['acquisition_gates'].values()) else 'REJECT_FRONTAL_TARGET_ACQUISITION';result['no_navigation_admission']=True
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');(O/'exposure_scores.json').write_text(json.dumps(scores,indent=2)+'\n');(O/'scene_collision_shapes.json').write_text(json.dumps(cyl,indent=2)+'\n');print(json.dumps(result,indent=2))
