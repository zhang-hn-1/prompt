"""One sealed current-RGB-only refinement preflight, development not online."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from local_binocular_refinement import refine
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from stereo_geometry import restore_native,FOCAL
from audit_stereo import summarize
H=Path(__file__).resolve().parent;D=H/'native_reverse_bank_observation_run_v1';O=H/'local_binocular_refinement_preflight_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
protocol=json.loads((H/'local_binocular_refinement_protocol_v1.json').read_text())
for group in ['input_sha256','source_sha256']:
 for f,s in protocol[group].items():assert sha(f)==s,f
review=json.loads((H/'local_binocular_refinement_author_review_v1.json').read_text());assert review['unresolved_critical']==review['unresolved_important']==0
manifest=D/'artifact_manifest_post_close.json'
for f,s in json.loads(manifest.read_text())['artifact_sha256'].items():assert sha(f)==s,f
O.mkdir(exist_ok=False);cad=RobotSelfMask();rows=json.loads((D/'source_rows.json').read_text());proofs=[];inputs={}
for row in rows:
 i=row['frame'];t=row['time_s'];name=f'frame_{i:03d}.npz'
 files=[D/'source_inputs'/name,D/'partial_observations'/name,D/'estimated_poses'/name,D/'partial_class_masks'/name,D/'source_predictions'/name,D/'history'/name,D/'sensor_packets'/f'source_{i:03d}.npz']
 for f in files:inputs[str(f)]=sha(f)
 rgb,s,pose,classes,pred,history,packet=[np.load(f) for f in files]
 assert float(rgb['time_s'])==float(pose['time_s'])==float(packet['time_s'])==float(pred['time_s'])==float(classes['time_s'])==t
 assert int(packet['frame'])==i==int(classes['frame'])==int(pred['frame'])
 y,x=np.divmod(s['query_pixel_id'],640);assert classes['left'].reshape(-1)[s['query_pixel_id']].all()
 masks=cad.project(joints_from_proprio(history['proprio_history']));disparity=restore_native(pred['disparity_px'])[y,x]
 result=refine(rgb['left_rgb'],rgb['right_rgb'],masks['left'],masks['right'],classes['right'],np.column_stack([x,y]),disparity)
 depth=FOCAL*.6/result['disparity_px'];keep=result['keep']&np.isfinite(depth)&(depth>=.5)&(depth<=3)
 rays=np.column_stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))]);T=pose['left_camera_usd_world'];world=T[:3,3]+(rays[keep]*depth[keep,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 partner_y=np.rint(y[keep]-result['disparity_px'][keep]).astype(int)
 np.savez_compressed(O/name,world=world,query_pixel_id=s['query_pixel_id'][keep],partner_pixel_id=partner_y*640+x[keep],raw_query_pixel_id=s['query_pixel_id'],keep=keep,refined_disparity_px=result['disparity_px'],score=result['score'],uniqueness_gap=result['uniqueness_gap'],frame=i,time_s=t,source_kind='CURRENT_RGB_PATCH_REFINED_PROVISIONAL',certified_free=False,source_model_admitted=False)
 proofs.append(dict(frame=i,time_s=t,raw_points=len(x),retained=int(keep.sum())))
(O/'point_proofs.json').write_text(json.dumps(proofs,indent=2)+'\n')
(O/'pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,source_sha256=protocol['source_sha256'],protocol_sha256=sha(H/'local_binocular_refinement_protocol_v1.json'),output_sha256={str(f):sha(f) for f in O.iterdir()},current_RGB_and_supplied_estimated_pose_only=True,no_future_exposure=True,not_free=True),indent=2)+'\n')
# Independent post-seal labels only. No candidate selection or corrected actor output.
errors=[];truth_classes=[];metrics=[]
for row in proofs:
 i=row['frame'];s=np.load(O/f'frame_{i:03d}.npz');gt=np.load(D/'frames'/f'frame_{i:03d}.npz');y,x=np.divmod(s['query_pixel_id'],640);z=gt['left_depth_m'][y,x];ray=np.column_stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))]);T=gt['left_camera_usd_world'];truth=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;valid=np.isfinite(z)&(z>0)&np.isfinite(truth).all(1);e=np.linalg.norm(s['world']-truth,axis=1);e[~valid]=np.inf
 obstacle=valid&(np.linalg.norm(truth[:,:2],axis=1)<=.351)&(truth[:,2]>.001)&(truth[:,2]<=1.001)
 errors.append(e);truth_classes.append(obstacle);metrics.append(dict(frame=i,n=len(e),metric=summarize(e)))
e=np.concatenate(errors);c=np.concatenate(truth_classes);m=summarize(e);precision=float(c.mean()) if len(c) else 0.;pose=json.loads((H/'native_reverse_bank_observation_pose_assessment_v1/assessment.json').read_text())
gates=dict(n=len(e)>=100,metric95=m['within_0p10m'] is not None and m['within_0p10m']>=.95,p90=m['finite_p90_m'] is not None and m['finite_p90_m']<=.1,class95=precision>=.95,two100point_exposures=sum(r['n']>=100 for r in metrics)>=2,native_pose=all(pose['gates'].values()))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_LOCAL_BINOCULAR_DEVELOPMENT_ONLY' if all(gates.values()) else 'REJECT_LOCAL_BINOCULAR_REFINEMENT',gates=gates,raw_n=sum(p['raw_points'] for p in proofs),retained_n=len(e),world_metric=m,obstacle_precision=precision,frames=metrics,seen_development_not_fresh=True,old_raw_axis_source_and_support_rejects_preserved=True,no_online_actor_or_navigation_admission=True,no_threshold_window_network_motion_retry=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='frames'},indent=2))
