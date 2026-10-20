"""Separate appearance-domain diagnostic, original errors and density gates."""
from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
from scipy.spatial import cKDTree
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'native_fixed_appearance_probe_run_v1';B=H/'native_live_point_memory_shadow_run_v1';O=H/'fixed_appearance_render_refined_source_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((H/'native_fixed_appearance_probe_stage_status_v1.json').read_text());assert status['exit_code']==0
assert sha(D/'artifact_manifest_post_close.json')==status['post_close_manifest_sha256']
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s
binding=json.loads((D/'appearance_binding.json').read_text());assert binding['geometry_attributes_before']==binding['geometry_attributes_after'] and binding['original_collision_attributes_unchanged'];assert binding['texture_sha256']==sha(H/'fixed_appearance_fixture_v1/fixed_noise_seed20261008.png')
ph=np.load(D/'physics.npz');old=np.load(B/'physics.npz');fields=['root_position_world_m','root_quaternion_world_wxyz','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','smoothed_target','raw_command_target'];physical={k:np.array_equal(ph[k],old[k]) for k in fields};firsts={}
for k in fields:
 bad=np.flatnonzero(np.any(ph[k]!=old[k],axis=1));firsts[k]=int(bad[0]) if len(bad) else None
request_exact=np.array_equal(np.load(D/'requests.npz')['requests'],np.load(B/'requests.npz')['requests'])
error=[];classes=[];epochs=[];worlds=[];render=[];appearance=np.load(H/'partial_surface_frozen_appearance_v1.npz')
def metric(e):
 e=np.asarray(e,float);return dict(n=len(e),within_0p10m=float((e<=.1).mean()) if len(e) else None,unconditional_p90_m=float(np.sort(e)[int(np.ceil(.9*len(e)))-1]) if len(e) else None,invalid_count=int((~np.isfinite(e)).sum()))
for i in range(0,141,5):
 name=f'frame_{i:03d}.npz';s=np.load(D/'refined_shadow_points'/name);gt=np.load(D/'frames'/name);ogt=np.load(B/'frames'/name);y,x=np.divmod(s['query_pixel_id'],640);z=gt['left_depth_m'][y,x];rays=np.column_stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))]);T=gt['left_camera_usd_world'];truth=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;valid=np.isfinite(z)&(z>0)&np.isfinite(truth).all(1);e=np.linalg.norm(s['world']-truth,axis=1);e[~valid]=np.inf;c=valid&(np.linalg.norm(truth[:,:2],axis=1)<=.351)&(truth[:,2]>.001)&(truth[:,2]<=1.001);error.extend(e.tolist());classes.extend(c.tolist());epochs.append(dict(frame=i,metric=metric(e),class_precision=float(c.mean()) if len(c) else None));worlds.extend(s['world'].tolist())
 rgb=np.load(D/'source_inputs'/name);org=np.load(B/'source_inputs'/name)
 # Offline cylinder-surface mask is only a rendering diagnostic, never actor input.
 yy,xx=np.mgrid[:480,:640];depth=gt['left_depth_m'];ray=np.stack([(xx-319.5)/FOCAL,(yy-239.5)/FOCAL,np.ones_like(xx)],axis=-1);wp=T[:3,3]+(ray*depth[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;mask=np.isfinite(wp).all(2)&(np.linalg.norm(wp[:,:,:2],axis=2)<=.351)&(wp[:,:,2]>.001)&(wp[:,:,2]<=1.001);mask=cv2.erode(mask.astype(np.uint8),np.ones((7,7),np.uint8))>0
 def stats(im):
  gray=cv2.cvtColor(im,cv2.COLOR_RGB2GRAY).astype(np.float32);mu=cv2.boxFilter(gray,-1,(7,7));std=np.sqrt(np.maximum(0,cv2.boxFilter(gray*gray,-1,(7,7))-mu*mu));lab=cv2.cvtColor(im,cv2.COLOR_RGB2LAB).astype(float);cl=np.linalg.norm((lab-appearance['centre'])/appearance['scale'],axis=-1)<=3
  return dict(pixels=int(mask.sum()),median_local_std7=float(np.median(std[mask])) if mask.any() else None,fraction_std7_ge2=float(np.mean(std[mask]>=2)) if mask.any() else None,mean_gray=float(gray[mask].mean()) if mask.any() else None,frozen_colour_class_fraction=float(cl[mask].mean()) if mask.any() else None)
 render.append(dict(frame=i,left_depth_exact=np.array_equal(gt['left_depth_m'],ogt['left_depth_m']),left_pose_exact=np.array_equal(T,ogt['left_camera_usd_world']),RGB_pixels_changed=int(np.any(rgb['left_rgb']!=org['left_rgb'],axis=2).sum()),current_appearance=stats(rgb['left_rgb']),historical_uniform_same_mask=stats(org['left_rgb']),same_state_comparison_qualified=all(physical.values())))
 if i==120:
  image=np.hstack([org['left_rgb'],rgb['left_rgb'],rgb['right_rgb']]);cv2.imwrite(str(O/'frame120_uniform_left_new_left_right.png'),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
m=metric(error);precision=float(np.mean(classes)) if classes else 0.;pose=json.loads((H/'native_fixed_appearance_probe_pose_assessment_v1/assessment.json').read_text());gates=dict(n100=m['n']>=100,metric95=m['within_0p10m'] is not None and m['within_0p10m']>=.95,p90=m['unconditional_p90_m'] is not None and m['unconditional_p90_m']<=.1,class95=precision>=.95,two100_exposures=sum(r['metric']['n']>=100 for r in epochs)>=2,native_pose=all(pose['gates'].values()))
occ=np.load(H/'controlled_obstacle_fixture_v1/occupancy.npz');print('OCC_KEYS',occ.files,flush=True)
# Same 185 proxy XY cells; scalar .2 disk is a MODEL footprint, never volume observation.
grid=occ['labels'] if 'labels' in occ else occ['grid'];origin=occ['origin'] if 'origin' in occ else occ['origin_xy'];res=float(occ['resolution'] if 'resolution' in occ else occ['resolution_m']);reference=origin+np.argwhere(grid>0)*res
w=np.asarray(worlds).reshape(-1,3);distance=cKDTree(w[:,:2]).query(reference)[0] if len(w) else np.full(len(reference),np.inf);coverage=int((distance<=.2+np.sqrt(2)*res/2).sum())
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_NEW_APPEARANCE_SPARSE_POINT_METRIC_ONLY' if all(gates.values()) else 'REJECT_NEW_APPEARANCE_SPARSE_SOURCE',gates=gates,refined_metric=m,refined_obstacle_precision=precision,peak_current_points=max(r['metric']['n'] for r in epochs),physical_fields_exact_vs_uniform=physical,first_physical_differences=firsts,requests_exact_vs_uniform=request_exact,binding_confirmed=True,render_rows=render,epochs=epochs,proxy_XY_MODEL_disk_cells_covered=coverage,proxy_XY_cells_total=len(reference),proxy_XY_MODEL_coverage_fraction=coverage/len(reference),MODEL_radius_m=.2,no_full3D_or_measured_free_or_volume_certificate=True,no_texture_only_causal_identification=True,no_old_uniform_gate_rescue_or_action_admission=True,no_NAV_or_learning_or_formal35=True,whole_research_complete=False,code_sha256=sha(__file__),postseal_sha256=sha(D/'artifact_manifest_post_close.json'))
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['render_rows','epochs']},indent=2))
