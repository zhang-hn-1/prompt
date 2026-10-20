"""Three factual states, postseal local-risk origins; no actor GT or safety upgrade."""
from pathlib import Path
import json,hashlib,numpy as np
from scipy.ndimage import distance_transform_edt
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'original_E1_nonbinding_side_hypothesis_navigation_run_v1';O=H/'E1_nonbinding_side_local_hazard_origin_v2';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(p)==s,p
queries=[535,540,545];(O/'protocol.json').write_text(json.dumps(dict(queries=queries,rule='all previously selected points rasterizing into observed hazard cells within .870711 of current raster cell; birth <= query; labels after output seal',no_filter_or_controller_change=True,not_fresh_heldout=True,post_close_seal=sha(D/'artifact_manifest_post_close.json')),indent=2)+'\n')
rows=json.loads((D/'source_rows.json').read_text());states=json.loads((D/'states.json').read_text());result=[]
for i in queries:
 q=np.load(D/'maps'/f'frame_{i:03d}.npz');origin=q['origin'];res=float(q['resolution']);xy=np.asarray(states[i]['position'])[:2];start=np.rint((xy-origin)/res).astype(int);hazard=np.argwhere(q['labels']==1);dist=np.linalg.norm(hazard-start,axis=1)*res;cells=hazard[dist<.870711];keys={tuple(c) for c in cells};cell_epochs={key:set() for key in keys};birth=[];errors=[];p_gt=[];p_est=[];point_count=0
 for r in rows:
  if r['frame']>i:break
  a=np.load(D/'partial_observations'/f"frame_{r['frame']:03d}.npz");c=np.rint((a['world'][:,:2]-origin)/res).astype(int);mask=np.asarray([tuple(x) in keys for x in c],bool)
  if not mask.any():continue
  for cell in c[mask]:cell_epochs[tuple(cell)].add(r["frame"])
  gt=np.load(D/'frames'/f"frame_{r['frame']:03d}.npz");pose=np.load(D/'known_poses'/f"frame_{r['frame']:03d}.npz");y,x=np.divmod(a['query_pixel_id'][mask],640);z=gt['left_depth_m'][y,x];ray=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))));T=pose['left_camera_usd_world'];w=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(axis=1);e=np.linalg.norm(a['world'][mask]-w,axis=1);e[~valid]=np.inf;errors.extend(e.tolist());p_gt.extend(w[valid].tolist());p_est.extend(a['world'][mask].tolist());birth.append(dict(source_frame=r['frame'],source_time_s=r['time_s'],selected_points=int(mask.sum()),good10cm=int((e<=.1).sum()),invalid_GT=int((~valid).sum())));point_count+=int(mask.sum())
 shapes=json.loads((H/'original_E1_cusp_coast_acquisition_sparse_membership_v1/scene_collision_shapes.json').read_text());nearest=min((np.linalg.norm(xy-np.asarray(c['position'])[:2])-c['radius'],c['path']) for c in shapes)
 e=np.asarray(errors);clearance=distance_transform_edt(np.pad(q['labels']!=1,1))[1:-1,1:-1]*res
 result.append(dict(frame=i,time_s=states[i]['time_s'],current_xy=xy.tolist(),raster_start_clearance_m=float(clearance[tuple(start)]),hazard_cells_inside_planning_tube=int(len(cells)),selected_birth_point_instances=point_count,fraction10cm=float((e<=.1).mean()) if len(e) else None,invalid_GT_points=int(np.isinf(e).sum()),source_births=birth,local_hazard_cell_exposure_counts=[dict(cell=[int(value) for value in key],unique_source_epochs=len(value),source_frames=sorted(value)) for key,value in sorted(cell_epochs.items())],true_cylinder_XY_surface_distance_m=float(nearest[0]),nearest_true_cylinder=nearest[1],GT_cylinder_distance_diagnostic_not_complete3D=True))
a=dict(decision='POSTSEAL_LOCAL_RISK_ORIGIN_DIAGNOSTIC_ONLY',queries=result,no_actual_requests_changed=True,no_complete_3D_safety_certificate=True,no_threshold_or_network_scan=True,no_new_physics_or_NN=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(result,indent=2))
