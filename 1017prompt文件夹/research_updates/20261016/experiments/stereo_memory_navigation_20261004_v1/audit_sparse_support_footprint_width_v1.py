"""One existing-radius MODEL footprint diagnostic, labels only after output seal."""
from pathlib import Path
import json,hashlib,datetime,zipfile,numpy as np
H=Path(__file__).resolve().parent;O=H/'sparse_support_footprint_width_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
domains=[('reverse',H/'local_binocular_refinement_preflight_v1',H/'native_reverse_bank_observation_run_v1'/'estimated_poses',H/'native_reverse_bank_observation_run_v1'/'frames'),('frontal',H/'frontal_local_binocular_refinement_preflight_v1',H/'native_flat_pose_navigation_R1_run_v1'/'estimated_poses',H/'native_flat_pose_navigation_R1_run_v1'/'offline_truth')]
rows={};arrays={};inputs={}
occupancy_path=H/'controlled_obstacle_fixture_v1/occupancy.npz'
metadata=np.load(occupancy_path);origin=metadata['global_origin_xy'];resolution=float(metadata['resolution_m'])
with zipfile.ZipFile(occupancy_path) as archive:
 with archive.open('grid.npy') as stream:
  version=np.lib.format.read_magic(stream);shape=np.lib.format._read_array_header(stream,version)[0]
xx,yy=np.indices(shape);centres=origin+np.stack([xx,yy],axis=-1)*resolution
for name,pred,poses,truth in domains:
 world=[];width=[];keys=[]
 for p in sorted(pred.glob('frame_*.npz')):
  s=np.load(p);i=int(s['frame']);assert float(s['time_s'])<=14.;q=s['query_pixel_id'];y,x=np.divmod(q,640);d=s['refined_disparity_px'][s['keep']];z=584.8888888888889*.6/d;ray=np.column_stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]);w=np.maximum(np.abs(584.8888888888889*.6/(d-1)-z),np.abs(584.8888888888889*.6/(d+1)-z))*np.linalg.norm(ray,axis=1)
  assert len(w)==len(s['world']);world.append(s['world']);width.append(w);keys.extend([(i,int(k)) for k in q]);inputs[str(p)]=sha(p)
 points=np.concatenate(world);widths=np.concatenate(width);cells=np.unique(np.floor((points[:,:2]-origin)/resolution).astype(int),axis=0)
 model=np.zeros(shape,bool)
 for p in points:model|=np.linalg.norm(centres-p[:2],axis=-1)<=.2
 np.savez_compressed(O/(name+'_MODEL_pretruth.npz'),world=points,one_pixel_depth_sensitivity_width_m=widths,unique_point_XY_cells=cells,possible_MODEL_cells=model,origin=origin,resolution_m=resolution,MODEL_radius_m=.2,not_observed_free=True,not_hidden_volume_certificate=True)
 rows[name]=dict(records=len(points),distinct_XY_point_cells=len(cells),MODEL_cells=int(model.sum()),width_median_m=float(np.median(widths)),width_p90_m=float(np.percentile(widths,90)))
 arrays[name]=(points,widths,keys,model)
(O/'pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),inputs_sha256=inputs,output_sha256={str(p):sha(p) for p in O.iterdir()},code_sha256=sha(__file__),existing_radius_only=True,no_occupancy_labels_read=True),indent=2)+'\n')
# Occupancy labels and actual surface positions are read only below this seal.
occ=np.load(H/'controlled_obstacle_fixture_v1/occupancy.npz');print('occupancy keys',occ.files)
# Archive geometry checked independently; grid keys are documented by fixture.
labels=occ['labels'] if 'labels' in occ.files else occ['grid']
occupied=labels.astype(bool)
assert occupied.shape==shape
for name,pred,poses,truth in domains:
 points,widths,keys,model=arrays[name];error=[];cache={}
 for point,(i,q) in zip(points,keys):
  if i not in cache:cache[i]=np.load(truth/f'frame_{i:03d}.npz')
  gt=cache[i];y,x=divmod(q,640);z=float(gt['left_depth_m'][y,x]);T=gt['left_camera_usd_world'];ray=np.array([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,1.]);target=T[:3,3]+T[:3,:3]@np.diag([1,-1,-1])@(ray*z);error.append(np.linalg.norm(point-target) if np.isfinite(z) and z>0 else np.inf)
 e=np.asarray(error);rows[name].update(physical_XY_cells=int(occupied.sum()),physical_XY_covered=int((model&occupied).sum()),physical_XY_coverage=float((model&occupied).sum()/occupied.sum()),errors_within_one_pixel_width=float((e<=widths).mean()),error_to_width_p90=float(np.percentile(e/widths,90)))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='ANALYTIC_ENDPOINT_DISK_FOOTPRINT_NOT_EXACT_RUNTIME_MAPPER_OR_ADMISSION',domains=rows,not_a_yaw_only_AB=True,not_new_admission_gate=True,old_two100_source_rejects_preserved=True,no_pose_uncertainty_certification=True,no_width_radius_bias_fit=True,no_actor_or_navigation=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps(r,indent=2))
