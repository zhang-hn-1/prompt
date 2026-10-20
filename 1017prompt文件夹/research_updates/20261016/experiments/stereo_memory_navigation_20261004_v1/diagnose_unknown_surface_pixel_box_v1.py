"""Post-evaluation explanatory split, no target selection or candidate modification."""
from pathlib import Path
import json,numpy as np,hashlib
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'unknown_surface_pixel_box_development_v2_checked';rows=[]
for row in json.loads((O/'predictions.json').read_text()):
 f=row['frame'];gt=np.load(D/'frames'/f'frame_{f:03d}.npz');T=gt['left_camera_usd_world'];invalid=black=valid=outside=behind=empty=boundedmiss=0;errors=[]
 for r in row['geometries']:
  q=np.asarray(r['query_xy']);x,y=np.rint(q).astype(int);z=float(gt['left_depth_m'][y,x]);available=np.isfinite(z) and z>0
  if not available:invalid+=1;black+=int(np.all(gt['left_rgb'][y,x]==0));continue
  valid+=1;w=T[:3,3]+(T[:3,:3]@np.diag([1,-1,-1]))@(np.array([(q[0]-319.5)/584.8888888888889,(q[1]-239.5)/584.8888888888889,1.])*z);past=np.load(D/'known_poses'/f"frame_{r['source_frame']:03d}.npz")['left_camera_usd_world'];op=(past[:3,:3]@np.diag([1,-1,-1])).T@(w-past[:3,3])
  if op[2]<=0:behind+=1;continue
  uv=op[:2]/op[2]*584.8888888888889+[319.5,239.5];error=float(np.abs(uv-r['source_xy']).max());errors.append(error);outside+=int(error>1.+1e-6);empty+=int(r['status']=='empty');boundedmiss+=int(r['status']=='bounded' and error>1.+1e-6)
 rows.append(dict(frame=f,missing_GT_depth=invalid,missing_depth_exact_black_query=black,valid_depth=valid,past_true_point_behind_camera=behind,GT_to_matched_past_pixel_maxcoordinate_error_gt1px=outside,empty_polytope_with_valid_label=empty,bounded_polytope_with_source_correspondence_outside_fixedbox=boundedmiss,past_reprojection_error_median_px=float(np.median(errors)) if errors else None,past_reprojection_error_p90_px=float(np.quantile(errors,.9)) if errors else None))
 p=O/'causal_rejection_split_v1.json';p.write_text(json.dumps(dict(rows=rows,no_refit_or_pixelbox_resize_or_target_selection=True,GT_postprediction_and_evaluation_only=True,currentGTcamera_and_clock_verified_exact=True,missing_depth_not_absence_or_free=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
print(json.dumps(rows,indent=2))
