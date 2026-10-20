"""GT_ONLY same-query pose substitution attribution; never an actor source."""
from pathlib import Path
import json,numpy as np
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'live_generic_height_navigation_run_v1';O=H/'generic_height_pose_source_attribution_v1';O.mkdir(exist_ok=False);rows=[]
for r in json.loads((D/'source_rows.json').read_text()):
 i=r['frame'];a=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');pose=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');g=np.load(D/'frames'/f'frame_{i:03d}.npz');w=a['world'];q=a['query_pixel_id'];x=q%640;y=q//640;z=g['left_depth_m'][y,x];et=pose['left_camera_usd_world'];gt=g['left_camera_usd_world'];local=(w-et[:3,3])@et[:3,:3];swapped=gt[:3,3]+local@gt[:3,:3].T;original=(w[:,2]>.05)&(w[:,2]<=1.5);new=(swapped[:,2]>.05)&(swapped[:,2]<=1.5);rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))))
 with np.errstate(invalid='ignore'):truth=gt[:3,3]+(rays*z[:,None])@(gt[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>0)&np.isfinite(truth).all(1);obstacle=valid&(np.linalg.norm(truth[:,:2],axis=1)<=.351)&(truth[:,2]>.001)&(truth[:,2]<=1.001);row=dict(frame=i,time_s=r['time_s'],original_selected=int(original.sum()),truepose_selected_same_raw_queries=int(new.sum()),original_false=int((original&~obstacle).sum()),false_removed_by_only_pose=int((original&~obstacle&~new).sum()),true_removed_by_only_pose=int((original&obstacle&~new).sum()),new_false=int((new&~obstacle).sum()),original_invalidGT=int((original&~valid).sum()),false_removed_invalidGT=int((original&~valid&~new).sum()));rows.append(row)
keys=[k for k in rows[0] if k not in ('frame','time_s')];summary={k:sum(r[k] for r in rows) for k in keys};summary.update(scope='GT_ONLY diagnostic SAME raw queries/depth/range/masks; root or pose never repaired in actor',no_new_source_or_threshold=True,no_nav_success_or_unique_cause_claim=True,old_source_pose_nav_rejects_preserved=True)
(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'assessment.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
