from pathlib import Path
import json,hashlib,numpy as np
from tentative_point_memory import TentativePointMemory
from live_refined_surface_shadow import refined_surface
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;D=H/'native_reverse_bank_observation_run_v1';R=H/'local_binocular_refinement_preflight_v1';m=TentativePointMemory();cad=RobotSelfMask();counts=[]
for row in json.loads((D/'source_rows.json').read_text()):
 i=row['frame'];name=f'frame_{i:03d}.npz';surface=dict(np.load(D/'partial_observations'/name));rgb=np.load(D/'source_inputs'/name);pose=np.load(D/'estimated_poses'/name);classes=np.load(D/'partial_class_masks'/name);pred=np.load(D/'source_predictions'/name);hist=np.load(D/'history'/name);mask=cad.project(joints_from_proprio(hist['proprio_history']));p=refined_surface(surface,rgb['left_rgb'],rgb['right_rgb'],mask,classes['right'],restore_native(pred['disparity_px']),pose['left_camera_usd_world'],row['time_s'],row['provenance']);old=np.load(R/name)
 for k in ('world','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(p[k],old[k])
 m.update(p,row['time_s']);s=m.at(row['time_s']);counts.append(dict(frame=i,current=s['current_count'],history=s['historical_count'],cells=s['distinct_XY_cells']))
assert sum(r['current'] for r in counts)==451
(H/'live_refinement_adapter_historical_verification_v1.json').write_text(json.dumps(dict(all29_adapter_outputs_exact=True,old451_source_reject_preserved=True,no_labels_or_scores_read=True,memory_counts=counts),indent=2)+'\n');print('29adapteroutputs EXACT,451 retained, no scientific source rescore')
