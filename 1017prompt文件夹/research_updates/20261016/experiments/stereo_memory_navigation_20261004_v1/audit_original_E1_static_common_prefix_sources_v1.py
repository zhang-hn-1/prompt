from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;B=H/'original_E1_direct_memory_navigation_run_v1';C=H/'original_E1_static_sparse_memory_navigation_run_v1';out=[]
for i in range(0,356,5):
 r=dict(frame=i)
 for folder,fields in [('source_inputs',['left_rgb','right_rgb']),('known_poses',['left_camera_usd_world','right_camera_usd_world','root_position_world_m']),('source_predictions',['depth_m','confidence','disparity_px']),('partial_observations',['world','query_pixel_id','partner_pixel_id'])]:
  a=np.load(B/folder/f'frame_{i:03d}.npz');b=np.load(C/folder/f'frame_{i:03d}.npz')
  for k in fields:
   r[folder+'/'+k]=dict(exact=a[k].shape==b[k].shape and np.array_equal(a[k],b[k],equal_nan=True),shape_B=list(a[k].shape),shape_C=list(b[k].shape))
 out.append(r)
first={k:next((r['frame'] for r in out if not r[k]['exact']),None) for k in out[0] if k!='frame'}
a=dict(decision='PASS_COMMON_REQUEST_PREFIX_SOURCE_TENSORS_EXACT' if all(v is None for v in first.values()) else 'REJECT_SINGLE_CAUSE_MEMORY_AB_SOURCE_EQUIVALENCE',first_differences=first,source_epochs=len(out),physical_request_prefix_exact_does_not_prove_visual_source_identity=True,no_new_physics_or_NN=True,no_old_result_revision=True,whole_research_complete=False)
(H/'original_E1_static_common_prefix_source_equivalence_v1.json').write_text(json.dumps(a,indent=2)+'\n');(H/'original_E1_static_common_prefix_source_rows_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(a,indent=2))
