"""Three common physical-state exposures, render/source differences only, no fix."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;B=H/'original_E1_direct_memory_navigation_run_v1';C=H/'original_E1_static_sparse_memory_navigation_run_v1';rows=[]
cb=json.loads((B/'camera_clocks.json').read_text());cc=json.loads((C/'camera_clocks.json').read_text())
for i in [0,155,350]:
 a=np.load(B/'frames'/f'frame_{i:03d}.npz');b=np.load(C/'frames'/f'frame_{i:03d}.npz');r=dict(frame=i,episode_time_s=cb[i]['episode_time_s'],raw_time_delta_s=cc[i]['raw_time_s']-cb[i]['raw_time_s'],eyes={})
 for eye in ['left','right']:
  rgb=np.abs(a[eye+'_rgb'].astype(float)-b[eye+'_rgb'].astype(float));x=a[eye+'_depth_m'];y=b[eye+'_depth_m'];finite=np.isfinite(x)&np.isfinite(y);depth=np.abs(x[finite]-y[finite]);r['eyes'][eye]=dict(RGB_exact=bool(np.array_equal(a[eye+'_rgb'],b[eye+'_rgb'])),changed_RGB_channel_fraction=float((rgb>0).mean()),RGB_absolute_mean=float(rgb.mean()),RGB_absolute_p90=float(np.percentile(rgb,90)),depth_finite_mask_exact=bool(np.array_equal(np.isfinite(x),np.isfinite(y))),GT_depth_exact=bool(np.array_equal(x,y,equal_nan=True)),both_finite_depth_absolute_p90_m=float(np.percentile(depth,90)),GT_camera_pose_exact=bool(np.array_equal(a[eye+'_camera_usd_world'],b[eye+'_camera_usd_world'])))
 rows.append(r)
a=dict(decision='FACTUAL_RENDER_VARIATION_DIAGNOSTIC_ONLY_CAUSE_NOT_IDENTIFIED',rows=rows,no_camera_or_lighting_change=True,no_new_physics_or_NN=True,render_sequence_or_temporal_radiance_hypothesis_not_proven=True,memory_alone_navigation_gain_not_identified=True,whole_research_complete=False)
(H/'original_E1_static_RGB_variation_diagnostic_v1.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
