"""Native output/source appearance attribution only, no candidate selection or rerun."""
from pathlib import Path
import json,datetime,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'mvs_hero_temporal_development_v2_api_scale_recovery';O=H/'qualified_hazard_navigation_run_v1';rows=[]
for r in json.loads((D/'records.json').read_text()):
 f=np.load(D/r['file']);uv=f['current_uv'].astype(int);x,y=uv.T;gt=np.load(O/'offline_truth'/f"frame_{r['frame']:03d}.npz");z=gt[r['eye']+'_depth_m'][y,x];valid=np.isfinite(z);rgb=np.load(O/'online_inputs'/f"frame_{r['frame']:03d}.npz")[r['eye']+'_rgb'];gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY).astype(np.float32);mean=cv2.boxFilter(gray,-1,(7,7));std=np.sqrt(np.maximum(0,cv2.boxFilter(gray*gray,-1,(7,7))-mean*mean));sky=f['sky_diagnostic_only'][y,x];record=dict(frame=r['frame'],eye=r['eye'],n=len(uv),native_sky_flag_points=int((sky>=.5).sum()),native_sky_mask_used_for_admission=False)
 for name,sel in [('current_label_insufficient',~valid),('current_label_finite',valid)]:
  record[name]=dict(points=int(sel.sum()),gray_intensity_median=float(np.median(gray[y[sel],x[sel]])) if sel.any() else None,current_patch_std_median=float(np.median(std[y[sel],x[sel]])) if sel.any() else None,old_std5_domain_below5=int((std[y[sel],x[sel]]<5).sum()),native_sky_flag_ge05=int((sky[sel]>=.5).sum()),native_sky_probability_median=float(np.median(sky[sel])) if sel.any() else None)
 rows.append(record)
p=D/'source_validity_attribution_v1.json';assert not p.exists();p.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,postseal_GT_diagnostic_only=True,depth_infinite_not_truth_sky_class=True,no_native_sky_or_texture_selection_admitted=True,no_RBG_camera_or_model_reexecution=True,old_gate_REJECT_preserved=True),indent=2)+'\n');print(rows)
