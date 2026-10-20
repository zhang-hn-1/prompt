"""Bounded saved-output height diagnosis; GT posthoc only, no gate changes."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;D=H/'fresh_rolling_prior_shadow_current_run';rows=[];total={k:0 for k in ['queries','finite_GT','predicted_hazard','hazard_true_floor','hazard_true_positive_height','hazard_insufficient_GT','hazard_height_not_above_sensitivity','ambiguous_hazard_true_floor','ambiguous_hazard_true_positive_height']}
for f in sorted((D/'sensor_packets').glob('*.npz')):
 p=np.load(f);i=int(p['frame']);g=np.load(D/'offline_truth'/f'frame_{i:03d}.npz');pix=p['query_pixel_id'];y=pix//640;x=pix%640;z=g['left_depth_m'][y,x];T=g['left_camera_usd_world'];rays=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))];truth=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;finite=np.isfinite(truth).all(1);tz=truth[:,2];pz=p['world'][:,2];sigma=p['sigma_m'];hazard=(pz>.05)&(pz<=1.5);floor=finite&(np.abs(tz)<=.05);positive=finite&(tz>.05)&(tz<=1.5);ambiguous=hazard&(pz-sigma<=.05)
 r=dict(frame=i,queries=len(x),finite_GT=int(finite.sum()),predicted_hazard=int(hazard.sum()),hazard_true_floor=int((hazard&floor).sum()),hazard_true_positive_height=int((hazard&positive).sum()),hazard_insufficient_GT=int((hazard&~finite).sum()),hazard_height_not_above_sensitivity=int(ambiguous.sum()),ambiguous_hazard_true_floor=int((ambiguous&floor).sum()),ambiguous_hazard_true_positive_height=int((ambiguous&positive).sum()))
 for k in total:total[k]+=r[k]
 rows.append(r)
result=dict(scope='all60 already sealed current yaw20 source outputs; posthoc GT-height diagnostic only; positive height not environment-object identity; scalar sensitivity not statistical bound; no threshold change/candidate/action',totals=total,rows=rows)
(H/'fresh_current_hazard_height_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(total,indent=2))
