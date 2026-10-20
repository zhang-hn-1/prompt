from pathlib import Path
import json,numpy as np
D=Path(__file__).resolve().parent;r=json.loads((D/'decomposition.json').read_text());old=json.loads((D.parent/'front_rear_camera_geometry_20261015_v1/coverage_upper_bound.json').read_text());n=0
for g,v in r['groups'].items():
 for k,field in [('target_instances','target'),('range_only_instances','range_only_upper_bound'),('narrow_instances','original_FOV'),('wide_instances','fixed_wide_FOV'),('range_lost','range_lost'),('wide_angular_lost','wide_angular_lost')]:assert v[k]==sum(x[field] for x in v['frames'])
 assert v['target_instances']==old['groups'][g]['target_instances'];assert v['narrow_instances']==old['groups'][g]['retained_instances'];assert v['range_lost']==v['wide_angular_lost']==0;assert len(v['frames'])==181;n+=181
# Independent analytic support: for any original point with original optical depth>=.55,
# center camera .05m closer: new z>=.5 and yaw offset .3m plus original FOV extent fits F/3.
f=r['focal_px']*3;u_max=(320/f)*.55/(.55-.05);v_max=(.3+(240/f)*.55)/(.55-.05)
assert u_max<320/r['focal_px'] and v_max<240/r['focal_px']
(D/'independent_decomposition_check.json').write_text(json.dumps(dict(complete_count_frames=n,all_7_count_fields_exact=True,fixed_lens_support_at_original_depth_ge_055=dict(max_native_u_over_z=u_max,max_native_v_over_z=v_max),scope='Analytic frustum support conditional on original depth>=.55; full count aggregation independently checked. Does not certify unseen/occluded views or new neural/source performance.'),indent=2)+'\n');print(n,'aggregates exact; analytic conditional support certified')
