from pathlib import Path
import json,numpy as np
from speed_target import feedback
D=Path(__file__).resolve().parent;R=D/'candidate_e1_run';rows=json.loads((R/'policy_records.json').read_text());native=json.loads((R/'native_predictions.json').read_text());bytime={round(r['time_s'],7):r for r in native};bad=[];count=0;below=0;roundonly=0;maxnoise=0;changed=0
for row in rows:
 a=row.get('current_speed_yaw_target')
 if not a:continue
 n=bytime[round(row['time_s'],7)];v=np.array(n['velocity_world_m_s'][:2],np.float32);s32=float(np.linalg.norm(v));s64=float(np.linalg.norm(v.astype(np.float64)));count+=1
 legacy,old=feedback(a['main_request_rad_s'],max(s32,.2),a['curvature_1_m'],a['measured_yaw_rate_unchanged'],a['gain_s']);newdesired,new=feedback(a['main_request_rad_s'],s64,a['curvature_1_m'],a['measured_yaw_rate_unchanged'],a['gain_s'])
 for x,y in [(a['measured_speed_m_s'],s64),(a['legacy_desired_yaw_rate_rad_s'],legacy),(a['old_correction_rad'],old),(a['desired_yaw_rate_rad_s'],newdesired),(a['new_correction_rad'],new)]:
  if not np.isclose(x,y,rtol=0,atol=1e-12):bad.append(row['frame'])
 changed+=int(new!=old)
 if s64<.2:below+=int(new!=old)
 elif new!=old:roundonly+=1;maxnoise=max(maxnoise,abs(new-old))
assert not bad,bad
r=dict(checked=count,logged_changed_corrections=changed,below020_changed_corrections=below,above020_float_precision_changes=roundonly,above020_max_correction_difference_rad=maxnoise,errors=bad,pass_gate=True,scope='recomputed against live-before-action native velocity cast by observation float32, original norm float32 vs audit norm float64; does not prove each final issued command changed; precision discrepancy disclosed, no candidate source edits after run')
(D/'effect_native_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(r)
