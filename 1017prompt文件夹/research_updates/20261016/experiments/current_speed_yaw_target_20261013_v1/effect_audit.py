from pathlib import Path
import json,math,numpy as np
D=Path(__file__).resolve().parent
from speed_target import feedback
R=D/'candidate_e1_run';rows=json.loads((R/'policy_records.json').read_text());eligible=0;changed=0;bad=[]
for row in rows:
 a=row.get('current_speed_yaw_target')
 if not a:continue
 eligible+=1;desired,c=feedback(a['main_request_rad_s'],a['measured_speed_m_s'],a['curvature_1_m'],a['measured_yaw_rate_unchanged'],a['gain_s'])
 if not np.isclose(desired,a['desired_yaw_rate_rad_s'],rtol=0,atol=1e-12) or not np.isclose(c,a['new_correction_rad'],rtol=0,atol=1e-12):bad.append(row['frame'])
 _,old=feedback(a['main_request_rad_s'],max(a['measured_speed_m_s'],.2),a['curvature_1_m'],a['measured_yaw_rate_unchanged'],a['gain_s'])
 if not np.isclose(old,a['old_correction_rad'],rtol=0,atol=1e-12):bad.append(row['frame'])
 if bool(c!=old)!=a['intervened']:bad.append(row['frame'])
 changed+=int(c!=old)
result=dict(eligible=eligible,logged_target_correction_interventions=changed,arithmetic_errors=bad,pass_gate=bool(eligible and changed and not bad),scope='current tracked requests arithmetic; upstream curvature proxy legacy; real transport separately audited')
(D/'effect_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
