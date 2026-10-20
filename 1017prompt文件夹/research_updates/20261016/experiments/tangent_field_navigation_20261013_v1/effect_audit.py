from pathlib import Path
import json,numpy as np
from field import target
D=Path(__file__).resolve().parent;R=D/'candidate_e1_run';rows=json.loads((R/'policy_records.json').read_text());native=json.loads((R/'native_predictions.json').read_text());bytime={round(r['time_s'],7):r for r in native};checked=0;different=0;normal=0;errors=[]
for row in rows:
 a=row.get('tangent_field')
 if not a:continue
 p=np.asarray(a['position']);expected=target(p,a['start'],a['end'],a['lookahead']);new=np.asarray(a['new_target']);ref=np.asarray(row['installed_reference_xy_after_act']);i=a['path_index'];n=bytime[round(row['time_s'],7)]
 try:
  np.testing.assert_allclose(new,expected,atol=1e-12,rtol=0);np.testing.assert_array_equal(a['start'],ref[i]);np.testing.assert_array_equal(a['end'],ref[i+1]);np.testing.assert_array_equal(p,np.asarray(n['position_world'][:2],np.float64))
 except AssertionError:errors.append(row['frame'])
 checked+=1;different+=int(not np.allclose(new,a['original_target'],atol=1e-12,rtol=0));normal+=int(row['tracker_state_after_act']['braking_state']=='tracking')
r=dict(field_calls_checked=checked,changed_targets=different,normal_tracking_field_calls=normal,errors=errors,pass_gate=bool(checked and different and not errors),scope='Actual current native float64 position input and installed reference target arithmetic, not request-level counterfactual benefit; physical request journal separately audited; original lookahead-index remains legacy upstream diagnostic')
(D/'effect_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(r)
