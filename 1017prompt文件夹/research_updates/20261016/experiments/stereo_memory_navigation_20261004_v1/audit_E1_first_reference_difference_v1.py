"""Bounded first historical request mismatch: current/past installed reference identity, no re-act."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_current_pixel_navigation_run_v1';B=H/'minimum_prefix_transition_navigation_run_v1';f=331;new=json.loads((D/'policy_records.json').read_text());old=json.loads((B/'policy_records.json').read_text());result={'scope':'HISTORICAL_REFERENCE_CHAIN_ONLY_NOT_FRESH_AB_OR_NOISE_ISOLATION','frame':f,'rows':[]}
for i in [329,330,331]:
 row={'frame':i}
 for name,data in [('new',new),('old',old)]:
  a=data[i];ref=np.asarray(a['installed_reference_xy_after_act']);row[name]={'version':a['local_reference_version'],'request':a['request'],'reference_points':len(ref),'reference_xy':ref.tolist(),'tracker':a['tracker_state_after_act']}
 result['rows'].append(row)
result['next_question']='Which current source/map change triggered the different reference install at the identical physical pre-fork state? Inspect existing planner metadata/cost and source provenance before proposing one reference consumer change. No new NN/gain/threshold matrix.'
(D/'first_reference_chain_diagnosis_v1.json').write_text(json.dumps(result,indent=2)+'\n');print([(r['frame'],r['new']['version'],r['old']['version']) for r in result['rows']])
