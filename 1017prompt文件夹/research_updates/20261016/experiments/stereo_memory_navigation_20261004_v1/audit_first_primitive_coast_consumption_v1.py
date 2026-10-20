"""Current frame211 actor call; measure intentional coast selection, no physics."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent
s=(H/'verify_first_primitive_direction_prefix_v3.py').read_text();s=s[:s.index('a,row,state=run(')]
ns={'__file__':str(H/'verify_first_primitive_direction_prefix_v3.py')};exec(compile(s,'<causal actor call>','exec'),ns)
a,row,state=ns['run'](211,True)
coast=row['cusp_coast_acquisition'];assert coast['reason']=='empirical_coast_aware_incoming_cusp' and coast['acquire_index']==1
assert abs(coast['remaining_arc_m']-.3)<.002
assert coast['remaining_arc_m']<coast['acquisition_arc_m']
r=dict(decision='PASS_CAUSAL_INTERFACE_EXPLANATION_NOT_INDEX_BUG',frame=211,current_coast_decision=coast,state_after_act=state,actual_request=a[-1].tolist(),first_primitive_length_m=.3,original_acquisition_arc_m=coast['acquisition_arc_m'],first_direction_mask_candidate_rejected=True,not_future_truth=True,not_new_physics=True,not_navigation_admission=True,whole_research_complete=False)
(H/'first_primitive_coast_consumption_audit_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
