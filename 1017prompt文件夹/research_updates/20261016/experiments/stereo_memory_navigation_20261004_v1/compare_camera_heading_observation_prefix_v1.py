"""Post-close paired-prefix identity check, never influences either policy."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent
A=H/'active_course_observation_run_v1';B=H/'camera_heading_observation_run_v1'
a=json.loads((A/'policy_records.json').read_text());b=json.loads((B/'policy_records.json').read_text())
event=next((i for i,r in enumerate(b) if r['local']['visibility_probe']['event_time_s'] is not None),len(b))
fields={};first={}
for ns in ['online_inputs','known_poses','history','policy_inputs']:
 count=0;bad=[]
 for i in range(min(event+1,len(a),len(b))):
  aa=np.load(A/ns/f'frame_{i:03d}.npz');bb=np.load(B/ns/f'frame_{i:03d}.npz')
  for k in aa.files:
   if k not in bb.files or not np.array_equal(aa[k],bb[k],equal_nan=True):bad.append(dict(frame=i,field=k))
   else:count+=1
 fields[ns]=dict(equal_fields=count,unequal_fields=len(bad));first[ns]=bad[:5]
statesA=json.loads((A/'states.json').read_text());statesB=json.loads((B/'states.json').read_text());physical_equal=statesA[:event+1]==statesB[:event+1]
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),through_event_frame=event,physical_state_prefix_exact=physical_equal,namespaces=fields,first_mismatches=first,strict_prefix_certified=physical_equal and all(x['unequal_fields']==0 for x in fields.values()),posthoc_only=True,no_future_control_information=True,no_navigation_admission=True)
p=H/'camera_heading_observation_prefix_comparison_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
