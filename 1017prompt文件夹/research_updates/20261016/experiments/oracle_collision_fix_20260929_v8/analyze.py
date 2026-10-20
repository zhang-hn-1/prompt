import json
from pathlib import Path
import numpy as np
h=Path(__file__).resolve().parent
pairs={'pid_0002_contact':('oracle_pid_validation_20260929_v4/navigation_hard_0','hard_0_e1_0002'),'course_0004_contact':('oracle_request_chain_20260929_v6/navigation_course_0004','hard_0_e1_0004'),'golden_e0_contact':('oracle_pid_tuning_20260929_v3/navigation_golden','golden_e0_0001'),'golden_e1_contact':('oracle_pid_tuning_20260929_v3/navigation_golden','golden_e1_0001'),'pid_0002_legacy':('oracle_pid_validation_20260929_v4/navigation_hard_0','hard_0_e1_0002')}
out={}
for case,(oldroot,ep) in pairs.items():
 d=h/case/f'episode_{ep}'
 if not (d/'metrics.json').exists():continue
 m=json.loads((d/'metrics.json').read_text());z=np.load(d/'states.npz');audit=json.loads((d/'collision_audit.json').read_text());samples=audit['samples'];contacts=[s for s in samples if s['contact']];bad=[s for s in samples if not s['available']]
 result={'metrics':m,'state_keys':list(z.files),'samples':len(samples),'unavailable_samples':len(bad),'first_physical_contact':contacts[0] if contacts else None,'last_physical_sample':samples[-1] if samples else None}
 old=h.parent/oldroot/f'episode_{ep}'
 if old.exists():
  o=np.load(old/'states.npz');a=np.load(d/'actions.npz');oa=np.load(old/'actions.npz');parity={}
  for name,new,prev in [('states',z,o),('actions',a,oa)]:
   parity[name]={}
   for k in new.files:
    if k not in prev.files:continue
    x,y=new[k],prev[k]
    if not x.ndim or not y.ndim or x.shape[1:]!=y.shape[1:]:continue
    n=min(len(x),len(y));equal=np.array_equal(x[:n],y[:n],equal_nan=True)
    parity[name][k]={'count':n,'exact':equal}
    if not equal and x.dtype.kind in 'fiu' and n:parity[name][k]['max_abs_delta']=float(np.nanmax(np.abs(x[:n]-y[:n])))
  result['prefix_parity']=parity
 else:result['old_path_missing']=str(old)
 out[case]=result
(h/'results.json').write_text(json.dumps(out,indent=2))
for case,r in out.items():
 print(case,'termination',r['metrics']['termination_reason'],'firstcontact',r['first_physical_contact'],'last',r['last_physical_sample'])
 print('DIFFERENCES', {n:{k:v for k,v in p.items() if not v['exact']} for n,p in r.get('prefix_parity',{}).items()}, r.get('old_path_missing',''))
