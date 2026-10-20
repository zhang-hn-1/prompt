"""Historical two sequences annotation transport. No label/actor/truth access."""
from pathlib import Path
import hashlib,json,numpy as np
from paired_prior_memory_v1 import PairedPriorMemory
H=Path(__file__).resolve().parent;O=H/'paired_prior_memory_historical_v1';O.mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();rows=[];inputs={}
cases=[('A1',H/'paired_current_evidence_A1_development_v1',range(0,81,5)),('A2',H/'paired_current_A2_run_v1/current_evidence',range(0,41,5))]
def get(root,f,kind):
 p=root/f'{kind}_packet_{f:03d}.npz';inputs[str(p)]=sha(p);z=np.load(p);v={k:z[k].copy() for k in z.files};v['input_sha256']={str(p):sha(p)};return v
for case,root,frames in cases:
 m=PairedPriorMemory()
 for f in frames:
  a=get(root,f,'eligible');b=get(root,f,'unknown');t=float(a['time_s']);m.update(a,b,t);s=m.at(t)
  for k,p in [('provisional',a),('unknown',b)]:
   record=s[k]['records'][-1];assert np.array_equal(record['world'],p['world']);assert record['time_s']==float(p['time_s']);assert np.array_equal(record['prior_inconsistent'],p['world'][:,2]<0)
  row=dict(case=case,frame=f,acquisition_time_s=t,provisional_current=len(a['world']),unknown_current=len(b['world']),prior_inconsistent_provisional=int(s['provisional']['records'][-1]['prior_inconsistent'].sum()),prior_inconsistent_unknown=int(s['unknown']['records'][-1]['prior_inconsistent'].sum()),historical_provisional=s['provisional']['historical_count'],historical_unknown=s['unknown']['historical_count']);rows.append(row)
  # Serialize all independent timestamps + source hashes and annotation arrays.
  serial=[]
  for layer in ['provisional','unknown']:
   for r in s[layer]['records']:serial.append(dict(layer=layer,frame=r['frame'],time_s=r['time_s'],input_sha256=r['input_sha256'],points=len(r['world']),prior_negative=int(r['prior_inconsistent'].sum())))
  (O/f'{case}_{f:03d}.json').write_text(json.dumps(serial,indent=2)+'\n')
 # Whole-prefix equality, before loading excluded future packet payloads.
 n=PairedPriorMemory()
 for f in frames:
  if f>20:continue
  a=get(root,f,'eligible');b=get(root,f,'unknown');n.update(a,b,float(a['time_s']))
 expected=json.loads((O/f'{case}_020.json').read_text());s=n.at(2.);actual=[]
 for layer in ['provisional','unknown']:
  for r in s[layer]['records']:actual.append(dict(layer=layer,frame=r['frame'],time_s=r['time_s'],input_sha256=r['input_sha256'],points=len(r['world']),prior_negative=int(r['prior_inconsistent'].sum())))
 assert actual==expected
 assert not m.at(float(frames[-1])*.1+10.1)['provisional']['records']
r=dict(scope='HISTORICAL_SEALED_REPLAY_NOT_ONLINE',epochs=len(rows),all26_raw_points_retained=sum(r['provisional_current']+r['unknown_current'] for r in rows)==30174,provisional=sum(r['provisional_current'] for r in rows),unknown=sum(r['unknown_current'] for r in rows),negative_provisional=sum(r['prior_inconsistent_provisional'] for r in rows),negative_unknown=sum(r['prior_inconsistent_unknown'] for r in rows),wholeprefix_truncation_both_exact=True,expiry_both_exact=True,prior_not_observation=True,source_reject_preserved=True,no_actor_and_truth_reads=True,rows=rows)
(O/'verification.json').write_text(json.dumps(r,indent=2)+'\n');(O/'seal.json').write_text(json.dumps(dict(inputs=inputs,outputs={str(p):sha(p) for p in O.iterdir() if p.is_file()},code=sha(__file__),memory=sha(H/'paired_prior_memory_v1.py')),indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='rows'},indent=2))
