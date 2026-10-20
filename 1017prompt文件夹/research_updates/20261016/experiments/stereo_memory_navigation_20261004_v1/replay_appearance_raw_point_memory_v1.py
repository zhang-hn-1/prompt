from pathlib import Path
import json,hashlib,copy,numpy as np
from tentative_point_memory import TentativePointMemory
H=Path(__file__).resolve().parent;D=H/'native_fixed_appearance_probe_run_v1';O=H/'fixed_appearance_raw_point_memory_v1';O.mkdir(exist_ok=False);(O/'tentative_memory_snapshots').mkdir();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for f,s in manifest.items():assert sha(f)==s,f
packets=[];inputs={}
for r in json.loads((D/'source_rows.json').read_text()):
 f=D/'partial_observations'/f"frame_{r['frame']:03d}.npz";a=np.load(f);inputs[str(f)]=sha(f)
 for name,digest in r['provenance'].items():assert sha(name)==digest
 packets.append(dict(frame=r['frame'],time_s=r['time_s'] if 'time_s' in r else float(a['acquisition_time_s']),world=a['world'],query_pixel_id=a['query_pixel_id'],partner_pixel_id=a['partner_pixel_id'],input_sha256=r['provenance']))
def serial(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,dict):return {k:serial(x) for k,x in v.items()}
 if isinstance(v,list):return [serial(x) for x in v]
 return v
m=TentativePointMemory();rows=[]
for p in packets:
 m.update(p,p['time_s']);s=serial(m.at(p['time_s']));(O/'tentative_memory_snapshots'/f"frame_{p['frame']:03d}.json").write_text(json.dumps(s)+'\n');rows.append(dict(frame=p['frame'],time_s=p['time_s'],current_count=s['current_count'],historical_count=s['historical_count'],distinct_XY_cells=s['distinct_XY_cells']))
# Prefix loop has no access to future payloads; poison in future objects must be harmless.
for cutoff,p in enumerate(packets):
 for poison in (False,True):
  data=copy.deepcopy(packets)
  if poison:
   for q in data[cutoff+1:]:q.clear();q['FORBIDDEN_FUTURE_PAYLOAD']=float('nan')
  else:data=data[:cutoff+1]
  check=TentativePointMemory()
  for q in data[:cutoff+1]:check.update(q,q['time_s'])
  assert serial(check.at(p['time_s']))==json.loads((O/'tentative_memory_snapshots'/f"frame_{p['frame']:03d}.json").read_text())
(O/'memory_shadow_rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'seal.json').write_text(json.dumps(dict(inputs=inputs,source_sha256={str(H/n):sha(H/n) for n in ['tentative_point_memory.py','replay_appearance_raw_point_memory_v1.py']},outputs={str(f):sha(f) for f in O.rglob('*') if f.is_file()},archive_reconstruction_not_online=True,not_fresh=True,no_truth_label_reads=True),indent=2)+'\n');result=dict(decision='PASS_ARCHIVAL_RAW_POINT_MEMORY_INTERFACE_ONLY',epochs=len(rows),total_birth_point_records=sum(r['current_count'] for r in rows),peak_current=max(r['current_count'] for r in rows),peak_historical=max(r['historical_count'] for r in rows),final=rows[-1],all29_future_truncation_and_payload_poison_exact=True,legacy_source_rejected_tag_means_full_volume_and_consumer_not_admitted=True,partial_point_accuracy_result_unchanged=True,no_action_or_free_or_inflation=True,not_online=True,not_fresh=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
