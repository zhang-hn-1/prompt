"""Postclose causal reconstruction before any candidate geometry label score."""
from pathlib import Path
import numpy as np,json,datetime,hashlib
from qualified_hazard_consumer import QualifiedHazardConsumer
from past_point_rgb_memory import PastPointRGBMemory
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';status=json.loads((H/'qualified_hazard_navigation_status_v1.json').read_text());assert status['status']=='finished' and status['exit_code']==0
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seal=json.loads((O/'artifact_manifest_post_close.json').read_text());assert all(sha(f)==s for f,s in seal['artifact_sha256'].items());states=json.loads((O/'states.json').read_text());rows=json.loads((O/'hazard_qualification_records.json').read_text());c=QualifiedHazardConsumer();m=PastPointRGBMemory();inputs=[];fieldcount=0;guards=0
def same(a,b):
 assert set(a)==set(b)
 for k in a:np.testing.assert_equal(a[k],b[k])
 return len(a)
class Bomb(dict):
 def __getitem__(self,k):
  if k not in ['time_s','current_appearance_time_s']:raise AssertionError('future payload read')
  return super().__getitem__(k)
for r in rows:
 i=r['frame'];t=r['time_s'];src=np.load(O/'sensor_packets'/f'source_{i:03d}.npz');rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz');mask=np.load(O/'masks'/f'frame_{i:03d}.npz');count=int(((src['world'][:,2]>.05)&(src['world'][:,2]<=1.5)).sum());low=i>=4 and all(np.linalg.norm(s['velocity'][:2])<.05 for s in states[i-4:i+1]);q=18-1e-6<=t<24-1e-6 and low and count>=100 and 0<=t-float(src['time_s'])<=.5
 if not m.seeded and q:m.seed(src,rgb,t,qualified=True)
 proof=m.query(rgb,pose,mask,t);saved=np.load(O/'appearance_predictions'/f'frame_{i:03d}.npz');fieldcount+=same(proof,{k:saved[k] for k in saved.files if k not in ['frame','time_s']});current,past=c.consume(src,proof,t,q);fieldcount+=same(current,np.load(O/'qualified_sensor_packets'/f'source_{i:03d}.npz'));fieldcount+=same(past,np.load(O/'appearance_old_packets'/f'source_{i:03d}.npz'))
 assert dict(frame=i,**c.diagnostics)==r
 for future_src,future_proof in [(Bomb(time_s=t+1),None),(src,Bomb(current_appearance_time_s=t+1))]:
  try:c.consume(future_src,future_proof,t,False)
  except ValueError:guards+=1
  else:raise AssertionError('future admitted')
 inputs.append((src,rgb,pose,mask,t,q,current,past,proof))
# Fresh class instances, explicitly no suffix source or sensor rows available.
a=QualifiedHazardConsumer();b=PastPointRGBMemory();prefix=0;prefixfields=0
for src,rgb,pose,mask,t,q,current,past,proof in inputs:
 if t>19.5+1e-6:break
 if not b.seeded and q:b.seed(src,rgb,t,qualified=True)
 p=b.query(rgb,pose,mask,t);cur,old=a.consume(src,p,t,q);prefixfields+=same(p,proof)+same(cur,current)+same(old,past);prefix+=1
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),all_source_updates=len(rows),exact_fields=fieldcount,future_source_and_appearance_rejections=guards,truncated_prefix_updates=prefix,prefix_fields=prefixfields,renderer_labels_read=False,policy_not_recalled=True,source_age_and_identity_not_refreshed=True,archive_wide_future_file_poison=False,old_source_global_REJECT_preserved=True,code_sha256=sha(__file__),postclose_manifest_sha256=sha(O/'artifact_manifest_post_close.json'))
f=H/'qualified_hazard_source_verification_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
