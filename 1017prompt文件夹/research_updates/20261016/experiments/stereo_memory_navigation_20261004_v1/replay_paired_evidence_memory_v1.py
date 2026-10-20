"""Sealed-prefix readonly replay, never an online control validation."""
from pathlib import Path
import hashlib,json,numpy as np
from paired_evidence_memory_v1 import PairedEvidenceMemory
H=Path(__file__).resolve().parent;D=H/'paired_current_A2_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def packet(f,kind):
 p=D/'current_evidence'/f'{kind}_packet_{f:03d}.npz';z=np.load(p);r={k:z[k].copy() for k in z.files};r['input_sha256']={str(q):sha(q) for q in [p,D/'sensor_packets'/f'source_{f:03d}.npz',D/'online_inputs'/f'frame_{f:03d}.npz',D/'masks'/f'frame_{f:03d}.npz'] if q.exists()};return r
def run(frames,stop):
 m=PairedEvidenceMemory()
 for f in frames:
  if f*.1>stop:continue
  m.update(packet(f,'eligible'),packet(f,'unknown'),f*.1)
 return m,m.at(stop)
a,full=run(range(0,41,5),2.);b,truncated=run(range(0,21,5),2.)
def digest(s):
 def conv(v):
  if isinstance(v,np.ndarray):return v.tolist()
  if isinstance(v,np.generic):return v.item()
  raise TypeError(type(v).__name__)
 return hashlib.sha256(json.dumps(s,sort_keys=True,default=conv).encode()).hexdigest()
assert digest(full)==digest(truncated)
m,last=run(range(0,41,5),4.);expiry=m.at(14.1);assert not expiry['provisional']['records'] and not expiry['unknown']['records']
r=dict(scope='SEALED_LOG_REPLAY_NOT_ONLINE_MEMORY',future_truncation_exact=True,future_packets_skipped_before_payload=True,all9_epochs_retained=True,provisional_points=sum(len(v['world']) for v in last['provisional']['records']),unknown_points=sum(len(v['world']) for v in last['unknown']['records']),retains_original_acquisition_times=True,expiry_excludes_all=True,source_reject_kept=True,geometry_and_free_and_actor_admission=False,no_truth_reads=True,code_sha256=sha(__file__),memory_sha256=sha(H/'paired_evidence_memory_v1.py'))
(D/'paired_memory_replay_verification_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
