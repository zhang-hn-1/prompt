from pathlib import Path
import json,hashlib,datetime,numpy as np
from causal_delayed_source_delivery_replay_core_v1 import run
H=Path(__file__).resolve().parent;O=H/'causal_delayed_source_middle_prefixes_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();whole=json.loads((H/'causal_delayed_source_delivery_replay_v1/assessment.json').read_text());assert whole['decision'].startswith('PASS') and whole['first_candidate_request_difference_frame'] is None
files=[H/'causal_delayed_source_delivery_replay_core_v1.py',H/'causal_source_delivery_memory.py',Path(__file__).resolve()];(O/'source_pretest_seal.json').write_text(json.dumps({str(f):sha(f) for f in files},indent=2)+'\n');results=[]
for cutoff in [145,252,503]:
 prefix,_,pf,_=run(True,stop=cutoff,poison=False);changed,_,xf,opened=run(True,stop=cutoff,poison=True);np.testing.assert_array_equal(prefix,changed);assert pf==xf is None;results.append(dict(cutoff=cutoff,navigation_requests=len(prefix),future_or_notyetavailable_source_payload_poison_and_truncate_exact=True,guarded_reads=len(opened)))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_THREE_MIDDLE_PREFIX_DELIVERY_AVAILABILITY_CAUSALITY_ONLY',cutoffs=results,synthetic_fixed75ticks_not_measuredlatency=True,not_another_online_nav_or_asyncworker=True,whole_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
