from paths import *
import json,hashlib,os
from durable_stage_supervision_v1 import run_durable
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
cfg=json.loads((D/'trace_protocol.json').read_text())
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
assert not (D/'trace_ledger.json').exists()
s=dict(cap=1,used=1,recovery_cap=0,recovery_used=0,fit=0,physics=0,parent_pid=os.getpid());(D/'trace_ledger.json').write_text(json.dumps(s,indent=2)+'\n')
r=run_durable([sys.executable,'-B',str(D/'trace_diagnosis.py')],D/'trace_status.json',D/'trace.log',timeout=30,grace=5,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1'))
s.update(status=r['status'],execution=r);(D/'trace_ledger.json').write_text(json.dumps(s,indent=2)+'\n');assert r['status']=='finished' and r['child_returncode']==0,r
