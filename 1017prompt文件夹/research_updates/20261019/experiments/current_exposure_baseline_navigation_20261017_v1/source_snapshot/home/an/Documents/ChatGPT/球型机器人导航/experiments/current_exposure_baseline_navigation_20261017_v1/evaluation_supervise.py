from paths import *
import json,hashlib,os,subprocess
from durable_stage_supervision_v1 import run_durable
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert json.loads((D/'navigation_status.json').read_text())['status']=='finished'
assert not (D/'evaluation_ledger.json').exists()
cfg=json.loads((D/'evaluation_protocol.json').read_text())
for p,h in cfg['evaluation_code_sha256'].items():assert sha(p)==h,p
state=dict(cap=1,used=1,recovery_cap=1,recovery_used=0,status='reserved',physics=0,parent_pid=os.getpid());(D/'evaluation_ledger.json').write_text(json.dumps(state,indent=2)+'\n')
r=run_durable([sys.executable,'-B',str(D/'evaluation_stage.py')],D/'evaluation_status.json',D/'evaluation.log',timeout=300,grace=15,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
state.update(status=r['status'],execution=r);(D/'evaluation_ledger.json').write_text(json.dumps(state,indent=2)+'\n');assert r['status']=='finished' and r['child_returncode']==0,r
