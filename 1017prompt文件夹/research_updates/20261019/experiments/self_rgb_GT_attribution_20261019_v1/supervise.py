from pathlib import Path
import json,hashlib,sys,os
D=Path(__file__).resolve().parent;W=D.parents[1];cfg=json.loads((D/'protocol.json').read_text())
for sec in ['input_sha256','code_sha256']:
    for p,h in cfg[sec].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
ledger=D/'ledger.json';assert not ledger.exists()
l=dict(cap=1,used=1,recovery_cap=0,recovery_used=0,parent_pid=os.getpid(),purpose='One independent saved-output postfailure GT attribution; no old budget reopened',status='reserved')
ledger.write_text(json.dumps(l,indent=2)+'\n')
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
s=run_group([sys.executable,'-B',str(D/'attribute.py')],D/'status.json',D/'run.log',timeout=90,grace=10,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
l.update(status=s['status'],execution=s);ledger.write_text(json.dumps(l,indent=2)+'\n');assert s['status']=='finished' and s['child_returncode']==0,s
