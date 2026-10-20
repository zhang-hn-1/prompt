from pathlib import Path
import json,hashlib,os,sys
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg=json.loads((D/'protocol.json').read_text())
for p,h in cfg['code_sha256'].items(): assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
assert not (D/'ledger.json').exists()
ledger=dict(dispatch_cap=1,dispatch_used=1,recovery_cap=1,recovery_used=0,feature_calls_cap=470,fit=0,source_calls=0,physics=0,navigation=0,parent_pid=os.getpid(),old_budgets_reopened=False)
put=lambda: (D/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
put();r=run_group([sys.executable,'-B',str(D/'qualify.py')],D/'status.json',D/'run.log',timeout=300,grace=15,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'))
ledger.update(status=r['status'],execution=r);put();assert r['status']=='finished' and r['child_returncode']==0,r
