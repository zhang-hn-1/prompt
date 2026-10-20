from common import *
import os,sys
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg=load(D/'protocol.json')
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
assert not (D/'ledger.json').exists()
ledger=dict(dispatch_cap=1,dispatch_used=1,recovery_cap=1,recovery_used=0,fit_cap=1,fit_used=0,
    scored_cell_rows_cap=42091,source_calls=0,physics=0,navigation=0,parent_pid=os.getpid(),old_budgets_reopened=False)
put(D/'ledger.json',ledger)
r=run_group([sys.executable,'-B',str(D/'stage.py')],D/'status.json',D/'run.log',timeout=300,grace=15,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1'))
ledger.update(status=r['status'],execution=r,fit_used=1 if (D/'fit_started.json').exists() else 0);put(D/'ledger.json',ledger)
assert r['status']=='finished' and r['child_returncode']==0,r
