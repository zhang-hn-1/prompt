from common import *
import os,sys
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg=load(D/'protocol.json')
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
assert not (D/'ledger.json').exists()
ledger=dict(dispatch_cap=1,dispatch_used=1,recovery_cap=1,recovery_used=0,advisory_generation_cap=1,advisory_sourcepoint_rows_cap=2985832,
    advisory_cell_rows_cap=40052,fit=0,model_calls=0,source_calls=0,physics=0,navigation=0,parent_pid=os.getpid(),old_budgets_reopened=False)
put(D/'ledger.json',ledger)
r=run_group([sys.executable,'-B',str(D/'stage.py')],D/'status.json',D/'run.log',timeout=300,grace=15,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1'))
ledger.update(status=r['status'],execution=r);put(D/'ledger.json',ledger);assert r['status']=='finished' and r['child_returncode']==0,r
