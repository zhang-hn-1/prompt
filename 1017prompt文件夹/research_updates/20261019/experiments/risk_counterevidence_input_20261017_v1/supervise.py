from common import *
import os,datetime
sys.path.insert(0,str(W/'experiments/stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable
cfg=load(D/'protocol.json')
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
assert not (D/'ledger.json').exists()
state=dict(dispatch_cap=1,dispatch_used=1,recovery_cap=1,recovery_used=0,fit=0,physics=0,source_regeneration=0,status='reserved',parent_pid=os.getpid(),old_budgets_reopened=False,utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
write(D/'ledger.json',state)
r=run_durable([sys.executable,str(D/'stage.py')],D/'status.json',D/'run.log',timeout=300,grace=10,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
state.update(status=r['status'],execution=r);write(D/'ledger.json',state)
assert r['status']=='finished' and r['child_returncode']==0,r
