from pathlib import Path
import json,sys,os,hashlib
D=Path(__file__).resolve().parent;W=D.parents[2]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg=json.loads((D/'protocol.json').read_text())
for p,h in cfg['code_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
assert not (D/'ledger.json').exists()
ledger=dict(dispatch_used=1,dispatch_cap=1,recovery_used=0,recovery_cap=0,geometry_nearest_queries=0,model_calls=0,source_calls=0,fit=0,physics=0,navigation=0,parent_pid=os.getpid())
(D/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
r=run_group([sys.executable,'-B',str(D/'analyze.py')],D/'status.json',D/'run.log',timeout=120,grace=10,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'))
ledger.update(status=r['status'],execution=r);(D/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n');assert r['status']=='finished' and r['child_returncode']==0
