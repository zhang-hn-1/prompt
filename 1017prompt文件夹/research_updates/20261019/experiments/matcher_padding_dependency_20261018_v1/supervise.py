from pathlib import Path
import sys,json,os,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/fixed_source_parallel_compute_20261017_v1'))
from group_supervision import run_group
cfg=json.loads((D/'protocol.json').read_text())
for section in ['input_sha256','code_sha256']:
 for p,h in cfg[section].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
assert not (D/'ledger.json').exists()
ledger=dict(dispatch_cap=1,dispatch_used=1,recovery_cap=1,recovery_used=0,source_compute_replays_cap=8,matcher_calls_cap=32,fit=0,physics=0,navigation=0,parent_pid=os.getpid(),status='dispatched')
put=lambda v:(D/'ledger.json').write_text(json.dumps(v,indent=2)+'\n');put(ledger)
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=cfg['sdk'],LD_LIBRARY_PATH=cfg['sdk']+'/bin'+(':'+os.environ['LD_LIBRARY_PATH'] if os.environ.get('LD_LIBRARY_PATH') else ''))
s=run_group([sys.executable,'-B',str(D/'diagnose.py')],D/'status.json',D/'run.log',timeout=120,grace=10,env=env)
ledger.update(status=s['status'],execution=s);put(ledger)
assert s['status']=='finished' and s['child_returncode']==0
