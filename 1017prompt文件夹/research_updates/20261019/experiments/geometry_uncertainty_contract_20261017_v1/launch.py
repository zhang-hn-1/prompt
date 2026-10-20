from pathlib import Path
import sys,os,json,datetime,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1];Prior=W/'experiments/radiance_pointwise_qualification_20261016_v1'
sys.path.insert(0,str(W/'experiments/stereo_memory_navigation_20261004_v1'));from durable_stage_supervision_v1 import run_durable
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
assert not (D/'ledger.json').exists(),'No duplicate dispatch'
for p,h in json.loads((Prior/'completed_seal.json').read_text())['files'].items():assert sha(Prior/p)==h,p
sources=json.loads((Prior/'dispatch_identity.json').read_text())['source_sha256'].copy()
for n in ['protocol.json','evaluate.py','launch.py']:sources[str(D/n)]=sha(D/n)
write(D/'dispatch_identity.json',dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sources,prior_completed_seal_sha256=sha(Prior/'completed_seal.json')))
l=dict(status='running',dispatch_used=1,dispatch_cap=1,diagnostic_used=1,diagnostic_cap=1,fit_used=0,fit_cap=0,optimizer_steps=0,physical=0,navigation=0,recovery_used=0,recovery_cap=1,budget_closed=False);write(D/'ledger.json',l)
env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','PYTHONUTF8':'1'}
s=run_durable([sys.executable,str(D/'evaluate.py')],D/'status.json',D/'evaluate.log',timeout=120,grace=10,env=env)
l.update(status='finished' if s['status']=='finished' and s['child_returncode']==0 else 'execution_failed',terminal=s,budget_closed=True);write(D/'ledger.json',l)
