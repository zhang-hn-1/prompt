from pathlib import Path
import sys,os,json,hashlib,datetime
D=Path(__file__).resolve().parent;W=D.parents[2];H=W/'experiments/stereo_memory_navigation_20261004_v1';sys.path.insert(0,str(H))
from durable_stage_supervision_v1 import run_durable
cfg=json.loads((D/'protocol.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
assert not (D/'ledger.json').exists()
s=dict(dispatch_cap=1,dispatch_used=1,recovery_cap=1,recovery_used=0,source_invocations_cap=2,source_invocations_expected=2,fit=0,physics=0,navigation=0,parent_pid=os.getpid(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_budgets_reopened=False)
(D/'ledger.json').write_text(json.dumps(s,indent=2)+'\n')
SDK=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
r=run_durable([sys.executable,'-B',str(D/'profile.py')],D/'status.json',D/'run.log',timeout=120,grace=15,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH=str(SDK),LD_LIBRARY_PATH=str(SDK/'bin')+':'+os.environ.get('LD_LIBRARY_PATH','')))
s.update(execution=r,status=r['status']);(D/'ledger.json').write_text(json.dumps(s,indent=2)+'\n');assert r['status']=='finished' and r['child_returncode']==0,r
