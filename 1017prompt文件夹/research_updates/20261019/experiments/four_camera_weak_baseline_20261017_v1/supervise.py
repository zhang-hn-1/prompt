from paths import *
import os,json,hashlib,datetime
from durable_stage_supervision_v1 import run_durable
stage=sys.argv[1];assert stage in ('offline','navigation')
protocol=json.loads((D/'protocol.json').read_text())
for p,h in protocol['source_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h,p
ledger=D/(stage+'_ledger.json');assert not ledger.exists()
state=dict(cap=1,used=1,recovery_cap=1,recovery_used=0,status='reserved',parent_pid=os.getpid(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_budgets_reopened=False)
ledger.write_text(json.dumps(state,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(map(str,PATHS+[SDK])),LD_LIBRARY_PATH=str(SDK/'bin')+':'+os.environ.get('LD_LIBRARY_PATH',''),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',DISPLAY=':0',XDG_RUNTIME_DIR='/run/user/1000',MPLCONFIGDIR='/tmp/mpl_fourweak',TORCH_COMPILE_DISABLE='1')
script='offline.py' if stage=='offline' else 'navigation.py'
r=run_durable([sys.executable,str(D/script)],D/(stage+'_status.json'),D/(stage+'.log'),timeout=900 if stage=='offline' else 2400,grace=45,env=env)
state.update(status=r['status'],execution=r);ledger.write_text(json.dumps(state,indent=2)+'\n')
assert r['status']=='finished' and r['child_returncode']==0,r
