from paths import *
import os,json,hashlib,datetime
from durable_stage_supervision_v1 import run_durable
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
nav=json.loads((D/'navigation_protocol.json').read_text())
for p,h in nav['source_sha256'].items():assert sha(p)==h,p
assert not (D/'ledger.json').exists()
state=dict(cap=1,used=1,recovery_cap=1,recovery_used=0,status='reserved',parent_pid=os.getpid(),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_budgets_reopened=False,arm='A',C_physics=0)
(D/'ledger.json').write_text(json.dumps(state,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(map(str,PATHS+[SDK])),LD_LIBRARY_PATH=str(SDK/'bin')+':'+os.environ.get('LD_LIBRARY_PATH',''),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',DISPLAY=':0',XDG_RUNTIME_DIR='/run/user/1000',MPLCONFIGDIR='/tmp/mpl_current_exposure_A',TORCH_COMPILE_DISABLE='1')
r=run_durable([sys.executable,str(D/'navigation.py')],D/'navigation_status.json',D/'navigation.log',timeout=2400,grace=45,env=env)
state.update(status=r['status'],execution=r);(D/'ledger.json').write_text(json.dumps(state,indent=2)+'\n')
assert r['status']=='finished' and r['child_returncode']==0,r
