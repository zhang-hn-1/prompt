from common import *
import os,datetime
from durable_stage_supervision_v1 import run_durable

reg=json.loads((D/'recovery_registration.json').read_text())
assert json.loads((D/'generation_status.json').read_text())['status']=='runtime_failed'
for section in ['preserved_prefix','source_hashes']:
    for p,digest in reg[section].items():assert sha(p)==digest,p
ledger=json.loads((D/'ledger.json').read_text());assert ledger['recovery_used']==0
ledger.update(status='source_generation_recovery_registered',recovery_used=1,parent_pid=os.getpid())
save('ledger.json',ledger)
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
env={**os.environ,'PYTHONUTF8':'1','PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','MPLCONFIGDIR':'/tmp/mpl_ordered'}
print(json.dumps(run_durable([sys.executable,str(D/'generate.py')],D/'generation_recovery_status.json',D/'generation_recovery.log',timeout=7200,env=env)))
