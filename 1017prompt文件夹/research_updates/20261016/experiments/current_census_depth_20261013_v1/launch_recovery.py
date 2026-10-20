from common import *
import os,datetime
from durable_stage_supervision_v1 import run_durable
s=json.loads((D/'ledger.json').read_text());assert s['recovery_used']==0;assert json.loads((D/'generation_status.json').read_text())['status']=='runtime_failed';s.update(status='registered_single_interface_recovery',recovery_used=1,parent_pid=os.getpid());save('ledger.json',s)
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310');env={**os.environ,'PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','MPLCONFIGDIR':'/tmp/mpl_census'}
r=run_durable([sys.executable,str(D/'generate.py')],D/'generation_recovery_status.json',D/'generation_recovery.log',timeout=7200,env=env);print(json.dumps(r))
