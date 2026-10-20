from common import *
import os
from durable_stage_supervision_v1 import run_durable
s=json.loads((D/'generation_recovery_status.json').read_text());assert s['status']=='finished' and s['child_returncode']==0
assert (D/'prediction_seal.json').exists()
env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','MPLCONFIGDIR':'/tmp/mpl_census'}
for stage in ['score','visuals']:
 r=run_durable([sys.executable,str(D/(stage+'.py'))],D/(stage+'_status.json'),D/(stage+'.log'),timeout=1200,env=env);print(json.dumps(r),flush=True)
 if r['status']!='finished':raise SystemExit('analysis execution incomplete; do not classify success')
