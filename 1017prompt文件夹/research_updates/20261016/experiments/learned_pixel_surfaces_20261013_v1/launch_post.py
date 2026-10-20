from common import *
import os
from durable_stage_supervision_v1 import run_durable
assert json.loads((D/'factor_review.json').read_text())['diagnostic_admitted']
assert sha(D/'factor_diagnostic.py')==json.loads((D/'factor_review.json').read_text())['source_sha256']
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
env={**os.environ,'PYTHONUTF8':'1','PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
for stage in ['factor_diagnostic','visuals']:
 r=run_durable([sys.executable,str(D/(stage+'.py'))],D/(stage+'_status.json'),D/(stage+'.log'),timeout=1800,env=env);print(json.dumps(r),flush=True)
 if r['status']!='finished':raise SystemExit('post-stage incomplete; no automatic retry')
