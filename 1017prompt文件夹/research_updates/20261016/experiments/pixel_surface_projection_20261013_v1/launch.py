from common import *
import os,datetime
from durable_stage_supervision_v1 import run_durable
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
env={**os.environ,'PYTHONUTF8':'1','PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
assert json.loads((D/'preflight_review.json').read_text())['offline_admitted']
metadata=dict(phase='offline_pixel_projection_qualification',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),parent_pid=os.getpid(),physical_cap=0,physical_used=0,fit=0,recovery_used=0)
with (D/'ledger.json').open('x') as f:json.dump(metadata,f,indent=2)
for name in ['export_scenes','qualify']:
    r=run_durable([sys.executable,str(D/(name+'.py'))],D/(name+'_status.json'),D/(name+'.log'),timeout=1800,env=env);print(json.dumps(r),flush=True)
    if r['status']!='finished':raise SystemExit('stage incomplete; no performance decision')
