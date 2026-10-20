from common import *
import os,datetime
from durable_stage_supervision_v1 import run_durable
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
env={**os.environ,'PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','MPLCONFIGDIR':'/tmp/mpl_ordered'}
assert json.loads((D/'preflight_status.json').read_text())['status']=='finished'
assert json.loads((D/'preflight_review.json').read_text())['offline_admitted']
# Atomic source-generation reservation; no physical run.
with (D/'ledger.json').open('x') as f:json.dump(dict(candidate='ordered_scanline_joint_depth_v1',status='source_generation_registered',utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),physical_cap=4,physical_used=0,fit=0,recovery_used=0,parent_pid=os.getpid(),source_hashes={str(p):sha(p) for p in [D/'protocol.json',D/'census.py',D/'generate.py',D/'common.py']}),f,indent=2)
r=run_durable([sys.executable,str(D/'generate.py')],D/'generation_status.json',D/'generation.log',timeout=7200,env=env)
print(json.dumps(r))
