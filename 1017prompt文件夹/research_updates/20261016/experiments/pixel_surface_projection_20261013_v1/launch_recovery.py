from common import *
import os
from durable_stage_supervision_v1 import run_durable
ledger=json.loads((D/'ledger.json').read_text());assert ledger['recovery_used']==0
assert json.loads((D/'export_scenes_status.json').read_text())['status']=='runtime_failed'
assert not list((D/'scenes').glob('*.json'))
assert json.loads((D/'axis_recovery_review.json').read_text())['recovery_admitted']
ledger.update(recovery_used=1,interface='exporter assumed explicit upAxis; H has no authored upAxis, reference does not convert coordinates');save('ledger.json',ledger)
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
env={**os.environ,'PYTHONUTF8':'1','PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
for name in ['export_scenes','qualify']:
    suffix='_recovery' if name=='export_scenes' else ''
    r=run_durable([sys.executable,str(D/(name+'.py'))],D/(name+suffix+'_status.json'),D/(name+suffix+'.log'),timeout=1800,env=env);print(json.dumps(r),flush=True)
    if r['status']!='finished':raise SystemExit('no further recovery; stage incomplete')
