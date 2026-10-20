from common import *
import os
from durable_stage_supervision_v1 import run_durable
r=json.loads((D/'visuals_review.json').read_text());assert r['render_admitted']
env={**os.environ,'PYTHONUTF8':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
status=run_durable([sys.executable,str(D/'visuals.py')],D/'visuals_reviewed_status.json',D/'visuals_reviewed.log',timeout=300,env=env);print(json.dumps(status),flush=True)
assert status['status']=='finished'
