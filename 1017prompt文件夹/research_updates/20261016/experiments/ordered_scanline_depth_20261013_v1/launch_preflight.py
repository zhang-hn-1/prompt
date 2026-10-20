from common import *
import os
from durable_stage_supervision_v1 import run_durable
env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
print(json.dumps(run_durable([sys.executable,str(D/'preflight.py')],D/'preflight_status.json',D/'preflight.log',timeout=600,env=env)))
