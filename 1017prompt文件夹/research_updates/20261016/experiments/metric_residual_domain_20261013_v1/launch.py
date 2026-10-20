from common import *
from durable_stage_supervision_v1 import run_durable
import os
r=json.loads((D/'preflight_review.json').read_text());assert r['admitted']
for p,s in r['source_sha256'].items():assert sha(D/p)==s
with (D/'launch_started.json').open('x') as f:json.dump(dict(review_sha256=sha(D/'preflight_review.json')),f)
env={**os.environ,'PYTHONUTF8':'1','PYTHONDONTWRITEBYTECODE':'1','OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1'}
print(json.dumps(run_durable([sys.executable,str(D/'qualify.py')],D/'qualify_status.json',D/'qualify.log',timeout=1800,env=env)),flush=True)
