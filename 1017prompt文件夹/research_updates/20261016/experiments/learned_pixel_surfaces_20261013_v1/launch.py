from common import *
import os
from durable_stage_supervision_v1 import run_durable
assert json.loads((Q/'independent_closure.json').read_text())['offline_oracle_review_pass']
assert json.loads((D/'interface_check.json').read_text())['pass_check']
review=json.loads((D/'prefit_review.json').read_text());assert review['prefit_admitted']
for name,s in review['source_sha256'].items():assert sha(D/name)==s
assert not (D/'prepare_status.json').exists()
with (D/'launch_started.json').open('x') as marker:json.dump(dict(review_sha256=sha(D/'prefit_review.json'),source_sha256=review['source_sha256']),marker,indent=2)
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
env={**os.environ,'PYTHONUTF8':'1','PYTHONPATH':str(usd),'LD_LIBRARY_PATH':str(usd/'bin'),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
for stage,timeout in [('prepare',1800),('train_predict',3600),('score',1800)]:
 if stage=='train_predict':
  assert json.loads((D/'coverage.json').read_text())['qualified'];l=json.loads((D/'ledger.json').read_text());assert l['fit_used']==0;l.update(fit_used=1,status='running');save('ledger.json',l)
 r=run_durable([sys.executable,str(D/(stage+'.py'))],D/(stage+'_status.json'),D/(stage+'.log'),timeout=timeout,env=env);print(json.dumps(r),flush=True)
 if r['status']!='finished':raise SystemExit('incomplete; preserve evidence, no automatic recovery')
