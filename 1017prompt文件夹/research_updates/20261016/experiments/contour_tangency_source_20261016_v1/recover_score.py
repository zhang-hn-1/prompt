from pathlib import Path
import sys,os,json,time,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
l=json.loads((D/'ledger.json').read_text());seal=json.loads((D/'prediction_seal.json').read_text())
assert l['status']=='execution_failed' and l['recovery_used']==0 and not l['budget_closed']
assert all(sha(D/'predictions'/r['frame'])==r['sha256'] for r in seal['rows'])
assert sha(D/'failed_initial_score/run.py')==seal['source_sha256']['run.py']
assert sha(D/'geometry.py')==seal['source_sha256']['geometry.py']
assert sha(D/'protocol.json')==seal['source_sha256']['protocol.json']
l['recovery_used']=1;l['status']='scoring_recovery';l['recovery_reason']='Only JSON serialization of numpy bool at final result write; no generation or scientific arithmetic change.'
l['recovery_scoring_sha256']=sha(D/'run.py')
(D/'ledger.json').write_text(json.dumps(l,ensure_ascii=False,indent=2)+'\n')
env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
r=run_durable([sys.executable,str(D/'run.py'),'score'],D/'score_recovery_status.json',D/'score_recovery.log',timeout=60,grace=10,env=env)
l['stages'].append(dict(action='score_recovery',terminal=r))
assert all(sha(D/'predictions'/v['frame'])==v['sha256'] for v in seal['rows'])
assert r['child_returncode']==0 and r['status']=='finished'
result=json.loads((D/'result.json').read_text());l['status']='necessary_qualification_passed' if result['necessary_pass'] else 'necessary_qualification_rejected'
l['budget_closed']=True;l['source_admitted']=False
l['next']='Close this contour mechanism before RGB/ML/nav if necessary gate failed; no rescans or old budget reopening.'
(D/'ledger.json').write_text(json.dumps(l,ensure_ascii=False,indent=2)+'\n')
print(l['status'],flush=True)
