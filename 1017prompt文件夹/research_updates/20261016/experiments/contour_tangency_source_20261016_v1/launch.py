from pathlib import Path
import os, sys, json, hashlib, datetime, time
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
cfg=json.loads((D/'protocol.json').read_text())
assert not (D/'ledger.json').exists(), 'No duplicate dispatch'
sources=[D/'protocol.json',D/'geometry.py',D/'run.py',D/'launch.py',W/'experiments/dual_monocular_semantic_bev_20261015_v1/projection.py',W/'experiments/dual_monocular_semantic_bev_20261015_v1/semantic_projection.py']
identity=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256={str(p):sha(p) for p in sources},capture_seal_sha256=sha(W/cfg['data_root']/'capture_seal.json'),geometry_contract=json.loads((D/'geometry_contract.json').read_text()))
(D/'dispatch_identity.json').write_text(json.dumps(identity,ensure_ascii=False,indent=2)+'\n')
ledger=dict(status='running',physical_used=0,physical_cap=0,ML_fit_used=0,ML_fit_cap=0,model_inference_used=0,navigation_used=0,CPU_used=1,CPU_cap=1,recovery_used=0,recovery_cap=1,budget_closed=False,stages=[],scope=cfg['scope'])
def save():
    p=D/'ledger.tmp';p.write_text(json.dumps(ledger,ensure_ascii=False,indent=2)+'\n');os.replace(p,D/'ledger.json')
save();start=time.monotonic()
env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','PYTHONUTF8':'1'}
for action,timeout in [('prepare',480),('predict',330),('score',60)]:
    assert all(sha(Path(p))==h for p,h in identity['source_sha256'].items())
    r=run_durable([sys.executable,str(D/'run.py'),action],D/(action+'_status.json'),D/(action+'.log'),timeout=timeout,grace=10,env=env)
    ledger['stages'].append(dict(action=action,terminal=r));save()
    if r.get('child_returncode')!=0 or r.get('status')!='finished':
        ledger['status']='execution_failed';ledger['wall_s']=time.monotonic()-start;save();sys.exit(1)
result=json.loads((D/'result.json').read_text())
ledger['status']='necessary_qualification_passed' if result['necessary_pass'] else 'necessary_qualification_rejected'
ledger['budget_closed']=True;ledger['wall_s']=time.monotonic()-start
ledger['source_admitted']=False;ledger['next']='No RGB detector/ML fit/navigation admitted; close candidate on necessary gate failure.'
save();print(json.dumps({k:ledger[k] for k in ['status','wall_s','budget_closed']},ensure_ascii=False),flush=True)
