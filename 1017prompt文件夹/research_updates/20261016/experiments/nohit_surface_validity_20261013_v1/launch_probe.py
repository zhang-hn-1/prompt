from pathlib import Path
import os,sys,json,datetime
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parents[1]/'experiments/stereo_memory_navigation_20261004_v1'));from durable_stage_supervision_v1 import run_durable
with (D/'ledger.json').open('x') as f:json.dump(dict(cap=4,used=1,fit=0,recovery_used=0,status='probe_reserved',utc=datetime.datetime.now(datetime.timezone.utc).isoformat()),f)
r=run_durable([sys.executable,str(D/'probe.py')],D/'probe_status.json',D/'probe.log',timeout=300,env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'});print(json.dumps(r))
