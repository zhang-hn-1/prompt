from pathlib import Path
import os,sys,json
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parents[1]/'experiments/stereo_memory_navigation_20261004_v1'));from durable_stage_supervision_v1 import run_durable
r=run_durable([sys.executable,str(D/'replay.py')],D/'cpu_status.json',D/'run.log',timeout=900,env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','OPENBLAS_NUM_THREADS':'1'});print(json.dumps(r))
