from pathlib import Path
import sys,os,json
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parents[1]/'experiments/stereo_memory_navigation_20261004_v1'));from durable_stage_supervision_v1 import run_durable
r=run_durable([sys.executable,str(D/'fit.py')],D/'fit_status.json',D/'fit.log',timeout=600,env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','OPENBLAS_NUM_THREADS':'1'});print(json.dumps(r))
