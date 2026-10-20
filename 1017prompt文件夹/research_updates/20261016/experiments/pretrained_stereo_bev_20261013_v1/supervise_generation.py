from pathlib import Path
import sys,os,json
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D.parent/'stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable
r=run_durable([sys.executable,str(D/'generate.py')],D/'generation_status.json',D/'generation.log',timeout=7200,grace=20,env={**os.environ,'PYTHONUTF8':'1','PYTHONDONTWRITEBYTECODE':'1','PYTHONPATH':str(D/'runtime_dependencies')})
print(json.dumps(r),flush=True)
