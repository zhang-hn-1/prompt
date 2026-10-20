from pathlib import Path
import sys,os,json
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D.parent/'stereo_memory_navigation_20261004_v1'))
from durable_stage_supervision_v1 import run_durable
r=run_durable([sys.executable,str(D/'generate.py')],D/'generation_recovery_status.json',D/'generation_recovery.log',timeout=7200,grace=20,env={**os.environ,'PYTHONUTF8':'1','PYTHONDONTWRITEBYTECODE':'1','PYTHONPATH':str(D/'runtime_dependencies')+':'+'/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310','LD_LIBRARY_PATH':'/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310/bin','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
print(json.dumps(r),flush=True)
