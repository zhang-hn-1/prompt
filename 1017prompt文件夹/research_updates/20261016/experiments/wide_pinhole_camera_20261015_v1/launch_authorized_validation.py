"""A concrete bounded validation, runnable ONLY after explicit additional budget authorization."""
from pathlib import Path
import sys,os,json,hashlib
D=Path(__file__).resolve().parent;C=D.parent/'camera_exposure_contract_20261015_v1';authorization=D/'additional_startup_authorization.json';assert authorization.exists(),'additional startup budget is not yet authorized';a=json.loads(authorization.read_text());assert a.get('human_authorized') is True and a.get('cap')==1
assert not (D/'additional_status.json').exists();sys.path[:0]=[str(D.parent/'stereo_memory_navigation_20261004_v1'),str(C)];from durable_stage_supervision_v1 import run_durable
from completion_guard import require_complete
usd=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310');env={**os.environ,'PYTHONPATH':str(usd)+':/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm','LD_LIBRARY_PATH':str(usd/'bin')+':'+os.environ.get('LD_LIBRARY_PATH',''),'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1','PYTHONUTF8':'1','MPLCONFIGDIR':'/tmp/ball_nav_mpl','TORCH_COMPILE_DISABLE':'1'}
(D/'additional_dispatch_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [authorization,D/'protocol.json',D/'native_probe.py',C/'exposure_contract.py',C/'completion_guard.py']},indent=2)+'\n')
r=run_durable([sys.executable,str(D/'native_probe.py'),'native_validation_run'],D/'additional_status.json',D/'additional.log',timeout=120,grace=20,env=env);print(json.dumps(r),flush=True)
try:require_complete(r,D/'native_validation_run')
except ValueError as e:
 (D/'additional_logical_result.json').write_text(json.dumps(dict(qualified=False,reason=str(e),retry=False,model=0,navigation=0))+'\n')
else:print(json.dumps(run_durable([sys.executable,str(D/'score.py'),'native_validation_run'],D/'additional_score_status.json',D/'additional_score.log',timeout=120,env=env)),flush=True)
