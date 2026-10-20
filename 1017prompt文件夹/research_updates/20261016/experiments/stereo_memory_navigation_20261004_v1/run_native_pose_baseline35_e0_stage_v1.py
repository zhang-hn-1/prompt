from pathlib import Path
import json,sys,os,datetime,hashlib
from bounded_child_cleanup_v1 import run_bounded
H=Path(__file__).resolve().parent;S=H/'native_pose_baseline35_e0_status_v1.json'
assert not S.exists()
assert json.loads((H/'sgbm_baseline35_remaining_status_v1.json').read_text())['status']=='finished'
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
r=dict(status='running',start_utc=now(),maximum_actual_sequences=1,role='one pose source substitution current native IMU encoder at registration and navigation; initial gauge privileged');S.write_text(json.dumps(r,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_baseline35',TORCH_COMPILE_DISABLE='1')
with (H/'native_pose_baseline35_e0_v1.log').open('w') as log:
 result=run_bounded([sys.executable,str(H/'run_native_pose_baseline35_e0_v1.py'),'--condition','e0'],log,timeout=1200,grace=45,env=env)
code=124 if result['timeout'] else result['returncode'];D=H/'native_pose_baseline35_e0_run_v1'
if D.exists():(D/('artifact_manifest_post_close.json' if code==0 else 'artifact_manifest_after_runtime_failure.json')).write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
r.update(status='finished' if code==0 else 'runtime_failed',child_execution=result,normal_close_confirmed=code==0,exit_code=code,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True);sys.exit(code)
