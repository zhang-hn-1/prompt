from pathlib import Path
import sys,json,datetime,hashlib,os
from bounded_child_cleanup_v1 import run_bounded
H=Path(__file__).resolve().parent;S=H/'endpoint_tail_e1_status_v1.json';assert not S.exists()
assert json.loads((H/'action_video_forecast_fit_validation_v1/assessment.json').read_text())['accepted_forecast_layer_only'] is False
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();r=dict(status='running',start_utc=now(),maximum_actual_sequences=1,no_retry=True,old_forecaster_rejected=True);S.write_text(json.dumps(r,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_baseline35',TORCH_COMPILE_DISABLE='1')
with (H/'endpoint_tail_e1_v1.log').open('w') as log:x=run_bounded([sys.executable,str(H/'run_endpoint_tail_e1_trial_v1.py'),'--condition','e1'],log,timeout=1200,grace=45,env=env)
code=124 if x['timeout'] else x['returncode'];D=H/'endpoint_tail_e1_run_v1'
if D.exists():(D/('artifact_manifest_post_close.json' if code==0 else 'artifact_manifest_after_runtime_failure.json')).write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
r.update(status='finished' if code==0 else 'runtime_failed',exit_code=code,end_utc=now(),child_execution=x);S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True);sys.exit(code)
