from pathlib import Path
import json,subprocess,sys,os,datetime,hashlib
from bounded_child_cleanup_v1 import run_bounded
H=Path(__file__).resolve().parent;assert json.loads((H/'E1_current_prefix_navigation_protocol_v1.json').read_text())['execution_admitted'], 'candidate rejected before any actual startup';S=H/'E1_current_prefix_navigation_status_v1.json';assert not S.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();r=dict(status='running',utc=now(),maximum_actual_sequences=1,not_navigation=False,knownpose_TaskH=True,not_original35=True);S.write_text(json.dumps(r,indent=2)+'\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_frontal_acquisition',TORCH_COMPILE_DISABLE='1')
with (H/'E1_current_prefix_navigation_v1.log').open('w') as log:
 result=run_bounded([sys.executable,str(H/'run_E1_current_prefix_navigation_v1.py')],log,timeout=1200,grace=45,env=env)
 code=124 if result['timeout'] else result['returncode']
D=H/'E1_current_prefix_navigation_run_v1';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if D.exists():(D/('artifact_manifest_post_close.json' if code==0 else 'artifact_manifest_after_runtime_failure.json')).write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):sha(p) for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
r.update(child_execution=result,normal_close_confirmed=code==0,status='finished' if code==0 else 'runtime_failed',exit_code=code,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True);sys.exit(code)
