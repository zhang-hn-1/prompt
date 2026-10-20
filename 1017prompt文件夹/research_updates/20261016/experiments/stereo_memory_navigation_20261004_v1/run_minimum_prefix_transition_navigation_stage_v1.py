from pathlib import Path
import json,subprocess,sys,os,datetime,hashlib
H=Path(__file__).resolve().parent;S=H/'minimum_prefix_transition_navigation_status_v1.json';assert not S.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();r=dict(status='running',utc=now(),maximum_actual_sequences=1,not_navigation=False,knownpose_TaskH=True,not_original35=True);S.write_text(json.dumps(r,indent=2)+'\n');env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_frontal_acquisition',TORCH_COMPILE_DISABLE='1')
with (H/'minimum_prefix_transition_navigation_v1.log').open('w') as log:
 try:result=subprocess.run([sys.executable,str(H/'run_minimum_prefix_transition_navigation_v1.py')],stdout=log,stderr=subprocess.STDOUT,env=env,timeout=1200);code=result.returncode
 except subprocess.TimeoutExpired:code=124
D=H/'minimum_prefix_transition_navigation_run_v1';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if D.exists():(D/'artifact_manifest_post_close.json').write_text(json.dumps(dict(utc=now(),artifact_sha256={str(p):sha(p) for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
r.update(status='finished' if code==0 else 'runtime_failed',exit_code=code,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True);sys.exit(code)
