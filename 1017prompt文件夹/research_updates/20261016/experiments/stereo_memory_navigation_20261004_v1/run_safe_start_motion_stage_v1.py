"""ONE 8s moving-source sequence; sealed closure, no fit or automatic retry."""
from pathlib import Path
import json,hashlib,subprocess,sys,os,time,datetime
H=Path(__file__).resolve().parent;S=H/'safe_start_motion_status_v1.json';assert not S.exists();sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();p=H/'safe_start_motion_runtime_protocol_v3_metadata.json';c=json.loads(p.read_text());review=json.loads((H/'safe_start_motion_author_review_v2.json').read_text());assert review['critical']==review['important']==0;assert review['driver_sha256']==sha(H/'run_safe_start_motion_v1.py') and review['protocol_sha256']==sha(p);assert c['maximum_actual_sequences']==1 and c['episode_budget_s']==8.;assert list(c['fresh_cases'])==['moving_surface_A1'];assert all(sha(f)==s for f,s in c['source_sha256'].items())
r=dict(utc=now(),stage='RUNNING',parent_pid=os.getpid(),child_pid=None,protocol_sha256=sha(p),actual_whole_sequences_max=1,no_fit=True,no_retry=True,no_navigation_or_source_admission=True)
def save():S.write_text(json.dumps(r,indent=2)+'\n')
save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_safe_start_motion',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1');begin=time.monotonic()
with (H/'safe_start_motion_actual_v1.log').open('x') as log:
 child=subprocess.Popen([sys.executable,str(H/'run_safe_start_motion_v1.py'),'D3','moving_surface_A1'],stdout=log,stderr=subprocess.STDOUT,env=env);r['child_pid']=child.pid;save()
 try:code=child.wait(timeout=900)
 except subprocess.TimeoutExpired:
  child.terminate()
  try:child.wait(timeout=30)
  except subprocess.TimeoutExpired:child.kill();child.wait()
  code=124
D=H/'moving_surface_A1_run_v1'
if D.exists():
 manifest={str(f):sha(f) for f in D.rglob('*') if f.is_file()};seal=D/'artifact_manifest_post_close.json';seal.write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,role='whole8s moving SOURCE-only; no fit or new navigation success',code=code),indent=2)+'\n');r['post_close_seal_sha256']=sha(seal)
r.update(stage='SEALED_SOURCE_SEQUENCE_NOT_ADMISSION' if code==0 else 'FAILED_PRESERVED_NO_RETRY',exit_code=code,wall_s=time.monotonic()-begin,closed_utc=now(),child_pid=None);save();print(json.dumps(r),flush=True);sys.exit(code)
