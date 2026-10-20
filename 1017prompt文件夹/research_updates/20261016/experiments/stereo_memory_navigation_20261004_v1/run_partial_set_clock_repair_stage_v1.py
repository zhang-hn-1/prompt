"""ONE8s actual source-only sequence, no retries; preserve old task closed."""
from pathlib import Path
import json,hashlib,datetime,os,subprocess,sys,time
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'partial_set_clock_repair_protocol_v1.json').read_text());assert all(sha(f)==v for f,v in p['source_sha256'].items());old=json.loads((H/'static_seed_belief_stage_status_v1.json').read_text());assert old['stage']=='BASELINE_BOOTSTRAP_NOT_EXERCISED_C1_NOT_LAUNCHED';assert (H/'static_seed_belief_B0_collection_verification_v1.json').exists();status=H/'partial_set_clock_repair_stage_status_v1.json';assert not status.exists();r=dict(utc=now(),stage='RUNNING',pid=os.getpid(),maximum_actual_sequences=1,budget_s=8.,wall_limit_s=400,source_only=True,formal_triggered=False,whole_research_complete=False)
def save():status.write_text(json.dumps(r,indent=2)+'\n')
save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_heading_probe',TORCH_COMPILE_DISABLE='1')
with (H/'partial_set_clock_repair_actual_v1.log').open('x') as log:
 child=subprocess.Popen([sys.executable,str(H/'run_partial_set_clock_repair_v1.py'),'B0'],env=env,stdout=log,stderr=subprocess.STDOUT);r['child_pid']=child.pid;save();begin=time.monotonic()
 try:code=child.wait(timeout=400)
 except subprocess.TimeoutExpired:
  child.terminate()
  try:child.wait(timeout=30)
  except subprocess.TimeoutExpired:child.kill();child.wait()
  r.update(stage='FAILED_WALL_LIMIT_PRESERVED',end_utc=now());save();raise
r.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 else 'FAILED_PRESERVED_NO_RETRY',returncode=code,end_utc=now(),wall_s=time.monotonic()-begin);save();assert code==0
D=H/'partial_set_clock_repair_run_v1';m={str(f):sha(f) for f in D.rglob('*') if f.is_file()};seal=D/'artifact_manifest_post_close.json';seal.write_text(json.dumps(dict(utc=now(),artifact_sha256=m,source_only=True,navigation_executed=False,formal_triggered=False,whole_research_complete=False),indent=2)+'\n');r['post_close_manifest_sha256']=sha(seal);save()
