"""One fresh actual planner-clearance candidate; historical rejectedC1 is comparator."""
from pathlib import Path
import json,hashlib,datetime,os,sys,subprocess,time
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'tracking_tube_protocol_v1.json').read_text());assert all(sha(f)==s for f,s in p['source_sha256'].items());assert json.loads((H/'single_tracking_tube_preflight_v1.json').read_text())['decision']=='PASS_SINGLE_TRACKING_TUBE_PREFLIGHT_ONLY';suite=json.loads((H/'tracking_tube_suite_evidence_v1.json').read_text());assert suite['passed'] and all(sha(f)==s for f,s in suite['source_sha256'].items());assert sha(suite['log'])==suite['log_sha256'];review=json.loads((H/'tracking_tube_author_review_closed_v1.json').read_text());assert review['unresolved_critical']==0 and review['unresolved_important']==0;from tracking_tube_launch_gate import verify_frozen_review;verify_frozen_review(p,suite,review,sha);status=H/'tracking_tube_stage_status_v1.json';assert not status.exists();r=dict(utc=now(),pid=os.getpid(),stage='RUNNING',maximum_fresh_actual_sequences=1,old_C1_not_fresh=True,model_retention_s=60,only_planner_clearance_changed=True,formal_triggered=False,whole_research_complete=False)
def save():status.write_text(json.dumps(r,indent=2)+'\n')
save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_tracking_tube',OPENBLAS_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1')
with (H/'tracking_tube_D1_actual_v1.log').open('x') as f:
 child=subprocess.Popen([sys.executable,str(H/'run_tracking_tube_D1_v1.py'),'D1'],env=env,stdout=f,stderr=subprocess.STDOUT);r['child_pid']=child.pid;save();begin=time.monotonic()
 try:code=child.wait(timeout=1500)
 except subprocess.TimeoutExpired:
  child.terminate()
  try:child.wait(timeout=30)
  except subprocess.TimeoutExpired:child.kill();child.wait()
  r.update(stage='WALL_LIMIT_PRESERVED_NO_RETRY',end_utc=now());save();raise
r.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 else 'FAILED_PRESERVED_NO_RETRY',returncode=code,wall_s=time.monotonic()-begin,end_utc=now());save();D=H/'tracking_tube_D1_run_v1';manifest={str(f):sha(f) for f in D.rglob('*') if f.is_file()};seal=D/'artifact_manifest_post_close.json';seal.write_text(json.dumps(dict(utc=now(),artifact_sha256=manifest,whole_research_complete=False,formal_triggered=False),indent=2)+'\n');r['post_close_manifest_sha256']=sha(seal);save();assert code==0
