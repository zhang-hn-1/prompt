from pathlib import Path
import sys,json,hashlib,datetime,subprocess,time,os
from probe_cleanup import seal_existing
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
p=json.loads((H/'native_current_motion_protocol_v1.json').read_text())
for group in ['source_sha256','input_sha256']:
 for f,s in p[group].items():assert sha(f)==s,f
assert json.loads((H/'native_flat_applicability_v1/assessment.json').read_text())['decision']=='PASS_HISTORICAL_TASK_H_APPLICABILITY_ONLY'
status=H/'native_current_motion_status_v1.json';assert not status.exists();r=dict(utc=now(),pid=os.getpid(),stage='RUNNING',complete=False,whole_research_complete=False,not_navigation=True,not_fresh_heldout=True)
def save():status.write_text(json.dumps(r,indent=2)+'\n')
save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_native_current',OPENBLAS_NUM_THREADS='1')
with (H/'native_current_motion_actual_v1.log').open('x') as f:
 child=subprocess.Popen([sys.executable,str(H/'run_native_current_motion_v1.py')],env=env,stdout=f,stderr=subprocess.STDOUT);r['child_pid']=child.pid;save();begin=time.monotonic();timeout=False;forced=False
 try:code=child.wait(timeout=900)
 except subprocess.TimeoutExpired:
  timeout=True;child.terminate()
  try:code=child.wait(timeout=30)
  except subprocess.TimeoutExpired:forced=True;child.kill();code=child.wait()
r.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 and not timeout else 'FAILED_PARTIAL_NO_RETRY',returncode=code,wall_s=time.monotonic()-begin,end_utc=now(),complete=code==0 and not timeout,forced_kill=forced,wall_timeout=timeout);save();seal=seal_existing(H/'native_current_motion_run_v1',dict(complete=r['complete'],forced_kill=forced,wall_timeout=timeout));r['manifest_sha256']=sha(seal);save();sys.exit(code)
