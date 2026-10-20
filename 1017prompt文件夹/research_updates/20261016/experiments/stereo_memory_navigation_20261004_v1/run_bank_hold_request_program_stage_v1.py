from pathlib import Path
import sys,json,hashlib,datetime,subprocess,time,os
from probe_cleanup import seal_existing
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
 p=json.loads((H/'bank_hold_request_program_protocol_v1.json').read_text());assert all(sha(f)==s for f,s in p['source_sha256'].items());assert all(sha(f)==s for f,s in p['input_sha256'].items())
 review=json.loads((H/'bank_hold_request_program_author_review_v1.json').read_text());assert review['unresolved_critical']==review['unresolved_important']==0;assert review['driver_sha256']==sha(H/'run_bank_hold_request_program_v1.py') and review['stage_sha256']==sha(__file__)
 status=H/'bank_hold_request_program_stage_status_v1.json';assert not status.exists();r=dict(utc=now(),pid=os.getpid(),stage='RUNNING',arms={},maximum_actual_sequences=2,not_fresh_heldout=True,whole_research_complete=False,formal_triggered=False)
 def save():status.write_text(json.dumps(r,indent=2)+'\n')
 save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_tracking_tube',OPENBLAS_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1')
 for arm in ['B0','C']:
  entry=dict(stage='RUNNING',utc=now(),complete=False);r['arms'][arm]=entry;save()
  with (H/('bank_hold_request_program_'+arm+'_actual_v1.log')).open('x') as f:
   child=subprocess.Popen([sys.executable,str(H/'run_bank_hold_request_program_v1.py'),arm],env=env,stdout=f,stderr=subprocess.STDOUT);entry['child_pid']=child.pid;save();begin=time.monotonic();timeout=False;forced=False
   try:code=child.wait(timeout=1000)
   except subprocess.TimeoutExpired:
    timeout=True;child.terminate()
    try:code=child.wait(timeout=30)
    except subprocess.TimeoutExpired:forced=True;child.kill();code=child.wait()
  entry.update(stage='ACTUAL_CLOSED_NEEDS_VERIFICATION' if code==0 and not timeout else 'FAILED_PARTIAL_EVIDENCE_NO_RETRY',returncode=code,wall_s=time.monotonic()-begin,end_utc=now(),complete=code==0 and not timeout,forced_kill=forced,wall_timeout=timeout);save()
  D=H/('bank_hold_request_program_'+arm+'_run_v1');seal=seal_existing(D,dict(complete=entry['complete'],forced_kill=forced,wall_timeout=timeout));entry['post_close_manifest_sha256']=sha(seal);save()
  if not entry['complete']:r['stage']='FAILED_NO_NEXT_ARM_NO_RETRY';save();raise RuntimeError('closed incomplete arm; inspect preserved evidence')
 r.update(stage='ACTUAL_PAIRED_CLOSED_NEEDS_VERIFICATION',end_utc=now());save()
if __name__=='__main__':main()
