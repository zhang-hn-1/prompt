"""Future-stage helper. No restart; saves PID before wait and interruption after cleanup.
SIGKILL/power loss cannot be caught: a running record then requires external reconciliation.
"""
from pathlib import Path
import subprocess,signal,json,datetime,os
class ParentInterrupted(Exception):
 def __init__(self,signum):self.signum=signum

def run_durable(args,status_path,log_path,*,timeout,grace=45,env=None):
 status_path=Path(status_path);log_path=Path(log_path)
 if status_path.exists():raise FileExistsError('never silently restart an existing stage')
 now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
 result=dict(status='starting',start_utc=now(),parent_pid=os.getpid(),child_pid=None,no_retry=True,timeout=False,forced_kill=False,parent_signal=None,child_returncode=None)
 def save():
  tmp=status_path.with_name(status_path.name+'.tmp');tmp.write_text(json.dumps(result,indent=2)+'\n');os.replace(tmp,status_path)
 def interrupt(signum,frame):raise ParentInterrupted(signum)
 old={s:signal.getsignal(s) for s in (signal.SIGTERM,signal.SIGINT)}
 p=None;save()
 try:
  for s in old:signal.signal(s,interrupt)
  with log_path.open('w') as log:
   p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,env=env)
   result.update(status='running',child_pid=p.pid);save()
   try:p.wait(timeout=timeout)
   except subprocess.TimeoutExpired:result.update(timeout=True,status='timeout')
   except ParentInterrupted as e:result.update(parent_signal=e.signum,status='interrupted')
   if p.poll() is None:
    # Prevent a second termination signal from aborting bounded cleanup.
    for s in old:signal.signal(s,signal.SIG_IGN)
    p.terminate()
    try:p.wait(timeout=grace)
    except subprocess.TimeoutExpired:result['forced_kill']=True;p.kill();p.wait(timeout=10)
   result['child_returncode']=p.returncode
   if result['status']=='running':result['status']='finished' if p.returncode==0 else 'runtime_failed'
 except ParentInterrupted as e:
  result.update(status='interrupted',parent_signal=e.signum)
 except Exception as e:
  result.update(status='runtime_failed',failure_type=type(e).__name__,failure_message=str(e))
  raise
 finally:
  try:
   if p is not None and p.poll() is None:
    p.kill();p.wait(timeout=10);result.update(forced_kill=True,child_returncode=p.returncode)
   result['end_utc']=now();save()
  finally:
   for s,h in old.items():signal.signal(s,h)
 return result
