"""One archived GPU transport run; hard parent deadline, no retry."""
from pathlib import Path
import os,sys,json,hashlib,subprocess,time,signal,datetime
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 c=json.loads((H/'spawn_stereo_transport_protocol_v1.json').read_text())
 for group in ('source_sha256','input_sha256'):
  for f,s in c[group].items():assert sha(f)==s,f
 review=json.loads((H/'spawn_stereo_transport_author_review_closed_v1.json').read_text())
 assert review['unresolved_critical']==review['unresolved_important']==0
 assert review['driver_sha256']==sha(H/'run_spawn_stereo_transport_v1.py')
 status=H/'spawn_stereo_transport_stage_status_v1.json';assert not status.exists()
 row=dict(start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage='RUNNING',pid=os.getpid(),maximum_actual_jobs=3,maximum_wall_s=300,no_retry=True,whole_complete=False)
 def save():status.write_text(json.dumps(row,indent=2)+'\n')
 save();env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',TORCH_COMPILE_DISABLE='1')
 with (H/'spawn_stereo_transport_actual_v1.log').open('x') as log:
  child=subprocess.Popen([sys.executable,str(H/'run_spawn_stereo_transport_v1.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  row['child_pid']=child.pid;save();begin=time.monotonic();timeout=False
  try:code=child.wait(timeout=300)
  except subprocess.TimeoutExpired:
   timeout=True;os.killpg(child.pid,signal.SIGTERM)
   try:code=child.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);code=child.wait()
 row.update(exit_code=code,wall_s=time.monotonic()-begin,wall_timeout=timeout,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),stage='CLOSED_NEEDS_VERIFICATION' if code==0 and not timeout else 'FAILED_NO_RETRY');save();print(json.dumps(row,indent=2));raise SystemExit(code if not timeout else 1)
if __name__=='__main__':main()
