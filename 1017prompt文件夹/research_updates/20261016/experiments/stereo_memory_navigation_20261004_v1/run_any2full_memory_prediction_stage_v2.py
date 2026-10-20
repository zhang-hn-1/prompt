"""One bounded three-nonempty-eye execution; no automatic model retries."""
from pathlib import Path
import subprocess,os,json,time,datetime
H=Path(__file__).resolve().parent;status=H/'any2full_memory_prediction_status_v2.json';assert not status.exists();t=time.monotonic();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();row=dict(start_utc=now(),pid=os.getpid(),stage='STARTED_ONE_CANDIDATE',max_wall_s=600,expected_learned_calls=3,formal_triggered=False,whole_research_complete=False);status.write_text(json.dumps(row,indent=2)+'\n');env=os.environ.copy();env['PYTHONDONTWRITEBYTECODE']='1';env['OPENBLAS_NUM_THREADS']='1'
try:
 with (H/'any2full_memory_prediction_actual_v2.log').open('x') as log:
  r=subprocess.run([os.sys.executable,str(H/'predict_any2full_memory_geometry_v2.py')],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600)
 assert r.returncode==0,'prediction process failed; preserve namespace and log'
 row.update(stage='PREDICTIONS_SEALED_BEFORE_GT',returncode=r.returncode,end_utc=now(),wall_s=time.monotonic()-t);status.write_text(json.dumps(row,indent=2)+'\n')
except Exception as exc:
 row.update(stage='FAILED_PRESERVED_NO_RETRY',failure=type(exc).__name__+': '+str(exc),end_utc=now(),wall_s=time.monotonic()-t);status.write_text(json.dumps(row,indent=2)+'\n');raise
