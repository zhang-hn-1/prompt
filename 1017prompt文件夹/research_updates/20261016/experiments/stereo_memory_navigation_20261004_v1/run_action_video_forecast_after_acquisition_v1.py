"""Wait only for already-running bounded acquisitions; exactly one fixed CPU fit, no retries."""
from pathlib import Path
import json,time,sys,datetime,os
from bounded_child_cleanup_v1 import run_bounded
H=Path(__file__).resolve().parent;S=H/'action_video_forecast_stage_status_v1.json';assert not S.exists();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();r=dict(start_utc=now(),status='waiting_existing_acquisitions',no_duplicate_physics=True,maximum_fits=1,review_Critical=0,review_Important=0);S.write_text(json.dumps(r,indent=2)+'\n');deadline=time.monotonic()+2400
while True:
 a=json.loads((H/'action_video_fresh_status_v1.json').read_text())
 if a['status']=='finished':break
 if a['status']!='running' or time.monotonic()>deadline:r.update(status='blocked_by_acquisition_runtime',acquisition_status=a,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');sys.exit(1)
 time.sleep(10)
r.update(status='single_fit_running',fit_start_utc=now());S.write_text(json.dumps(r,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
with (H/'action_video_forecast_fit_validation_v1.log').open('w') as log:result=run_bounded([sys.executable,str(H/'run_action_video_forecast_fit_validation_v1.py')],log,timeout=900,grace=15,env=env)
code=124 if result['timeout'] else result['returncode'];r.update(status='finished' if code==0 else 'runtime_failed',exit_code=code,child_execution=result,end_utc=now());S.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True);sys.exit(code)
