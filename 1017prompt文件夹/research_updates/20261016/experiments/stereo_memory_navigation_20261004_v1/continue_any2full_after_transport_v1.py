"""Finish the authorized one candidate after existing transfer; no retry/no duplicate."""
from pathlib import Path
import subprocess,time,json,datetime,os
H=Path(__file__).resolve().parent;A=H/'any2full_predicted_geometry_preflight_v1';out=H/'any2full_transport_to_prediction_stage_v1.json';assert not out.exists();start=time.monotonic();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();row=dict(start_utc=now(),pid=os.getpid(),stage='WAIT_EXISTING_TRANSPORT_NO_DUPLICATE',transport_session=85006,whole_research_complete=False,formal_triggered=False)
def save():out.write_text(json.dumps(row,indent=2)+'\n')
save()
try:
 while True:
  s=json.loads((A/'transport_recovery_status_v1.json').read_text())
  if s['stage'].startswith('failed'):raise RuntimeError('ONE same-artifact transport recovery failed; no model inference, preserve evidence')
  if s['stage']=='downloaded_verified_no_load':break
  assert time.monotonic()-start<360,'existing transport wait bounded'
  time.sleep(2)
 row.update(stage='RUN_ACTUAL_PREDICTION_ONCE');save();r=subprocess.run([os.sys.executable,str(H/'run_any2full_memory_prediction_stage_v1.py')],timeout=620);assert r.returncode==0,'actual prediction stage failed; preserve output namespace'
 row.update(stage='SEALED_PREDICTIONS_ASSESS_ALL_NONEMPTY');save()
 with (H/'any2full_memory_geometry_assessment_actual_v1.log').open('x') as log:r=subprocess.run([os.sys.executable,str(H/'assess_any2full_memory_geometry_v1.py')],stdout=log,stderr=subprocess.STDOUT,timeout=120)
 assert r.returncode==0,'assessment failed; preserve predictions'
 env=os.environ.copy();env['MPLCONFIGDIR']='/tmp/mpl-ball-navigation'
 with (H/'any2full_memory_geometry_plot_actual_v1.log').open('x') as log:r=subprocess.run([os.sys.executable,str(H/'plot_any2full_memory_geometry_v1.py')],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120)
 assert r.returncode==0,'plot failed; preserve assessment';row.update(stage='ACTUAL_DEVELOPMENT_PREDICTION_AND_ASSESSMENT_CLOSED',end_utc=now(),wall_s=time.monotonic()-start);save()
except Exception as exc:
 row.update(stage='FAILED_PRESERVED_STOP_NO_RETRY',failure=type(exc).__name__+': '+str(exc),end_utc=now(),wall_s=time.monotonic()-start);save();raise
