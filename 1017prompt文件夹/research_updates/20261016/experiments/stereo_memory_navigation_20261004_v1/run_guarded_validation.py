import datetime,json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent
P='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm'
command=[sys.executable,str(H/'run_guarded_response.py')]
row=dict(command=command,start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='running')
(H/'guarded_response_status.json').write_text(json.dumps(row,indent=2)+'\n')
with (H/'guarded_response_run.log').open('w') as stream:
    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=P,OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
row.update(status='finished',exit_code=result.returncode,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
(H/'guarded_response_status.json').write_text(json.dumps(row,indent=2)+'\n')
print(json.dumps(row,indent=2))
if result.returncode:raise RuntimeError('sealed whole validation failed at runtime; no automatic rerun')
