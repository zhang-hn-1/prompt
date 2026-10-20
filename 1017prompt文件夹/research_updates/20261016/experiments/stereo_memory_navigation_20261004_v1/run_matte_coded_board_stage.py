import os,sys,subprocess,json,datetime
from pathlib import Path
H=Path(__file__).resolve().parent
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
command=[sys.executable,str(H/'run_matte_coded_board.py')]
row=dict(command=command,start_utc=now(),status='running')
(H/'matte_coded_board_status.json').write_text(json.dumps(row,indent=2)+'\n')
with (H/'matte_coded_board_run.log').open('w') as f:
    result=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
row.update(status='finished',exit_code=result.returncode,end_utc=now())
(H/'matte_coded_board_status.json').write_text(json.dumps(row,indent=2)+'\n')
print('FINISHED',result.returncode,flush=True)
raise SystemExit(result.returncode)
