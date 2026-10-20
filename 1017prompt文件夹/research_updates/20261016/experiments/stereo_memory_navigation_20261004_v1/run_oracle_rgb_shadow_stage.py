import os,sys,json,datetime,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
assert not (H/'oracle_rgb_shadow_status.json').exists()
row=dict(start_utc=now(),status='running',maximum_sequences=1)
(H/'oracle_rgb_shadow_status.json').write_text(json.dumps(row,indent=2)+'\n')
with (H/'oracle_rgb_shadow_run.log').open('w') as f:
 r=subprocess.run([sys.executable,str(H/'run_oracle_rgb_shadow.py')],stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory'))
row.update(end_utc=now(),status='finished',exit_code=r.returncode);(H/'oracle_rgb_shadow_status.json').write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row,indent=2));raise SystemExit(r.returncode)
