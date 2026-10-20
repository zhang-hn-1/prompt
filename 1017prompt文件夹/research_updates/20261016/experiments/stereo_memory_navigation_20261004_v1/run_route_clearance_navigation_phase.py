import datetime,json,os,subprocess,sys
from pathlib import Path
H=Path(__file__).resolve().parent
command=json.loads((H/'local_stopping_status.json').read_text())['command']
command[1]=str(H/'run_route_clearance_navigation.py');command[command.index('--run-root')+1]=str(H/'route_clearance_navigation_run')
assert not (H/'route_clearance_navigation_status.json').exists(),'one-run budget: status exists, inspect rather than rerun'
row=dict(command=command,start_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='running');status=H/'route_clearance_navigation_status.json';status.write_text(json.dumps(row,indent=2)+'\n')
base=H.parent/'visible_surface_connectivity_20261004_v1';paths=[str(H),str(base),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
with (H/'route_clearance_navigation.log').open('w') as log:
 result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=':'.join(paths),OPENBLAS_NUM_THREADS='1',MPLCONFIGDIR='/tmp/mpl_stereo_memory',INFORMATION_MODE='visible',MEMORY_MODE='history',MAP_REPRESENTATION='points'))
row.update(status='finished',exit_code=result.returncode,end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat());status.write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row,indent=2))
if result.returncode:raise RuntimeError('one bounded route clearance simulation failed; retain logs/no rerun')
