import os,sys,subprocess,json,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent
root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{root}:{root}/scripts')
paths=list(h.glob('*.py'))+[h/'PROTOCOL.md',root/'benchmark/isaac_runtime.py',root/'scripts/action_smoothing.py',root/'configs/benchmark_v1/drive_dynamics_v1.json']
(h/'pre_run_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},indent=2))
status=[]
for repeat in [False,True]:
 if repeat:
  files=[p for kind in ['LR','RL'] for p in (h/(kind+'_measure')).glob('*') if p.is_file()]
  (h/'frozen_templates.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
 for kind in ['LR','RL']:
  name=kind+('_repeat' if repeat else '_measure');cmd=[sys.executable,str(h/'run_one.py'),'--kind',kind]+(['--repeat'] if repeat else [])
  print('START',name,flush=True)
  with (h/(name+'.log')).open('w') as log:r=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
  status.append({'name':name,'returncode':r.returncode});(h/'run_status.json').write_text(json.dumps(status,indent=2))
  print('DONE',name,r.returncode,flush=True)
  if r.returncode:sys.exit(r.returncode)
