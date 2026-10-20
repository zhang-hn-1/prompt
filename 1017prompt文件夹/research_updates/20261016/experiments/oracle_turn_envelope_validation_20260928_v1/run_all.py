import os,sys,subprocess,json,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{root}:{root}/scripts')
paths=list(h.glob('*.py'))+[h/'PROTOCOL.md',root/'benchmark/isaac_runtime.py',root/'scripts/action_smoothing.py']
for kind in ['LR','RL']:paths+=list((h.parent/'oracle_measured_turn_primitives_20260928_v1'/f'{kind}_measure').glob('*.npz'))+list((h.parent/'oracle_measured_turn_primitives_20260928_v1'/f'{kind}_measure').glob('*.json'))
(h/'pre_run_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},indent=2))
status=[]
for kind in ['LR','RL']:
 print('START',kind,flush=True)
 with (h/(kind+'.log')).open('w') as log:r=subprocess.run([sys.executable,str(h/'run_one.py'),'--kind',kind],env=env,stdout=log,stderr=subprocess.STDOUT)
 status.append({'kind':kind,'returncode':r.returncode});(h/'run_status.json').write_text(json.dumps(status,indent=2));print('DONE',kind,r.returncode,flush=True)
 if r.returncode:sys.exit(r.returncode)
