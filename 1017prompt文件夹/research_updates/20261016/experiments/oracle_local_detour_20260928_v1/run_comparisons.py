import os,sys,subprocess,json,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{root}:{root}/scripts')
paths=list(h.glob('*.py'))+list((h/'scenes').glob('*'))+[h/'PROTOCOL.md',h/'manifest.json',root/'benchmark/policies.py',root/'benchmark/isaac_runtime.py',root/'scripts/action_smoothing.py']
for kind in ['LR','RL']:paths+=list((h/f'{kind}_hold').glob('*'))
(h/'pre_compare_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()},indent=2));status=[]
for kind in ['LR','RL']:
 for mode in ['baseline','candidate']:
  name=kind+'_'+mode;print('START',name,flush=True)
  with (h/(name+'.log')).open('w') as log:r=subprocess.run([sys.executable,str(h/'run_compare.py'),'--kind',kind,'--mode',mode],env=env,stdout=log,stderr=subprocess.STDOUT)
  status.append({'name':name,'returncode':r.returncode});(h/'comparison_status.json').write_text(json.dumps(status,indent=2));print('DONE',name,r.returncode,flush=True)
  if r.returncode:sys.exit(r.returncode)
