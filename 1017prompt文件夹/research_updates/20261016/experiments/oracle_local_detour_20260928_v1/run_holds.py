import os,sys,subprocess,json,hashlib
from pathlib import Path
h=Path(__file__).resolve().parent;root='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm'
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',MPLCONFIGDIR='/tmp/ball_wm_mpl',PYTHONPATH=f'/home/an/.cache/ball_wm_visibility_numba_0612:{root}:{root}/scripts');paths=[h/'run_hold.py',h/'PROTOCOL.md',h/'manifest.json'];(h/'collection_identity.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},indent=2));status=[]
for kind in ['LR','RL']:
 print('START',kind,flush=True)
 with (h/f'{kind}_hold.log').open('w') as log:r=subprocess.run([sys.executable,str(h/'run_hold.py'),'--kind',kind],env=env,stdout=log,stderr=subprocess.STDOUT)
 status.append({'name':kind+'_hold','returncode':r.returncode});(h/'collection_status.json').write_text(json.dumps(status,indent=2));print('DONE',kind,r.returncode,flush=True)
 if r.returncode:sys.exit(r.returncode)
